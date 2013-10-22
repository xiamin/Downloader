
/*
 * RT-Thread SD Card Driver
 * 20100715 Bernard support SDHC card great than 4G.
 */
#include <rtthread.h>
#include <dfs_fs.h>

/* Private typedef
-----------------------------------------------------------*/
#define SPI_FLASH_PageSize    0x100
#define SPI_FLASH_ChipSize	  0x100000

/* Private define
------------------------------------------------------------*/
#define WRITE      0x02  /* Write to Memory instruction */
#define WRSR       0x01  /* Write Status Register instruction */
#define WREN       0x06  /* Write enable instruction */
#define WRDIS       0x04  /* Write disable instruction */
#define WEVSR     0x50
#define READ       0x03  /* Read from Memory instruction */
#define RDSR       0x05  /* Read Status Register instruction  */
#define RDID       0x9F  /* Read identification */
#define PE	    0xD8
#define SE         0x20  /* Sector Erase instruction */
#define BE         0xC7  /* Bulk Erase instruction */

#define WIP_Flag   0x01  /* Write In Progress (WIP) flag */

//#define Dummy_Byte 0xA5
#define Dummy_Byte 0x00

#define GPIO_SPISOFT1_DO        GPIOA
#define GPIO_SPISOFT1_DO_pin    GPIO_Pin_7
#define GPIO_SPISOFT1_SCK       GPIOA
#define GPIO_SPISOFT1_SCK_pin   GPIO_Pin_5
#define GPIO_SPISOFT1_DI        GPIOA
#define GPIO_SPISOFT1_DI_pin    GPIO_Pin_6

#define GPIO_FLASH1_CS                  GPIOA
#define RCC_APB2Periph_GPIO_FLASH1_CS   RCC_APB2Periph_GPIOA
#define GPIO_Pin_FLASH1_CS              GPIO_Pin_4 

/* Select SPI FLASH: Chip Select pin low  */
#define SPI_FLASH_CS_LOW()       GPIO_ResetBits(GPIO_FLASH1_CS, GPIO_Pin_FLASH1_CS)

/* Deselect SPI FLASH: Chip Select pin high */
#define SPI_FLASH_CS_HIGH()      GPIO_SetBits(GPIO_FLASH1_CS, GPIO_Pin_FLASH1_CS)

void SPI_FLASH_BufferWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void SPI_FLASH_BufferRead(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
uint32_t drv_FlashChipReadDword(uint32_t dwOffset);
void drv_FlashChipWriteDatas(uint8_t *pData);

void drv_FlashChipRead(uint8_t * pdata);
uint32_t SPI_FLASH_ReadID(void);
void SPI_FLASH_StartReadSequence(uint32_t ReadAddr);
void SPI_FLASH_WriteEnable();
void SPI_FLASH_WaitForWriteEnd(void);
void SPI_FLASH_WriteStatus(uint16_t wStatus);
uint8_t SPI_FLASH_ReadStatus(uint8_t Status);
void SPI_FLASH_WriteDisable(void);
void SPI_FLASH_ProtectSW(uint8_t sw);

void Spi_Soft_Init()
{
    GPIO_InitTypeDef  GPIO_InitStructure;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

        {
            SPI_InitTypeDef SPI_InitStruct;

            RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

            /* Configure SPI1 pins: DO SCK  and DI */
            GPIO_InitStructure.GPIO_Pin =  GPIO_SPISOFT1_DO_pin;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//Out_PP;
            GPIO_Init(GPIO_SPISOFT1_DO, &GPIO_InitStructure);

            GPIO_InitStructure.GPIO_Pin =  GPIO_SPISOFT1_SCK_pin;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//Out_PP;
            GPIO_Init(GPIO_SPISOFT1_SCK, &GPIO_InitStructure);

            GPIO_InitStructure.GPIO_Pin =  GPIO_SPISOFT1_DI_pin;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//IN_FLOATING;
            GPIO_Init(GPIO_SPISOFT1_DI, &GPIO_InitStructure);

            SPI_StructInit(&SPI_InitStruct);
            SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
            SPI_InitStruct.SPI_CPHA=SPI_CPHA_1Edge;
            SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
            SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
            SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
            SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
            SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
            SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
            SPI_InitStruct.SPI_CRCPolynomial = 7;
            SPI_Init(SPI1, &SPI_InitStruct);
            //SPI_SSOutputCmd(SPI1, ENABLE);
            //SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Reset);
            SPI_Cmd(SPI1, ENABLE);
        }
}

uint8_t Spi_WriteReadData(uint8_t data)
{
    while(RESET == SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE));
    SPI_I2S_SendData(SPI1, data);

    while(RESET == SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
    return SPI_I2S_ReceiveData(SPI1);
}

void Spi_Soft_WriteByte(uint8_t byData)
{
        Spi_WriteReadData(byData);
}

uint8_t Spi_Soft_ReadByte()
{
        return Spi_WriteReadData(0xFF);
}

/* set sector size to 512 */
#define SECTOR_SIZE		512

uint8_t SPI_FLASH_GetByte(void)
{
    return (Spi_Soft_ReadByte());
}

/**
  * @brief  Sends a byte through the SPI interface and return the byte
  *   received from the SPI bus.
  * @param byte : byte to send.
  * @retval : The value of the received byte.
  */
void SPI_FLASH_SendByte(uint8_t byte)
{
    /* Send byte through the SPI1 peripheral */
    Spi_Soft_WriteByte(byte);
}

/**
  * @brief  Initializes the peripherals used by the SPI FLASH driver.
  * @param  None
  * @retval : None
  */
void SPI_FLASH_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    uint8_t State1;
    Spi_Soft_Init();

    /* Enable SPI1 and GPIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_FLASH1_CS, ENABLE);

    /* Configure I/O for Flash Chip select */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_FLASH1_CS;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIO_FLASH1_CS, &GPIO_InitStructure);

    /* Deselect the FLASH: Chip Select high */
    SPI_FLASH_CS_HIGH();
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    GPIO_SetBits(GPIOC, GPIO_Pin_13);     //flash写使能
    
    SPI_FLASH_ProtectSW(0);	//写使能
    #if 0
    State1 =  SPI_FLASH_ReadStatus(RDSR);
    if((State1 &0x9c)!=0x9c)
    {
        SPI_FLASH_WriteStatus(0x009c);		/* 使用软件保护模式，写的时候才使能 modify by FJD 20120905*/
    }
    #endif
}


/**
  * @brief  Erases the specified FLASH sector.
  * @param SectorAddr: address of the sector to erase.
  * @retval : None
  */
void SPI_FLASH_SectorErase(uint32_t SectorAddr)
{
    /* Send write enable instruction */
    SPI_FLASH_WriteEnable();

    /* Sector Erase */
    /* Select the FLASH: Chip Select low */
    SPI_FLASH_CS_LOW();
    /* Send Sector Erase instruction */
    SPI_FLASH_SendByte(SE);
    /* Send SectorAddr high nibble address byte */
    SPI_FLASH_SendByte((SectorAddr & 0xFF0000) >> 16);
    /* Send SectorAddr medium nibble address byte */
    SPI_FLASH_SendByte((SectorAddr & 0xFF00) >> 8);
    /* Send SectorAddr low nibble address byte */
    SPI_FLASH_SendByte(SectorAddr & 0xFF);
    /* Deselect the FLASH: Chip Select high */
    SPI_FLASH_CS_HIGH();

    /* Wait the end of Flash writing */
    SPI_FLASH_WaitForWriteEnd();
}

void SPI_FLASH_PageErase(uint32_t SectorAddr)
{
    /* Send write enable instruction */
    SPI_FLASH_WriteEnable();

    /* Sector Erase */
    /* Select the FLASH: Chip Select low */
    SPI_FLASH_CS_LOW();
    /* Send Sector Erase instruction */
    SPI_FLASH_SendByte(PE);
    /* Send SectorAddr high nibble address byte */
    SPI_FLASH_SendByte((SectorAddr & 0xFF0000) >> 16);
    /* Send SectorAddr medium nibble address byte */
    SPI_FLASH_SendByte((SectorAddr & 0xFF00) >> 8);
    /* Send SectorAddr low nibble address byte */
    SPI_FLASH_SendByte(SectorAddr & 0xFF);
    /* Deselect the FLASH: Chip Select high */
    SPI_FLASH_CS_HIGH();

    /* Wait the end of Flash writing */
    SPI_FLASH_WaitForWriteEnd();
}


/**
  * @brief  Writes more than one byte to the FLASH with a single WRITE
  *         cycle(Page WRITE sequence). The number of byte can't exceed
  *         the FLASH page size.
  * @param pBuffer : pointer to the buffer  containing the data to be
  *                  written to the FLASH.
  * @param WriteAddr : FLASH's internal address to write to.
  * @param NumByteToWrite : number of bytes to write to the FLASH,
  *                       must be equal or less than "SPI_FLASH_PageSize"
value.
  * @retval : None
  */
void SPI_FLASH_PageWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    /* Enable the write access to the FLASH */
    SPI_FLASH_WriteEnable();

    /* Select the FLASH: Chip Select low */
    SPI_FLASH_CS_LOW();
    /* Send "Write to Memory " instruction */
    SPI_FLASH_SendByte(WRITE);
    /* Send WriteAddr high nibble address byte to write to */
    SPI_FLASH_SendByte((WriteAddr & 0xFF0000) >> 16);
    /* Send WriteAddr medium nibble address byte to write to */
    SPI_FLASH_SendByte((WriteAddr & 0xFF00) >> 8);
    /* Send WriteAddr low nibble address byte to write to */
    SPI_FLASH_SendByte(WriteAddr & 0xFF);

    /* while there is data to be written on the FLASH */
    while (NumByteToWrite--)
    {
        /* Send the current byte */
        SPI_FLASH_SendByte(*pBuffer);
        /* Point on the next byte to be written */
        pBuffer++;
    }

    /* Deselect the FLASH: Chip Select high */
    SPI_FLASH_CS_HIGH();

    /* Wait the end of Flash writing */
    SPI_FLASH_WaitForWriteEnd();
}

/**
  * @brief  Writes block of data to the FLASH. In this function, the
  *         number of WRITE cycles are reduced, using Page WRITE sequence.
  * @param pBuffer : pointer to the buffer  containing the data to be
  *                  written to the FLASH.
  * @param WriteAddr : FLASH's internal address to write to.
  * @param NumByteToWrite : number of bytes to write to the FLASH.
  * @retval : None
  */
void SPI_FLASH_BufferWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    uint8_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

    Addr = WriteAddr % SPI_FLASH_PageSize;
    count = SPI_FLASH_PageSize - Addr;
    NumOfPage =  NumByteToWrite / SPI_FLASH_PageSize;
    NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;

    if ((WriteAddr & 0x00000FFF) == 0)		/*4 K擦写*/
    {
        SPI_FLASH_SectorErase(WriteAddr);
    }
    
    if (Addr == 0) /* WriteAddr is SPI_FLASH_PageSize aligned  */
    {
        if (NumOfPage == 0) /* NumByteToWrite < SPI_FLASH_PageSize */
        {
            SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumByteToWrite);
        }
        else /* NumByteToWrite > SPI_FLASH_PageSize */
        {
            while (NumOfPage--)
            {
                SPI_FLASH_PageWrite(pBuffer, WriteAddr, SPI_FLASH_PageSize);
                WriteAddr +=  SPI_FLASH_PageSize;
                pBuffer += SPI_FLASH_PageSize;
            }

            SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumOfSingle);
        }
    }
    else /* WriteAddr is not SPI_FLASH_PageSize aligned  */
    {
        if (NumOfPage == 0) /* NumByteToWrite < SPI_FLASH_PageSize */
        {
            if (NumOfSingle > count) /* (NumByteToWrite + WriteAddr) > SPI_FLASH_PageSize */
            {
                temp = NumOfSingle - count;

                SPI_FLASH_PageWrite(pBuffer, WriteAddr, count);
                WriteAddr +=  count;
                pBuffer += count;

                SPI_FLASH_PageWrite(pBuffer, WriteAddr, temp);
            }
            else
            {
                SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumByteToWrite);
            }
        }
        else /* NumByteToWrite > SPI_FLASH_PageSize */
        {
            NumByteToWrite -= count;
            NumOfPage =  NumByteToWrite / SPI_FLASH_PageSize;
            NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;

            SPI_FLASH_PageWrite(pBuffer, WriteAddr, count);
            WriteAddr +=  count;
            pBuffer += count;

            while (NumOfPage--)
            {
                SPI_FLASH_PageWrite(pBuffer, WriteAddr, SPI_FLASH_PageSize);
                WriteAddr +=  SPI_FLASH_PageSize;
                pBuffer += SPI_FLASH_PageSize;
            }

            if (NumOfSingle != 0)
            {
                SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumOfSingle);
            }
        }
    }
}

/**
  * @brief  Reads a block of data from the FLASH.
  * @param pBuffer : pointer to the buffer that receives the data read
  *                  from the FLASH.
  * @param ReadAddr : FLASH's internal address to read from.
  * @param NumByteToRead : number of bytes to read from the FLASH.
  * @retval : None
  */
void SPI_FLASH_BufferRead(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
        /* Select the FLASH: Chip Select low */
        SPI_FLASH_CS_LOW();

        /* Send "Read from Memory " instruction */
        SPI_FLASH_SendByte(READ);

        /* Send ReadAddr high nibble address byte to read from */
        SPI_FLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
        /* Send ReadAddr medium nibble address byte to read from */
        SPI_FLASH_SendByte((ReadAddr& 0xFF00) >> 8);
        /* Send ReadAddr low nibble address byte to read from */
        SPI_FLASH_SendByte(ReadAddr & 0xFF);

        while (NumByteToRead--) /* while there is data to be read */
        {
            /* Read a byte from the FLASH */
            *pBuffer = SPI_FLASH_GetByte();
            /* Point to the next location where the byte read will be saved */
            pBuffer++;
        }

        /* Deselect the FLASH: Chip Select high */
        SPI_FLASH_CS_HIGH();
}

uint32_t drv_FlashChipReadDword(uint32_t dwOffset)
{
    uint8_t abyTmp[4];
    SPI_FLASH_BufferRead(abyTmp, dwOffset, 4);
    return (((uint32_t)abyTmp[3] << 24) | ((uint32_t)abyTmp[2] << 16) | ((uint32_t)abyTmp[1] << 8) | (uint32_t)abyTmp[0]);
}

void drv_FlashChipWriteDatas(uint8_t *pData)
{
    uint32_t dwOffset;

    dwOffset = *pData++;
    dwOffset <<= 8;
    dwOffset += *pData++;
    dwOffset <<= 8;
    dwOffset += *pData++;
    dwOffset <<= 8;
    dwOffset += *pData++;

#if 0
    if ((dwOffset & 0x0000FFFF) == 0)
    {
        SPI_FLASH_SectorErase(dwOffset);
    }
#else
    if ((dwOffset & 0x00000FFF) == 0)		/*4 K擦写*/
    {
        SPI_FLASH_SectorErase(dwOffset);
    }
#endif

    SPI_FLASH_BufferWrite(pData, dwOffset, 32);

}

void drv_FlashChipRead(uint8_t * pdata)
{
    uint32_t dwOffset;

    dwOffset = *pdata++;
    dwOffset <<= 8;
    dwOffset += *pdata++;
    dwOffset <<= 8;
    dwOffset += *pdata++;
    dwOffset <<= 8;
    dwOffset += *pdata++;

    SPI_FLASH_BufferRead(pdata, dwOffset, 32);
}

/**
  * @brief  Reads FLASH identification.
  * @param  None
  * @retval : FLASH identification
  */
uint32_t SPI_FLASH_ReadID(void)
{
    uint32_t Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;

    /* Select the FLASH: Chip Select low */
    SPI_FLASH_CS_LOW();

    /* Send "RDID " instruction */
    SPI_FLASH_SendByte(0x9F);

    /* Read a byte from the FLASH */
    Temp0 = SPI_FLASH_GetByte();

    /* Read a byte from the FLASH */
    Temp1 = SPI_FLASH_GetByte();

    /* Read a byte from the FLASH */
    Temp2 = SPI_FLASH_GetByte();

    /* Deselect the FLASH: Chip Select high */
    SPI_FLASH_CS_HIGH();

    Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;

    return Temp;
}

/**
  * @brief  Initiates a read data byte (READ) sequence from the Flash.
  *   This is done by driving the /CS line low to select the device,
  *   then the READ instruction is transmitted followed by 3 bytes
  *   address. This function exit and keep the /CS line low, so the
  *   Flash still being selected. With this technique the whole
  *   content of the Flash is read with a single READ instruction.
  * @param ReadAddr : FLASH's internal address to read from.
  * @retval : None
  */
void SPI_FLASH_StartReadSequence(uint32_t ReadAddr)
{
    /* Select the FLASH: Chip Select low */
    SPI_FLASH_CS_LOW();

    /* Send "Read from Memory " instruction */
    SPI_FLASH_SendByte(READ);

    /* Send the 24-bit address of the address to read from -------------------- ---*/
    /* Send ReadAddr high nibble address byte */
    SPI_FLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
    /* Send ReadAddr medium nibble address byte */
    SPI_FLASH_SendByte((ReadAddr& 0xFF00) >> 8);
    /* Send ReadAddr low nibble address byte */
    SPI_FLASH_SendByte(ReadAddr & 0xFF);
}


/**
  * @brief  Enables the write access to the FLASH.
  * @param  None
  * @retval : None
  */
void SPI_FLASH_WriteEnable()
{
    /* Select the FLASH: Chip Select low */
    SPI_FLASH_CS_LOW();

    /* Send "Write Enable" instruction */
    SPI_FLASH_SendByte(WREN);

    /* Deselect the FLASH: Chip Select high */
    SPI_FLASH_CS_HIGH();

    GPIO_SetBits(GPIOC, GPIO_Pin_13);     //flash写使能	/* 使用硬件保护模式，写的时候才使能 modify by FJD 20120905*/
}

/**
  * @brief  Polls the status of the Write In Progress (WIP) flag in the
  *   FLASH's status  register  and  loop  until write  opertaion
  *   has completed.
  * @param  None
  * @retval : None
  */
void SPI_FLASH_WaitForWriteEnd(void)
{
    uint16_t i = 0;
    uint8_t FLASH_Status = 0;

    /* Select the FLASH: Chip Select low */
    SPI_FLASH_CS_LOW();

    /* Send "Read Status Register" instruction */
    SPI_FLASH_SendByte(RDSR);

    /* Loop as long as the memory is busy with a write cycle */
    do
    {
        /* Send a dummy byte to generate the clock needed by the FLASH
        and put the value of the status register in FLASH_Status variable */
        FLASH_Status = SPI_FLASH_GetByte();

        i++;
        if (i >= 60000) break;
    }
    while ((FLASH_Status & WIP_Flag) == SET); /* Write in progress */

    /* Deselect the FLASH: Chip Select high */
    SPI_FLASH_CS_HIGH();
    SPI_FLASH_WriteDisable();
}

void SPI_FLASH_WriteStatus(uint16_t wStatus)
{
    SPI_FLASH_WriteEnable();
    SPI_FLASH_CS_LOW();

    //SPI_FLASH_WriteEnable();
    //Spi_Soft_Delay(5);
    SPI_FLASH_SendByte(WRSR);
    SPI_FLASH_SendByte(wStatus & 0xff);
    SPI_FLASH_SendByte(wStatus>>8);

    SPI_FLASH_CS_HIGH();

    SPI_FLASH_WaitForWriteEnd();
}

uint8_t SPI_FLASH_ReadStatus(uint8_t Status)
{
    uint8_t FLASH_Status;
    SPI_FLASH_WriteEnable();
    SPI_FLASH_CS_LOW();
    SPI_FLASH_SendByte(Status);
    FLASH_Status = SPI_FLASH_GetByte();
    SPI_FLASH_CS_HIGH();
    //SPI_FLASH_WriteDisable();
    return FLASH_Status;
}
void SPI_FLASH_WriteDisable(void)
{
    SPI_FLASH_CS_LOW();
    SPI_FLASH_SendByte(WRDIS);
    SPI_FLASH_CS_HIGH();
    GPIO_ResetBits(GPIOC, GPIO_Pin_13);
}

void SPI_FLASH_ProtectSW(uint8_t sw)
{
    uint8_t State1;
    if(sw)
    {
        State1 =  SPI_FLASH_ReadStatus(RDSR);
        if((State1 &0x9c)!=0x9c)
        {
            SPI_FLASH_WriteStatus(0x009c);	/*写保护 modify by FJD 20120905*/
        }
    }
    else
    {
        State1 =  SPI_FLASH_ReadStatus(RDSR);
        if((State1 &0x9c)==0x9c)
        {
            SPI_FLASH_WriteStatus(0x0080);	/* 写使能modify by FJD 20120905*/
        }
    }
}


static struct rt_device SpiFlash_device;

/* RT-Thread Device Driver Interface */
static rt_err_t rt_SpiFlash_init(rt_device_t dev)
{

    return RT_EOK;
}

static rt_err_t rt_SpiFlash_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

static rt_err_t rt_SpiFlash_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_size_t rt_SpiFlash_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
    SPI_FLASH_BufferRead((uint8_t *)buffer, pos, size);
    return size;
}

static rt_size_t rt_SpiFlash_write (rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
    //SPI_FLASH_ProtectSW (0);
    //rt_thread_delay(5);
    SPI_FLASH_BufferWrite((uint8_t *)buffer, pos, size);
    //SPI_FLASH_ProtectSW (1);
    return size;
}

static rt_err_t rt_SpiFlash_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    switch (cmd)
    {
    case 0:
        SPI_FLASH_ProtectSW (0);
        break;
    case 1:
        SPI_FLASH_ProtectSW (1);
        break;
    }
    return RT_EOK;
}

void rt_hw_SpiFlash_init()
{
     SPI_FLASH_Init();
     
        /* register sdcard device */
        SpiFlash_device.type  = RT_Device_Class_Block;
        SpiFlash_device.init 	= rt_SpiFlash_init;
        SpiFlash_device.open 	= rt_SpiFlash_open;
        SpiFlash_device.close = rt_SpiFlash_close;
        SpiFlash_device.read 	= rt_SpiFlash_read;
        SpiFlash_device.write = rt_SpiFlash_write;
        SpiFlash_device.control = rt_SpiFlash_control;

        /* no private */
        SpiFlash_device.user_data = RT_NULL;

        rt_device_register(&SpiFlash_device, "flash",
                           RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_REMOVABLE | RT_DEVICE_FLAG_STANDALONE);
}

