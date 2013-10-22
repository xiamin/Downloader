
#include "I2C_Eeprom.h"

#define LOW		0
#define HIGH	1

#define I2C_READ    (0x1)                       /*读命令*/
#define I2C_WRITE   (0x0)                       /*写命令*/

#define I2C_W0_ADDR(a)   ( (a) & 0xFF )         /*转换字地址*/
#define I2C_W1_ADDR(a)   ( (a>>8) & 0xFF )      /*转换字地址*/

#define CLOCK_LOW_TIME				3
#define CLOCK_HIGH_TIME				3
#define START_CONDITION_HOLD_TIME	3
#define STOP_CONDITION_HOLD_TIME	3

#define GPIO_I2CSOFT1_SCL        GPIOB
#define GPIO_I2CSOFT1_SCL_pin    GPIO_Pin_6
#define GPIO_I2CSOFT1_SDA        GPIOB
#define GPIO_I2CSOFT1_SDA_pin    GPIO_Pin_7

#define GPIO_I2CSOFT2_SCL        GPIOB
#define GPIO_I2CSOFT2_SCL_pin    GPIO_Pin_4
#define GPIO_I2CSOFT2_SDA        GPIOB
#define GPIO_I2CSOFT2_SDA_pin    GPIO_Pin_14

/* I2C waite device ready try times */
#define I2C_DEV_WAITE_TRY_TIMES  100

static GPIO_TypeDef* GPIO_SDA1;
static GPIO_TypeDef* GPIO_SDA2;

static BYTE byCurI2cPort = 1;

static BYTE ack1 = 1, ack2 = 1;        /*对应应答位*/
static BYTE i2c_ack_stat;              /*状态记录*/

BYTE I2C_E2IcType;
WORD I2C_E2End;
BYTE I2C_PageSize;
BYTE I2C_SlAddMask;
BYTE I2C_SubAddr16B;
#ifdef VCTest
BYTE byE2InitFlag = 1;
#endif

VOID I2c_Soft_Delay(__IO BYTE byTime)
{
    while(byTime--);
}

VOID I2c_Soft_SCL(BYTE x)
{
    if (byCurI2cPort == 1)
    {
        if(x)
            GPIO_WriteBit(GPIO_I2CSOFT1_SCL, GPIO_I2CSOFT1_SCL_pin, Bit_SET);
        else
            GPIO_WriteBit(GPIO_I2CSOFT1_SCL, GPIO_I2CSOFT1_SCL_pin, Bit_RESET);
    }
    else if (byCurI2cPort == 2)
    {
        if(x)
            GPIO_WriteBit(GPIO_I2CSOFT2_SCL, GPIO_I2CSOFT2_SCL_pin, Bit_SET);
        else
            GPIO_WriteBit(GPIO_I2CSOFT2_SCL, GPIO_I2CSOFT2_SCL_pin, Bit_RESET);
    }
}

VOID I2c_Soft_SDA(BYTE x)
{
    if (byCurI2cPort == 1)
    {
        if(x)
            GPIO_WriteBit(GPIO_I2CSOFT1_SDA, GPIO_I2CSOFT1_SDA_pin, Bit_SET);
        else
            GPIO_WriteBit(GPIO_I2CSOFT1_SDA, GPIO_I2CSOFT1_SDA_pin, Bit_RESET);
    }
    else if (byCurI2cPort == 2)
    {
        if(x)
            GPIO_WriteBit(GPIO_I2CSOFT2_SDA, GPIO_I2CSOFT2_SDA_pin, Bit_SET);
        else
            GPIO_WriteBit(GPIO_I2CSOFT2_SDA, GPIO_I2CSOFT2_SDA_pin, Bit_RESET);
    }
}

VOID I2c_Soft_Dir_Read(VOID)
{
    /* 输入，上拉 */

    if (byCurI2cPort == 1)
    {
        GPIO_SDA1->CRL &= 0x0FFFFFFF;
        GPIO_SDA1->CRL |= 0x80000000;
        GPIO_SDA1->BSRR = 0x00000080;
    }
    else if (byCurI2cPort == 2)
    {
        GPIO_SDA2->CRH &= 0xF0FFFFFF;
        GPIO_SDA2->CRH |= 0x08000000;
        GPIO_SDA2->BSRR = 0x00004000;
    }
}

VOID I2c_Soft_Dir_Write(VOID)
{
    /* 输出，推 */
    if (byCurI2cPort == 1)
    {
        GPIO_SDA1->CRL &= 0x0FFFFFFF;
        GPIO_SDA1->CRL |= 0x30000000;
    }
    else if (byCurI2cPort == 2)
    {
        GPIO_SDA2->CRH &= 0xF0FFFFFF;
        GPIO_SDA2->CRH |= 0x03000000;
    }
}

BYTE I2c_Soft_ReadBitL(VOID)
{
    volatile BYTE x;

    if (byCurI2cPort == 1)
    {
        x = GPIO_ReadInputDataBit(GPIO_I2CSOFT1_SDA, GPIO_I2CSOFT1_SDA_pin);
    }
    else if (byCurI2cPort == 2)
    {
        x = GPIO_ReadInputDataBit(GPIO_I2CSOFT2_SDA, GPIO_I2CSOFT2_SDA_pin);
    }

    return x;
}

BYTE I2c_Soft_ReadBit(VOID)
{
    volatile BYTE x;

    I2c_Soft_SCL(LOW);
    I2c_Soft_Delay(CLOCK_LOW_TIME );
    I2c_Soft_SCL(HIGH);
    I2c_Soft_Delay(CLOCK_HIGH_TIME);

    x = I2c_Soft_ReadBitL();

    I2c_Soft_Delay(CLOCK_HIGH_TIME);
    I2c_Soft_SCL(LOW);
    I2c_Soft_Delay(CLOCK_LOW_TIME);
    return x;
}

VOID I2c_Soft_WriteBit(BYTE x)
{
    I2c_Soft_SCL(LOW);
    I2c_Soft_Delay(CLOCK_LOW_TIME/2);

    if (x & 0x80) /* the MSB is sent out first*/
        I2c_Soft_SDA(HIGH);
    else
        I2c_Soft_SDA(LOW);

    I2c_Soft_Delay(CLOCK_LOW_TIME);
    I2c_Soft_SCL(HIGH);
    I2c_Soft_Delay(CLOCK_HIGH_TIME);
    I2c_Soft_SCL(LOW);
    I2c_Soft_Delay(CLOCK_LOW_TIME);
}

VOID I2c_Soft_Start(VOID)
{
    I2c_Soft_Dir_Write();

    /*SDA=1, SCL=1 */
    I2c_Soft_SDA(HIGH);
    I2c_Soft_SCL(HIGH);
    I2c_Soft_Delay(CLOCK_HIGH_TIME);

    /*SCL=1, SDA=0*/
    I2c_Soft_SDA(LOW);
    I2c_Soft_Delay(START_CONDITION_HOLD_TIME);

    /*SCL=0 SDA=0*/
    I2c_Soft_SCL(LOW);
    I2c_Soft_Delay(CLOCK_LOW_TIME);
}

VOID I2c_Soft_Stop(VOID)
{
    I2c_Soft_Dir_Write();

    /*SCL=0, SDA=0 */
    I2c_Soft_SDA(LOW);
    I2c_Soft_Delay(CLOCK_LOW_TIME*2);

    /*SCL=1, SDA=0*/
    I2c_Soft_SCL(HIGH);
    I2c_Soft_Delay(CLOCK_HIGH_TIME*2);

    /*SCL=1 SDA=1*/
    I2c_Soft_SDA(HIGH);
    I2c_Soft_Delay(STOP_CONDITION_HOLD_TIME);
    I2c_Soft_Delay(CLOCK_LOW_TIME*3);
}

VOID I2c_Soft_WriteByte(BYTE byData)
{
    int i;

    I2c_Soft_Dir_Write();

    for(i=0; i<8; i++)
    {
        I2c_Soft_WriteBit(byData);
        byData<<=1;
    }

    I2c_Soft_Dir_Read();
}

BYTE I2c_Soft_ReadByte(VOID)
{
    BYTE i;
    BYTE byData = 0;

    I2c_Soft_Dir_Read();

    /*The MSB is read first*/
    for(i=0; i<7; i++)
    {
        byData |= I2c_Soft_ReadBit();
        byData <<= 1;
    }

    byData |= I2c_Soft_ReadBit();

    I2c_Soft_Dir_Write();

    return byData;
}

VOID I2c_Soft_Init(BYTE byI2cPort)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    if (byI2cPort == 1)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

        /* Configure I2C pins: SCL and SDA */
        GPIO_InitStructure.GPIO_Pin =  GPIO_I2CSOFT1_SCL_pin;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIO_I2CSOFT1_SCL, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Pin =  GPIO_I2CSOFT1_SDA_pin;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIO_I2CSOFT1_SDA, &GPIO_InitStructure);

        GPIO_SDA1 = GPIOB;

        byCurI2cPort = 1;
        I2c_Soft_SCL(HIGH);
        I2c_Soft_SDA(HIGH);
    }
    else if (byI2cPort == 2)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

        /* Configure I2C pins: SCL and SDA */
        GPIO_InitStructure.GPIO_Pin =  GPIO_I2CSOFT2_SCL_pin;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIO_I2CSOFT2_SCL, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Pin =  GPIO_I2CSOFT2_SDA_pin;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIO_I2CSOFT2_SDA, &GPIO_InitStructure);

        GPIO_SDA2 = GPIOB;

        byCurI2cPort = 2;
        I2c_Soft_SCL(HIGH);
        I2c_Soft_SDA(HIGH);
    }
}

BYTE I2c_Soft_Write(BYTE byI2cPort, BYTE bySlave, BYTE byAddr, BYTE *pbyData)
{
    volatile BYTE i2c_bw_ctrl; /* I2C write control */
    volatile BYTE i2c_addr_00; /* I2C address (0) */

    byCurI2cPort = byI2cPort;

    i2c_ack_stat=0x0;
    i2c_bw_ctrl = bySlave | I2C_WRITE;
    i2c_addr_00 = I2C_W0_ADDR(byAddr);

    if(1)
    {
        I2c_Soft_Start();
        I2c_Soft_WriteByte(i2c_bw_ctrl);
        ack1 = I2c_Soft_ReadBit();
        if(ack1 != 0)
            i2c_ack_stat |= 0x2;
        I2c_Soft_Delay(CLOCK_LOW_TIME*3);

        I2c_Soft_WriteByte(i2c_addr_00);
        ack1 = I2c_Soft_ReadBit();
        if(ack1 != 0)
            i2c_ack_stat |= 0x2;

        I2c_Soft_Delay(CLOCK_LOW_TIME*3);

        I2c_Soft_WriteByte(*pbyData);
        ack2 = I2c_Soft_ReadBit();
        if(ack2 != 0)
            i2c_ack_stat |= 0x8;

        I2c_Soft_Delay(CLOCK_LOW_TIME*3);

        I2c_Soft_Stop();
    }

    if(0 != i2c_ack_stat)
    {
        return FALSE;
    }

    I2c_Soft_WaitE2StandBy(byI2cPort, bySlave);
    return TRUE;
}

BYTE I2c_Soft_Read(BYTE byI2cPort, BYTE bySlave, BYTE byAddr, BYTE *pbyData)
{
    BYTE byData = 0;
    volatile BYTE i2c_bw_ctrl; /* I2C write control */
    volatile BYTE i2c_br_ctrl; /* I2C read control */
    volatile BYTE i2c_addr_00; /* I2C address (0) */

    byCurI2cPort = byI2cPort;

    i2c_ack_stat=0x0;
    i2c_bw_ctrl = bySlave | I2C_WRITE;
    i2c_br_ctrl = bySlave | I2C_READ;
    i2c_addr_00 = I2C_W0_ADDR(byAddr);

    if(1)
    {
        I2c_Soft_Start();
        I2c_Soft_WriteByte(i2c_bw_ctrl);
        ack1 = I2c_Soft_ReadBit(); /*read ack from slave*/
        if(ack1 != 0)
            i2c_ack_stat |= 0x1;
        I2c_Soft_Delay(CLOCK_LOW_TIME*3);

        I2c_Soft_WriteByte(i2c_addr_00);
        ack1 = I2c_Soft_ReadBit(); /*read ack from slave*/
        if(ack1!=0)
            i2c_ack_stat |= 0x2;
        I2c_Soft_Start();
        I2c_Soft_Delay(CLOCK_LOW_TIME*3);
        I2c_Soft_WriteByte(i2c_br_ctrl);
        ack1 = I2c_Soft_ReadBit(); /*read ack from slave*/
        if(ack1 != 0)
            i2c_ack_stat |= 0x3;
        I2c_Soft_Delay(CLOCK_LOW_TIME*3);
        byData = I2c_Soft_ReadByte();

        I2c_Soft_Stop();
    }

    *pbyData = byData;

    if(0 != i2c_ack_stat)
    {
        return FALSE;
    }

    I2c_Soft_WaitE2StandBy(byI2cPort, bySlave);
    return TRUE;
}

BYTE I2c_Soft_PageWrite(BYTE byI2cPort, BYTE bySlave, WORD WriteAddr, BYTE* pBuffer, BYTE NumByteToWrite)
{
    volatile BYTE i2c_bw_ctrl; /* I2C write control */
    volatile BYTE i2c_addr_00; /* I2C address (0) */
    volatile BYTE i2c_addr_01; /* I2C address (1) */

    byCurI2cPort = byI2cPort;

    i2c_ack_stat=0x0;
    i2c_bw_ctrl = bySlave | I2C_WRITE;
    if (I2C_SlAddMask)
    {
        i2c_bw_ctrl = (WriteAddr >> 8) & I2C_SlAddMask;
        i2c_bw_ctrl <<= 1;
        i2c_bw_ctrl |= bySlave;
        i2c_bw_ctrl |= I2C_WRITE;
    }
    i2c_addr_00 = I2C_W0_ADDR(WriteAddr);
    i2c_addr_01 = I2C_W1_ADDR(WriteAddr);

    if (1)
    {
        I2c_Soft_Start();

        I2c_Soft_WriteByte(i2c_bw_ctrl );
        ack1 = I2c_Soft_ReadBit(); /*read ack from slave*/
        if(ack1 != 0)
            i2c_ack_stat |= 0x1;
        I2c_Soft_Delay(CLOCK_LOW_TIME*3);
        if (I2C_SubAddr16B)
        {
            I2c_Soft_WriteByte(i2c_addr_01);
            ack1 = I2c_Soft_ReadBit(); /*read ack from slave*/
            if(ack1!=0)
                i2c_ack_stat |= 0x2;
            I2c_Soft_Delay(CLOCK_LOW_TIME*3);
        }

        I2c_Soft_WriteByte(i2c_addr_00);
        ack1 = I2c_Soft_ReadBit(); /*read ack from slave*/
        if(ack1!=0)
            i2c_ack_stat |= 0x4;
        I2c_Soft_Delay(CLOCK_LOW_TIME*3);

        /* While there is data to be written */
        while (NumByteToWrite--)
        {
            I2c_Soft_WriteByte(*pBuffer);
            ack2 = I2c_Soft_ReadBit();
            if(ack2 != 0)
                i2c_ack_stat |= 0x8;

            I2c_Soft_Delay(CLOCK_LOW_TIME*3);

            /* Point to the next byte to be written */
            pBuffer++;
        }

        I2c_Soft_Stop();
    }

    if(0 != i2c_ack_stat)
    {
        return FALSE;
    }

    I2c_Soft_WaitE2StandBy(byI2cPort, bySlave);
    return TRUE;
}

//BYTE I2c_Soft_BufferWrite(BYTE byI2cPort, BYTE bySlave, WORD WriteAddr, BYTE* pBuffer, BYTE NumByteToWrite)
BYTE I2c_Soft_BufferWrite(BYTE byI2cPort, BYTE bySlave, WORD WriteAddr, BYTE* pBuffer, WORD NumByteToWrite)
{
    BYTE byRes = TRUE;
    BYTE NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0;

    byCurI2cPort = byI2cPort;

    Addr = WriteAddr % I2C_PageSize;
    count = I2C_PageSize - Addr;
    NumOfPage =  NumByteToWrite / I2C_PageSize;
    NumOfSingle = NumByteToWrite % I2C_PageSize;

    if ((WriteAddr + NumByteToWrite) > I2C_E2End)
    {
        Addr = 0;
        return byRes;
    }

    /* If WriteAddr is I2C_PageSize aligned  */
    if(Addr == 0)
    {
        /* If NumByteToWrite < I2C_PageSize */
        if(NumOfPage == 0)
        {
            byRes |= I2c_Soft_PageWrite(byI2cPort, bySlave, WriteAddr, pBuffer, NumOfSingle);
        }
        /* If NumByteToWrite > I2C_PageSize */
        else
        {
            while(NumOfPage--)
            {
                byRes |= I2c_Soft_PageWrite(byI2cPort, bySlave, WriteAddr, pBuffer, I2C_PageSize);
                WriteAddr +=  I2C_PageSize;
                pBuffer += I2C_PageSize;
            }

            if(NumOfSingle!=0)
            {
                byRes |= I2c_Soft_PageWrite(byI2cPort, bySlave, WriteAddr, pBuffer, NumOfSingle);
            }
        }
    }
    /* If WriteAddr is not I2C_PageSize aligned  */
    else
    {
        if(NumByteToWrite > count)
        {
            NumByteToWrite -= count;
        }
        else
        {
            count = 0;
        }
        NumOfPage =  NumByteToWrite / I2C_PageSize;
        NumOfSingle = NumByteToWrite % I2C_PageSize;

        if(count != 0)
        {
            byRes |= I2c_Soft_PageWrite(byI2cPort, bySlave, WriteAddr, pBuffer, count);
            WriteAddr += count;
            pBuffer += count;
        }

        while(NumOfPage--)
        {
            byRes |= I2c_Soft_PageWrite(byI2cPort, bySlave, WriteAddr, pBuffer, I2C_PageSize);
            WriteAddr +=  I2C_PageSize;
            pBuffer += I2C_PageSize;
        }
        if(NumOfSingle != 0)
        {
            byRes |= I2c_Soft_PageWrite(byI2cPort, bySlave, WriteAddr, pBuffer, NumOfSingle);
        }
    }

    I2c_Soft_WaitE2StandBy(byI2cPort, bySlave);
    return byRes;
}

BYTE I2c_Soft_BufferRead(BYTE byI2cPort, BYTE bySlave, WORD WriteAddr, BYTE* pBuffer, WORD NumByteToRead)
{
    volatile BYTE i2c_bw_ctrl; /* I2C write control */
    volatile BYTE i2c_br_ctrl; /* I2C read control */
    volatile BYTE i2c_addr_00; /* I2C address (0) */
    volatile BYTE i2c_addr_01; /* I2C address (1) */

    byCurI2cPort = byI2cPort;

    i2c_ack_stat=0x0;
    i2c_bw_ctrl = bySlave | I2C_WRITE;
    i2c_br_ctrl = bySlave | I2C_READ;

    if (I2C_SlAddMask)
    {
        i2c_bw_ctrl = (WriteAddr >> 8) & I2C_SlAddMask;
        i2c_bw_ctrl <<= 1;
        i2c_bw_ctrl |= bySlave;
        i2c_bw_ctrl |= I2C_WRITE;

        i2c_br_ctrl = (WriteAddr >> 8) & I2C_SlAddMask;
        i2c_br_ctrl <<= 1;
        i2c_br_ctrl |= bySlave;
        i2c_br_ctrl = bySlave | I2C_READ;
    }
    i2c_addr_00 = I2C_W0_ADDR(WriteAddr);
    i2c_addr_01 = I2C_W1_ADDR(WriteAddr);

    if (1)
    {
        I2c_Soft_Start();

        I2c_Soft_WriteByte(i2c_bw_ctrl );
        ack1 = I2c_Soft_ReadBit(); /*read ack from slave*/
        if(ack1 != 0)
            i2c_ack_stat |= 0x1;
        I2c_Soft_Delay(CLOCK_LOW_TIME*3);

        if (I2C_SubAddr16B)
        {
            I2c_Soft_WriteByte(i2c_addr_01);
            ack1 = I2c_Soft_ReadBit(); /*read ack from slave*/
            if(ack1!=0)
                i2c_ack_stat |= 0x2;
            I2c_Soft_Delay(CLOCK_LOW_TIME*3);
        }

        I2c_Soft_WriteByte(i2c_addr_00);
        ack1 = I2c_Soft_ReadBit(); /*read ack from slave*/
        if(ack1!=0)
            i2c_ack_stat |= 0x4;
        I2c_Soft_Delay(CLOCK_LOW_TIME*3);

        /* Send STRAT condition a second time */
        I2c_Soft_Start();
        I2c_Soft_Delay(CLOCK_LOW_TIME*3);
        I2c_Soft_WriteByte(i2c_br_ctrl);
        ack1 = I2c_Soft_ReadBit(); /*read ack from slave*/
        if(ack1 != 0)
            i2c_ack_stat |= 0x3;

        /* While there is data to be read */
        while(NumByteToRead)
        {
            I2c_Soft_Delay(CLOCK_LOW_TIME*3);

            *pBuffer = I2c_Soft_ReadByte();

            /* Point to the next location where the byte read will be saved */
            pBuffer++;

            /* Decrement the read bytes counter */
            NumByteToRead--;

            if(NumByteToRead)
            {
                I2c_Soft_WriteBit(0x00);
            }
        }
    }

    I2c_Soft_Stop();

    if(0 != i2c_ack_stat)
    {
        return FALSE;
    }

    I2c_Soft_WaitE2StandBy(byI2cPort, bySlave);
    return TRUE;
}

VOID I2c_Soft_WaitE2StandBy(BYTE byI2cPort, BYTE bySlave)
{
    volatile BYTE i2c_bw_ctrl; /* I2C write control */
    volatile BYTE i2c_addr_00; /* I2C address (0) */
    volatile BYTE byTryTimes = 0; /* zl add for I2C try times. */

    byCurI2cPort = byI2cPort;

    i2c_ack_stat = 0x0;
    i2c_bw_ctrl = bySlave | I2C_WRITE;
    i2c_addr_00 = I2C_W0_ADDR(0);

    if (I2C_SlAddMask)
    {
        i2c_bw_ctrl = (0 >> 8) & I2C_SlAddMask;
        i2c_bw_ctrl <<= 1;
        i2c_bw_ctrl |= bySlave;
        i2c_bw_ctrl |= I2C_WRITE;
    }

    do
    {
        I2c_Soft_Start();
        I2c_Soft_WriteByte(i2c_bw_ctrl);
        ack1 = I2c_Soft_ReadBit(); /*read ack from slave*/
        I2c_Soft_Stop();

        I2c_Soft_Delay(CLOCK_LOW_TIME*3);
        byTryTimes++;
    }
    while((ack1 != 0) && (byTryTimes < I2C_DEV_WAITE_TRY_TIMES));  /* zl add for try times. */
}

BYTE I2c_Soft_E2IcIndentify(BYTE byI2cPort, BYTE bySlave)
{
    volatile BYTE i2c_bw_ctrl; /* I2C write control */
    volatile BYTE i2c_addr_00; /* I2C address (0) */
    volatile BYTE i2c_addr_01; /* I2C address (1) */

    WORD WriteAddr   = I2C_E2End;	/* 读写最后一个字节地址 */

    byCurI2cPort = byI2cPort;

    i2c_ack_stat=0x0;
    i2c_bw_ctrl = bySlave | I2C_WRITE;

    if (I2C_SlAddMask)
    {
        i2c_bw_ctrl = (WriteAddr >> 8) & I2C_SlAddMask;
        i2c_bw_ctrl <<= 1;
        i2c_bw_ctrl |= bySlave;
        i2c_bw_ctrl |= I2C_WRITE;
    }
    i2c_addr_00 = I2C_W0_ADDR(WriteAddr);
    i2c_addr_01 = I2C_W1_ADDR(WriteAddr);

    I2c_Soft_Delay(100);
    if (1)	/* Write */
    {
        I2c_Soft_Start();

        I2c_Soft_WriteByte(i2c_bw_ctrl );
        ack1 = I2c_Soft_ReadBit(); /*read ack from slave*/
        if(ack1 != 0)
            i2c_ack_stat |= 0x1;

        I2c_Soft_Delay(CLOCK_LOW_TIME*3);

        if(0 != i2c_ack_stat)
        {
            I2c_Soft_Stop();
            return FALSE;	/* ACK 不正确 */
        }

        if (I2C_SubAddr16B)
        {
            I2c_Soft_WriteByte(i2c_addr_01);
            ack1 = I2c_Soft_ReadBit(); /*read ack from slave*/
            if(ack1!=0)
                i2c_ack_stat |= 0x2;

            I2c_Soft_Delay(CLOCK_LOW_TIME*3);

            if(0 != i2c_ack_stat)
            {
                I2c_Soft_Stop();
                return FALSE;	/* ACK 不正确 */
            }
        }

        I2c_Soft_WriteByte(i2c_addr_00);
        ack1 = I2c_Soft_ReadBit(); /*read ack from slave*/
        if(ack1!=0)
            i2c_ack_stat |= 0x4;

        I2c_Soft_Delay(CLOCK_LOW_TIME*3);

        if(0 != i2c_ack_stat)
        {
            I2c_Soft_Stop();
            return FALSE;	/* ACK 不正确 */
        }

        I2c_Soft_Stop();
        I2c_Soft_WaitE2StandBy(byI2cPort, bySlave);
    }

    return TRUE;
}

VOID I2c_Soft_E2IcTypeSet(BYTE byE2IcTyp)
{
    I2C_E2IcType = byE2IcTyp;
    switch(byE2IcTyp)
    {
    case USE_24C01:
        I2C_E2End      = 0x007F;
        I2C_PageSize   = 16;
        I2C_SlAddMask  = 0;
        I2C_SubAddr16B = 0;
        break;
    case USE_24C02:
        I2C_E2End      = 0x00FF;
        I2C_PageSize   = 16;
        I2C_SlAddMask  = 0;
        I2C_SubAddr16B = 0;
        break;
    case USE_24C04:
        I2C_E2End      = 0x01FF;
        I2C_PageSize   = 16;
        I2C_SlAddMask  = 0x01;
        I2C_SubAddr16B = 0;
        break;
    case USE_24C08:
        I2C_E2End      = 0x03FF;
        I2C_PageSize   = 16;
        I2C_SlAddMask  = 0x03;
        I2C_SubAddr16B = 0;
        break;
    case USE_24C16:
        I2C_E2End      = 0x07FF;
        I2C_PageSize   = 16;
        I2C_SlAddMask  = 0x07;
        I2C_SubAddr16B = 0;
        break;
    case USE_24C32:
        I2C_E2End      = 0x0FFF;
        I2C_PageSize   = 32;
        I2C_SlAddMask  = 0;
        I2C_SubAddr16B = 1;
        break;
    case USE_24C64:
        I2C_E2End      = 0x1FFF;
        I2C_PageSize   = 32;
        I2C_SlAddMask  = 0;
        I2C_SubAddr16B = 1;
        break;
    case USE_24C128:
        I2C_E2End      = 0x3FFF;
        I2C_PageSize   = 64;
        I2C_SlAddMask  = 0;
        I2C_SubAddr16B = 1;
        break;
    case USE_24C256:
        I2C_E2End      = 0x7FFF;
        I2C_PageSize   = 64;
        I2C_SlAddMask  = 0;
        I2C_SubAddr16B = 1;
        break;
    case USE_24C512:
        I2C_E2End      = 0xFFFF;
        I2C_PageSize   = 64;
        I2C_SlAddMask  = 0;
        I2C_SubAddr16B = 1;
        break;
    default:
        break;
    }
}

CONST BYTE E2IcTypeTab16B[5] =
{
    USE_24C512,
    USE_24C256,
    USE_24C128,
    USE_24C64,
    USE_24C32
};

CONST BYTE E2IcTypeTab08B[5] =
{
    USE_24C16,
    USE_24C08,
    USE_24C04,
    USE_24C02,
    USE_24C01
};

/*
  E2 芯片型号检测函数
param:
	byI2cPort : 挂接的端口号
	bySlave   : Slave地址
retuen:
	识别出的E2型号名
*/
BYTE I2c_Soft_E2IcTypeDet(BYTE byI2cPort, BYTE bySlave)	/* cql cdd for E2 Type det */
{
    //BYTE i;
    BYTE byE2IcType = USE_24C16;	/* 默认24C16 */

    I2c_Soft_Init(byI2cPort);

    /* 只区分 24C16  24C256 默认24C16 */
    I2c_Soft_E2IcTypeSet(USE_24C16);
    if (I2c_Soft_E2IcIndentify(byI2cPort, bySlave) == TRUE)
    {
        byE2IcType = USE_24C16;
    }
    else
    {
        I2c_Soft_E2IcTypeSet(USE_24C256);
        if (I2c_Soft_E2IcIndentify(byI2cPort, bySlave) == TRUE)
        {
            byE2IcType = USE_24C256;
        }
    }

    I2c_Soft_E2IcTypeSet(byE2IcType);

    return byE2IcType;
}

BYTE ReadByte(WORD wAddrS)
{
    BYTE byValue;

    I2c_Soft_BufferRead(E2_PORTNUM, E2_SLAVEADDR, wAddrS, &byValue, 1);

    return byValue;
}

VOID WriteByte(WORD wAddrS, BYTE byValue)
{
    I2c_Soft_BufferWrite(E2_PORTNUM, E2_SLAVEADDR, wAddrS, (BYTE*)&byValue, 1);
}

/*
    The WORD read write should we care about of Ending Mode? WARNNING!!!
*/
DWORD ReadDword(WORD wAddrS)
{
    DWORD wValue;

    I2c_Soft_BufferRead(E2_PORTNUM, E2_SLAVEADDR, wAddrS, (BYTE*)&wValue, 4);

    return wValue;
}

VOID WriteDword(WORD wAddrS, DWORD wValue)
{
    I2c_Soft_BufferWrite(E2_PORTNUM, E2_SLAVEADDR, wAddrS, (BYTE*)&wValue, 4);
}

VOID WriteBlock(WORD wAddrS, BYTE *pbuf, WORD wbufSize)
{
    I2c_Soft_BufferWrite(E2_PORTNUM, E2_SLAVEADDR, wAddrS, pbuf, wbufSize);
}

VOID ReadBlock(WORD wAddrS, BYTE *pbuf, WORD wbufSize)
{
    I2c_Soft_BufferRead(E2_PORTNUM, E2_SLAVEADDR, wAddrS, pbuf, wbufSize);
}


