
#include <rtthread.h>
#include <stdlib.h>
#include "serial.h"
#include "lib_Misc.h"
#include "tad_LCD.h"
#include "tad_Downloader.h"
#include "I2C_Eeprom.h"
#include "tad_lcd.h"

#define   SOH    0x01
#define   ACK    0x06
#define   EOT    0x04
#define   CAN    0x18
#define   STX    0x02
#define   NAK    0x15
#define   PAK    0x88
#define   CCE    0x99

#pragma pack(1)
typedef struct _TaDownFile_
{
    uint32_t MagicNum;
    uint8_t  FileName [64];
    uint8_t FileType;
    uint32_t FileLen;
} TaDownFile;

enum _Tad_Status_
{
    Status_Idle,
    Status_Menu,
    Status_WaitingConnect,
    Status_Burning,
};

typedef enum _Tad_FileType_
{
    Type_Fw = 1,
    Type_Menu,
    Type_Osd,
} File_Type;

enum _Tad_KeyValue_
{
    TadKey_Start = 0x01,
    TadKey_Cancel = 0x02,
    TadKey_Open = 0x04,	//0x04,
    TadKey_Close = 0x08,
};

typedef struct _FLASH_ObjInfo_
{
    File_Type FileType;
    uint32_t FontFileOs;
    uint8_t FileName[64];
    uint32_t TargetOs;	/*需要被烧写到目标板的位置*/
} ObjInfo;

#pragma pack()
ObjInfo flashobj[10];	/*flash中可存放最多10个对象*/
ObjInfo SelectedObj;	/*当前被选中的对象*/
ObjInfo* theMenuObj = NULL;
ObjInfo* theHzkobj = NULL;
ObjInfo* thePinyinObj = NULL;
ObjInfo* theASCIIObj = NULL;

rt_device_t pCom232;
rt_device_t pCom485;
rt_device_t pSpiFlash;
rt_device_t pLcd1602;
enum _Tad_Status_ tad_status = Status_Idle;

uint32_t CurBps = 2;
uint8_t AutoUpdateEn = 0;
rt_timer_t ComTimer;

#ifdef _BEEP_THREAD_
rt_thread_t BeepThread;
rt_timer_t BeepTimeOut;
#endif
uint8_t FwDownLoadFlag = 0;

#define GPIO_PORT_BEEP	GPIOB
#define GPIO_PIN_BEEP	GPIO_Pin_12

//const char OsStr[8][4] = {{"00 "}, {"04 "}, {"08 "}, {"0C "}, {"10 "}, {"14 "}, {"18 "}, {"1C"}};
const char OsStr[10][4] = {{"01 "}, {"02 "}, {"03 "}, {"04 "}, {"05 "}, {"06 "}, {"07 "}, {"08"}, {"09"}, {"10"}};

const uint8_t taReboot[7] = {0x99, 0x00, 0x07, 0xAA, 0x00, 0x00, 0x4A};

const uint8_t taConnectPassW[5] = {'T', 'U', 'O', 'A', 'N'};

const char BpsStr[3][20] = {{"57600-N-1"}, {"115200-N-1"}, {"230400-N-1"}};

void TaBeepThread_entry();
uint8_t taD_AutoUpdateOsd();
void taD_Display(uint8_t percent);
File_Type taD_GetFileType(uint8_t objname[]);
uint8_t usart_GetData(rt_device_t Dev, uint8_t *Buffer, uint8_t Size, uint32_t TimeOut);

void taD_KeyPortInit()
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_GPIOB , ENABLE);

    /* Configure SPI1 pins: DO SCK  and DI */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_IPU;	//Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  GPIO_PIN_BEEP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//Out_PP;
    GPIO_Init(GPIO_PORT_BEEP, &GPIO_InitStructure);
}

uint8_t taD_GetKeyValue()
{
    uint16_t PortVal;
    PortVal = GPIO_ReadInputData(GPIOC);

    return (uint8_t)(((~PortVal) & 0x003c) >> 2);
}

uint8_t taD_CheckKey(uint8_t Key)
{
    if (taD_GetKeyValue() == Key)
    {
        rt_thread_delay(300);
        if (taD_GetKeyValue() == Key)
        {
            return 1;
        }
    }
    return 0;
}

#ifdef _BEEP_THREAD_
void BeepTimeOutProc(void *para)
{
    //if ((BeepThread->stat == RT_THREAD_READY) || (BeepThread->stat == RT_THREAD_RUNNING))
    {
        rt_thread_delete(BeepThread);
        rt_thread_delay(100);
    }
    GPIO_ResetBits(GPIO_PORT_BEEP, GPIO_PIN_BEEP);
}
#endif

uint8_t taCmdMatchIndex;
uint8_t taCmdMatchFlag;
uint8_t taFileBuffer[138];
uint8_t taCmdLen = 0;

const uint8_t taCmd[] = {0x99, 0x00, 0x07, 0xAD};

uint8_t taD_FontDownloaderSw = 0;		//ê?·?′ò?a×??ˉ×??a????
void ComTimeOutHandle(void *paramter)
{
    taCmdMatchIndex = 0;
    pCom485->control(pCom485, RT_DEVICE_CTRL_SUSPEND, (void *)RT_NULL);
    pCom485->control(pCom485, RT_DEVICE_CTRL_RESUME, (void *)RT_NULL);
    //pCom232->control(pCom232, RT_SERIAL_CMD_ClearRxBuff, (void *)RT_NULL);
    rt_timer_stop(ComTimer);
}
uint8_t lib_ChecksumAdd(uint8_t *pArray, uint8_t byNum)
{
    uint8_t    i;
    uint8_t    sum = 0;

    for (i = 0; i < byNum; i++)
    {
        sum += *(pArray + i);
    }
    return sum;
}
/*void taD_SelfUpdate(uint8_t* data)
{
	uint8_t chksum = 0;
        for()
}*/
void ta_CmdPaser(uint8_t Data)
{
    if (taCmdMatchIndex < 3)
    {
        if (taCmdMatchIndex < 2)
        {
            if (Data == taCmd[taCmdMatchIndex])
            {
                taFileBuffer[taCmdMatchIndex] = Data;
                taCmdMatchIndex++;
                return;
            }
            else
            {
                taCmdMatchIndex = 0;
                return;
            }
        }
        else
        {
            taFileBuffer[taCmdMatchIndex] = Data;
            taCmdMatchIndex++;
            taCmdLen = taFileBuffer[2];
        }
    }
    else
    {
        taFileBuffer[taCmdMatchIndex] = Data;
        taCmdMatchIndex++;
        if (taCmdMatchIndex >= taCmdLen)
        {
            uint8_t abyTmp[3];
            uint32_t offset;
            if (lib_ChecksumAdd(taFileBuffer, taCmdLen - 1) == taFileBuffer[taCmdLen - 1])
            {
                switch (taFileBuffer[3])
                {
                case 0xAD:
                    rt_thread_delay(80);
                    abyTmp[0] = 0x06;
                    pCom485->write(pCom485, 0, abyTmp, 1);
                    break;
                case 0xAB:
                    //if (taCmdLen == 41)
                {
                    //offset = *((uint32_t *)&taFileBuffer[4]);
                    offset = (((uint32_t)taFileBuffer[4])<<24) | (((uint32_t)taFileBuffer[5])<<16) | (((uint32_t)taFileBuffer[6])<<8) | ((uint32_t)taFileBuffer[7]);
                    pSpiFlash->write(pSpiFlash, offset, &taFileBuffer[8], 32);
                    //rt_thread_delay(10);
                    abyTmp[0] = 0x06;
                    pCom485->write(pCom485, 0, abyTmp, 1);
                }
                break;
                }
            }
            ComTimeOutHandle((void *)0);
            //taCmdMatchIndex = 0;
        }
    }
}

void taD_IdleHandle()
{
    rt_size_t RsBytes = 0;
    uint8_t RcvData = 0;
    //uint8_t RcvData[7] = {0};
    uint8_t Key = taD_GetKeyValue();
    if (Key == TadKey_Start)
    {
        rt_thread_delay(150);
        if (taD_GetKeyValue() == TadKey_Start)
        {
            tad_status = Status_WaitingConnect;
            ComTimeOutHandle((void *)0);
            while (taD_GetKeyValue() & TadKey_Start);
            //rt_thread_delay(1000);
            return;
        }
    }
    else if (Key == TadKey_Open)
    {
        rt_thread_delay(1000);
        if (taD_GetKeyValue() == TadKey_Open)
        {
            tad_status = Status_Menu;
            while (taD_GetKeyValue() & TadKey_Open);
            return;
        }
    }
    RsBytes = pCom485->read(pCom485, 0, &RcvData, 1);		//用485烧写固件
    if (RsBytes)
    {
#if 0
        uint8_t buffer[41] = {0};
        uint32_t offset = 0;
        uint8_t ack = 0x06;

        if(lib_ChecksumAdd(RcvData, 6) != RcvData[6])
        {
            return ;
        }

        switch(RcvData[5])
        {
        case 0x1B:
            pCom485->control(pCom485, RT_SERIAL_CMD_SETBPS, (void *)BpsStr[0]);
            break;
        case 0x1c:
            pCom485->control(pCom485, RT_SERIAL_CMD_SETBPS, (void *)BpsStr[1]);
            break;
        case 0xff:
            pCom485->control(pCom485, RT_SERIAL_CMD_SETBPS, (void *)BpsStr[2]);
            break;
        default:
            break;
        }
        pCom485->write(pCom485, 0, &ack, 1);
        rt_thread_delay(1000);

        while(usart_GetData(pCom485, buffer, 41, 100))
        {
            
            offset = (((uint32_t)taFileBuffer[4])<<24) | (((uint32_t)taFileBuffer[5])<<16) | (((uint32_t)taFileBuffer[6])<<8) | ((uint32_t)taFileBuffer[7]);
            pSpiFlash->write(pSpiFlash, offset, &buffer[8], 32);
            pCom485->write(pCom485, 0, &ack, 1);
        }
        pCom485->control(pCom485, RT_SERIAL_CMD_SETBPS, (void *)BpsStr[CurBps]);
#endif
        ta_CmdPaser(RcvData);
        rt_timer_start(ComTimer);
    }
    else
    {
        rt_thread_delay(20);
    }
}


const uint8_t FirstLevelStr[][16] =
{
    {"->Select File   "},
    {"->Setting Bps   "},
    {"->AutoUpdate Y/N"}
};

const uint8_t BpsStrSetting[][16] =
{
    {"->57600Bps   "},
    {"->115200Bps   "},
    {"->230400Bps   "},
};

const uint8_t AutoUpdateSetting[][16] =
{
    {"->No"},
    {"->Yes"}
};

struct _MenuStruct_
{
    uint8_t MenuId;
    uint8_t Menu;
};

struct _FlashFile_
{
    uint8_t Flag;
};

//uint8_t FileName[10][64];
//uint32_t FileOs[10];

uint8_t lib_TraverseItem(uint8_t byMaxItemNo, uint8_t byCurItemNo, uint8_t byFlag) //0~MaxItemNo-1; Flag Ture: 加  False: 减
{
    if (byFlag == 1)
    {
        if (byCurItemNo < (byMaxItemNo - 1))
        {
            byCurItemNo++;
        }
    }
    else
    {
        if (0 < byCurItemNo)
        {
            byCurItemNo--;
        }
    }
    return byCurItemNo;
}

uint8_t ScanFlash()
{
    uint8_t FileNum = 0;
    uint8_t i;
    uint8_t FileHead[sizeof(TaDownFile)];
    struct _TaDownFile_ *pFileHead;
#if 1
    for (i = 0; i < 7; i++)
    {
        memset(FileHead, 0, sizeof(TaDownFile));
        pSpiFlash->read(pSpiFlash, i*0x40000, FileHead, sizeof(TaDownFile));
        pFileHead = (TaDownFile *)FileHead;
        if (pFileHead->MagicNum == 0x55AA)
        {
            memcpy(flashobj[FileNum].FileName, (const void *)&(pFileHead->FileName), 64);
            flashobj[FileNum].FontFileOs=  i*0x40000;
            flashobj[FileNum].FileType = taD_GetFileType(flashobj[FileNum].FileName);
            if(flashobj[FileNum].FileType == Type_Menu)
            {
                theMenuObj = &flashobj[FileNum];	/*选中菜单这个对象*/
                flashobj[FileNum].TargetOs = 0x80000;
            }
            FileNum++;
        }
    }

    /*将固定的三个文件加到对象数组中*/
    strcpy(flashobj[FileNum].FileName, "hzk14.bin");/*汉字库*/
    flashobj[FileNum].FontFileOs = 0x1c0000;
    flashobj[FileNum].FileType = Type_Osd;
    flashobj[FileNum].TargetOs = 0x20000;
    theHzkobj = &flashobj[FileNum];

    strcpy(flashobj[FileNum + 1].FileName, "ASC16_spec.bin");	/*ASCII*/
    flashobj[FileNum + 1].FontFileOs = 0x1f9000;
    flashobj[FileNum + 1].FileType = Type_Osd;
    flashobj[FileNum + 1].TargetOs = 0x10000;
    theASCIIObj = &flashobj[FileNum + 1];

    strcpy(flashobj[FileNum + 2].FileName, "pinyin_tol.bin");	/*拼音*/
    flashobj[FileNum + 2].FontFileOs = 0x1fb000;
    flashobj[FileNum + 2].FileType = Type_Osd;
    flashobj[FileNum + 2].TargetOs = 0x0;
    thePinyinObj = &flashobj[FileNum + 2];


#else
    {
        const uint8_t CFileName [][64] =
        {
            {"TAD-AH_01.09.20.bin"},
            {"TAD-RH.IR10_01.09.20.bin"},
            {"TAD-RJH.IR10_01.09.20.bin"},
            {"TAD-RJHL.IR10_01.09.20.bin"}
        };
        for (i = 0; i < 4; i++)
        {
            memcpy(FileName[FileNum], CFileName[i], 64);
            FileOs[FileNum] =  i*0x40000;
            FileNum++;
        }
    }
#endif
    return (FileNum + 3);
}

uint8_t WaitKey()
{
    uint8_t CurKey = 0;
    while (1)
    {
        CurKey = taD_GetKeyValue();
        if (CurKey)
        {
            rt_thread_delay(80);
            if (CurKey == taD_GetKeyValue())
            {
                return CurKey;
            }
        }
    }
}

void taD_MenuHandle()
{
    uint8_t MenuLevel = 0;
    uint8_t LevelOneIndex = 0;
    uint8_t LevelTwoIndex = 0;
    uint8_t LevelTwoNum = 3;
    uint8_t MenuId = 0;
    uint8_t Key = 0;
    uint8_t tmpOs;

    pLcd1602->write(pLcd1602, -1, FirstLevelStr[LevelOneIndex], 16);
    while (1)
    {
        Key = WaitKey();
        if (Key == TadKey_Open)
        {
            if (0 == MenuId)
            {
                if (0 == LevelOneIndex)
                {
                    LevelTwoNum = ScanFlash();

                    //tmpOs = FileOs[LevelTwoIndex]/0x40000;
                    tmpOs = LevelTwoIndex;

                    pLcd1602->write(pLcd1602, -1, "->", 32);
                    pLcd1602->write(pLcd1602, 2, OsStr[tmpOs], 3);
                    pLcd1602->write(pLcd1602, 5, flashobj[LevelTwoIndex].FileName, 32);
                    MenuId = 1;
                }
                else if(1 == LevelOneIndex)
                {
                    LevelTwoIndex = CurBps;
                    pLcd1602->write(pLcd1602, -1, BpsStrSetting[LevelTwoIndex], 16);
                    MenuId = 2;
                }
                else
                {
                    LevelTwoIndex = AutoUpdateEn;
                    pLcd1602->write(pLcd1602, -1, AutoUpdateSetting[LevelTwoIndex], 16);
                    MenuId = 3;
                }
            }
            else if(1 == MenuId)
            {
                memcpy(&SelectedObj, &flashobj[LevelTwoIndex], sizeof(ObjInfo));
                //SelectedObj.FontFileOs = flashobj[LevelTwoIndex].FontFileOs;
                WriteDword(FILE_SEL_OS, SelectedObj.FontFileOs);		//记住选择
                //memcpy(SelectedObj.FileName, flashobj[LevelTwoIndex].FileName, 64);
                tad_status = Status_Idle;
                break;
            }
            else if(2 == MenuId)
            {
                CurBps = LevelTwoIndex;
                WriteByte(BPS_SETTING_OS, CurBps);		//记住选择
                pCom485->control(pCom485, RT_SERIAL_CMD_SETBPS, (void *)&BpsStr[CurBps]);
                tad_status = Status_Idle;
                break;
            }
            else if(3 == MenuId)
            {
                AutoUpdateEn = LevelTwoIndex;
                WriteByte(AutoUpdate_SET_OS, AutoUpdateEn);
                tad_status = Status_Idle;
                break;
            }
        }
        else if (Key == TadKey_Close)
        {
            tad_status = Status_Idle;
            break;
        }
        else if (Key == TadKey_Start)
        {
            if (0 == MenuId)
            {
                LevelOneIndex = lib_TraverseItem(3, LevelOneIndex, 1);
                pLcd1602->write(pLcd1602, -1, FirstLevelStr[LevelOneIndex], 16);
            }
            else if(1 == MenuId)
            {
                LevelTwoIndex = lib_TraverseItem(LevelTwoNum, LevelTwoIndex, 1);
                //tmpOs = FileOs[LevelTwoIndex]/0x40000;
                tmpOs = LevelTwoIndex;
                pLcd1602->write(pLcd1602, -1, "->", 32);
                pLcd1602->write(pLcd1602, 2, OsStr[tmpOs], 3);
                pLcd1602->write(pLcd1602, 5, flashobj[LevelTwoIndex].FileName, 32);
            }
            else if(2 == MenuId)
            {
                LevelTwoIndex = lib_TraverseItem(LevelTwoNum, LevelTwoIndex, 1);
                pLcd1602->write(pLcd1602, -1, BpsStrSetting[LevelTwoIndex], 16);
            }
            else if(3 == MenuId)
            {
                LevelTwoIndex = lib_TraverseItem(2, LevelTwoIndex, 1);
                pLcd1602->write(pLcd1602, -1, AutoUpdateSetting[LevelTwoIndex], 16);
            }
        }
        else if (Key == TadKey_Cancel)
        {
            if (0 == MenuId)
            {
                LevelOneIndex = lib_TraverseItem(3, LevelOneIndex, 0);
                pLcd1602->write(pLcd1602, -1, FirstLevelStr[LevelOneIndex], 16);
            }
            else if(1 == MenuId)
            {
                LevelTwoIndex = lib_TraverseItem(LevelTwoNum, LevelTwoIndex, 0);
                //tmpOs = FileOs[LevelTwoIndex]/0x40000;
                tmpOs = LevelTwoIndex;
                pLcd1602->write(pLcd1602, -1, "->", 32);
                pLcd1602->write(pLcd1602, 2, OsStr[tmpOs], 3);
                pLcd1602->write(pLcd1602, 5, flashobj[LevelTwoIndex].FileName, 32);
            }
            else if(2 == MenuId)
            {
                LevelTwoIndex = lib_TraverseItem(LevelTwoNum, LevelTwoIndex, 0);
                pLcd1602->write(pLcd1602, -1, BpsStrSetting[LevelTwoIndex], 16);
            }
            else if(3 == MenuId)
            {
                LevelTwoIndex = lib_TraverseItem(2, LevelTwoIndex, 0);
                pLcd1602->write(pLcd1602, -1, AutoUpdateSetting[LevelTwoIndex], 16);
            }
        }
        Key = 0;
        rt_thread_delay(150);
    }
    pLcd1602->write(pLcd1602, -1, SelectedObj.FileName, 32);
    rt_thread_delay(1000);
}

void ta_CmdPacker(char *pDes, char *pSrcData, uint16_t Size)
{
}


uint8_t usart_GetData(rt_device_t Dev, uint8_t *Buffer, uint8_t Size, uint32_t TimeOut)
{
    uint32_t  i;
    uint8_t RxByte = Size;
    uint8_t *pBuf = Buffer;

    for (i = 0; i < TimeOut; i++)
    {
        if (pCom485->read(pCom485, 0, pBuf, 1))
        {
            Size--;
            if (!Size)
            {
                return 1;
            }
            pBuf++;
        }
        else
        {
            rt_thread_delay(1);
        }
    }
    return 0;
}

uint8_t taD_ConnectBootLoader(uint32_t preBps, uint32_t DwnBps, uint32_t RetryTimes)
{
    uint32_t i, j;
    uint8_t tmpRxBuffer[5];
    uint8_t *pBuffer;

    pLcd1602->write(pLcd1602, -1, SelectedObj.FileName, 32);
    pLcd1602->control(pLcd1602, DispConnecting, NULL);
    rt_sem_release(&lcd_writesem);
    rt_thread_delay(200);

    //pLcd1602->control(pLcd1602, DispConnecting, (void *)0);
    for (i = 0; i < RetryTimes; i++)
    {
        pCom485->control(pCom485, RT_SERIAL_CMD_SETBPS, "2400-N-1");
        pCom485->write(pCom485, 0, taReboot, 7);
        rt_thread_delay(80);

#if 1
        pCom485->control(pCom485, RT_SERIAL_CMD_SETBPS, (void *)BpsStr[CurBps]);
        for (j = 0; j < 40; j++)
        {
            uint8_t SynData[2] = {0x55, 0xAA};
            pCom485->write(pCom485, 0, SynData, 1);
            Delay_1us(50);
            pCom485->write(pCom485, 0, (SynData + 1), 1);
            rt_thread_delay(1);
            if (taD_CheckKey(TadKey_Cancel))
            {
                pLcd1602->control(pLcd1602, DispEnd, (void *)0);
                pLcd1602->write(pLcd1602, -1, SelectedObj.FileName, 32);
                tad_status = Status_Idle;
                return 0;
            }
        }

        pCom485->control(pCom485, RT_SERIAL_CMD_ClearRxBuff, (void *)0);
        for (j = 0; j < 5; j++)
        {
            pCom485->write(pCom485, 0, taConnectPassW, 5);

            if (usart_GetData(pCom485, tmpRxBuffer, 1, 150))
            {
                if (0x43 == tmpRxBuffer[0])
                {
                    uint8_t percent = 0;
                    //pLcd1602->control(pLcd1602, DispProcessing, (void *)&percent);
                    return 1;
                }
                pCom485->control(pCom485, RT_SERIAL_CMD_ClearRxBuff, (void *)RT_NULL);
            }
            if (taD_CheckKey(TadKey_Cancel))
            {
                pLcd1602->control(pLcd1602, DispEnd, (void *)0);
                pLcd1602->write(pLcd1602, -1, SelectedObj.FileName, 32);
                tad_status = Status_Idle;
                return 0;
            }
        }
#endif
        rt_thread_delay(100);
    }
    return 0;
}

uint16_t xmodem_CheckSumAdd(uint8_t *pData, uint8_t Size)
{
    uint16_t i, cs = 0;
    if (-1 == Size)
        return 0;
    for (i = 0; i < Size; i++)
    {
        cs += *pData;
        pData++;
    }
    return cs;
}

uint8_t taD_FwDownloader(uint32_t FwFileOs)
{
    uint8_t tmpData[133];
    uint8_t Index = 1;
    uint8_t ErrTryS = 0;
    uint32_t CurDataOs;
    uint32_t TranferedBytes = 0;
    TaDownFile CurFile;
    uint8_t RxBuf[3];
    uint32_t     Promp = 200;
    uint8_t Flag = 1;
    //uint8_t str[16] = {'U', 'p', 'd', 'a', 't', 'i', 'n', 'g','.', '.', '.', '.', ' ', ' ', ' ', '%'};

    pSpiFlash->read(pSpiFlash, FwFileOs, (uint8_t *)&CurFile, sizeof(TaDownFile));

    CurDataOs = FwFileOs + sizeof(TaDownFile);
    if (CurFile.FileLen)
    {
        do
        {
            uint32_t ReadBytes;
            uint16_t CheckSum;
            //uint8_t str[16] = {'U', 'p', 'd', 'a', 't', 'i', 'n', 'g','.', '.', '.', '.', ' ', ' ', ' ', '%'};

            if (taD_CheckKey(TadKey_Cancel))
            {
                tad_status = Status_Idle;
                return 0;
            }
            tmpData [0] = SOH;
            tmpData [1] = Index;
            tmpData [2] = ~Index;
            ReadBytes = CurFile.FileLen - TranferedBytes;
            if (ReadBytes > 128)
            {
                pSpiFlash->read(pSpiFlash, CurDataOs, tmpData + 3, 128);
            }
            else
            {
                pSpiFlash->read(pSpiFlash, CurDataOs, tmpData + 3, ReadBytes);
                memset (tmpData + 3 + ReadBytes, 0x00, 128 - ReadBytes);
            }
            CheckSum = xmodem_CheckSumAdd(tmpData + 3, 128);
            tmpData[131] = CheckSum >> 8;
            tmpData[132] = CheckSum & 0xff;

            for (ErrTryS = 0; ErrTryS < 3; ErrTryS++)
            {
                pCom485->control(pCom485, RT_SERIAL_CMD_ClearRxBuff, (void *)RT_NULL);
                pCom485->write(pCom485, 0, tmpData, 133);

                if (usart_GetData(pCom485, RxBuf, 1, 300))	//700
                {
                    if (ACK == RxBuf[0])
                    {
                        uint8_t Percent;
                        uint8_t temp = 0;

                        Index++;
                        CurDataOs += 128;
                        ErrTryS = 0;
                        TranferedBytes += 128;
                        Percent =(uint8_t)((float)TranferedBytes * 100 / CurFile.FileLen);
                        if(!(Percent % 10)  && Percent != temp)
                        {
                            pLcd1602->control(pLcd1602, DispProcessing, &Percent);
                            rt_sem_release(&lcd_writesem);
                            temp = Percent;
                        }
#if 0
                        if(TranferedBytes == 128)
                        {
                            pLcd1602->write(pLcd1602, 16,str, 16);
                        }
                        if(Percent % 10 == 0)
                        {
                            taD_Display(Percent);
                        }
#endif
                        if (Promp)
                        {
                            if (Flag)
                            {
                                GPIO_SetBits(GPIO_PORT_BEEP, GPIO_PIN_BEEP);
                                Flag = 0;
                            }
                            else
                            {
                                GPIO_ResetBits(GPIO_PORT_BEEP, GPIO_PIN_BEEP);
                                Flag = 1;
                            }
                            Promp--;
                        }
                        break;

                    }
                    else
                    {
                        rt_thread_delay(50);
                    }
                }
                else
                {
                    rt_thread_delay(50);
                }
            }
            if (3 <= ErrTryS)
            {
                return 0;
            }
        }
        while(TranferedBytes < CurFile.FileLen);
        tmpData[0] = EOT;
        pCom485->write(pCom485, 0, tmpData, 1);
        return 1;
    }
}

/*根据烧写方式(bootloader烧写和程序中烧写)，将bin文件分成两类*/
File_Type taD_GetFileType(uint8_t objname[])
{
    if(objname[0] == 'T' && objname[2] == 'D')
    {
        return Type_Fw;
    }
    else if(objname[0] == 'T' && objname[2] == 'M')
    {
        return Type_Menu;
    }
    else
        return Type_Osd;
}

/*
  *	功能	: 查询球机主板中的flash 中文件信息
  *    备注	: 菜单0 ~ 200, 0xFF为flash 初始值，说明无菜单包
  */
uint8_t taD_GetCurFontStatus(uint8_t RxBuf[])
{
    uint8_t Reply = 0;
    uint8_t ver_query[8] = {0x99, 0x00, 0x08, 0xF1, 0x0D, 0x00, 0x00, 0x9F};

    pCom485->control(pCom485, RT_SERIAL_CMD_SETBPS, "2400-N-1");

#if 0
    uint16_t i = 50;
    while(i--)
    {
        Delay_1ms(10);
        readNum = pCom485->read(pCom485, 0, &menu_verinfo, 1);
        if(readNum)
        {
            //printf("OK\n");
            break;
        }

    }
#endif

    uint8_t num = 50;
    while(--num)
    {
        pCom485->write(pCom485, 0, ver_query, 8);
        usart_GetData(pCom485, RxBuf, 11, 300);
        if(RxBuf[0] == 0x99 && RxBuf[3] == 0xF1)
        {
            //printf("Query OK\n");
            break;
        }
    }
    if(!num)
    {
        pLcd1602->control(pLcd1602, DispEnd, (void *)0);
        rt_thread_delay(500);
        pLcd1602->write(pLcd1602, 16, "AutoUpdateFail!!", 16);
        return 0;
    }
    rt_thread_delay(50);

    return 1;
}
uint8_t taD_AutoUpdateXfer(ObjInfo Object)
{
    uint8_t tmpData[41];
    uint16_t Index = 1;
    uint8_t ErrTryS = 0;
    uint32_t CurDataOs;
    uint32_t TranferedBytes = 0;
    TaDownFile CurFile;
    uint8_t RxBuf[3];
    uint32_t     Promp = 200;
    uint8_t Flag = 1;

    pSpiFlash->read(pSpiFlash, Object.FontFileOs, (uint8_t *)&CurFile, sizeof(TaDownFile));
    uint32_t tmpOs = Object.TargetOs;	/*获取目标文件的烧写地址*/

    pCom485->control(pCom485, RT_SERIAL_CMD_SETBPS, (void *)BpsStr[1]);
    rt_thread_delay(50);

    /*开始发送数据*/
    CurDataOs = Object.FontFileOs+ sizeof(TaDownFile);

    if (CurFile.FileLen)
    {
        do
        {
            uint32_t ReadBytes;
            uint16_t CheckSum;
            //uint8_t str[16] = {'U', 'p', 'd', 'a', 't', 'i', 'n', 'g','.', '.', '.', '.', ' ', ' ', ' ', '%'};

            if (taD_CheckKey(TadKey_Cancel))
            {
                tad_status = Status_Idle;
                return 0;
            }
            /*4位数据头*/
            tmpData [0] = 0x99;
            tmpData [1] = 0;
            tmpData [2] = 0x29;
            tmpData [3] = 0xAB;

            /*烧写地址*/
            tmpData [4] = (uint8_t)(tmpOs >> 24);
            tmpData [5] = (uint8_t)(tmpOs >> 16);
            tmpData [6] = (uint8_t)(tmpOs >> 8);
            tmpData [7] = (uint8_t)tmpOs;
            tmpOs += 0x20;

            ReadBytes = CurFile.FileLen - TranferedBytes;
            if (ReadBytes > 32)
            {
                pSpiFlash->read(pSpiFlash, CurDataOs, tmpData + 8, 32);
            }
            else
            {
                pSpiFlash->read(pSpiFlash, CurDataOs, tmpData + 8, ReadBytes);
                memset (tmpData + 3 + ReadBytes, 0x00, 32 - ReadBytes);
            }
            CheckSum = xmodem_CheckSumAdd(tmpData, 40);
            //tmpData[40] = CheckSum >> 8;
            tmpData[40] = (uint8_t)(CheckSum & 0xff);

            for (ErrTryS = 0; ErrTryS < 3; ErrTryS++)
            {
                pCom485->control(pCom485, RT_SERIAL_CMD_ClearRxBuff, (void *)RT_NULL);
                pCom485->write(pCom485, 0, tmpData, 41);

                if (usart_GetData(pCom485, RxBuf, 1, 300))	//700
                {
                    if (ACK == RxBuf[0])
                    {
                        uint8_t Percent;
                        uint8_t temp;
#if 0
                        if(Index % 1000 == 0)
                        {
                            printf("%d\n",Index);
                        }
                        Index++;
#endif
                        CurDataOs += 32;
                        ErrTryS = 0;
                        TranferedBytes += 32;
                        Percent =(uint8_t)((float)TranferedBytes * 100 / CurFile.FileLen);
                        if(!(Percent % 10)  && Percent != temp)
                        {
                            pLcd1602->control(pLcd1602, DispProcessing, &Percent);
                            rt_sem_release(&lcd_writesem);
                            temp = Percent;
                        }
#if 0
                        if(TranferedBytes == 32)
                        {
                            pLcd1602->write(pLcd1602, 16,str, 16);
                        }
                        if(Percent != temp)
                        {
                            taD_Display(Percent);
                        }
                        temp = Percent;
#endif
                        if (Promp)
                        {
                            if (Flag)
                            {
                                GPIO_SetBits(GPIO_PORT_BEEP, GPIO_PIN_BEEP);
                                Flag = 0;
                            }
                            else
                            {
                                GPIO_ResetBits(GPIO_PORT_BEEP, GPIO_PIN_BEEP);
                                Flag = 1;
                            }
                            Promp--;
                        }
                        break;

                    }
                    else
                    {
                        rt_thread_delay(50);
                    }
                }
                else
                {
                    rt_thread_delay(50);
                }
            }
            if (3 <= ErrTryS)
            {
                return 0;
            }
        }
        while(TranferedBytes < CurFile.FileLen);
        tmpData[0] = EOT;
        pCom485->write(pCom485, 0, tmpData, 1);
        return 1;
    }
}
uint8_t taD_FontDown(ObjInfo Object)
{

    uint8_t tmpData[41];
    uint8_t connectHead[7] = {0x99, 0x00, 0x07, 0xAD, 0x00};
    uint16_t Index = 1;
    uint8_t ErrTryS = 0;
    uint32_t CurDataOs;
    uint32_t TranferedBytes = 0;
    TaDownFile CurFile;
    uint8_t RxBuf[3];
    uint32_t     Promp = 200;
    uint8_t Flag = 1;
#if 1
    switch(CurBps)
    {
    case 0:
        connectHead[5] = 0x1B;
        connectHead[6] = 0x68;
        break;
    case 1:
    case 2:	/*230400 波特率烧写不稳定，屏蔽*/
        connectHead[5] = 0x1C;
        connectHead[6] = 0x69;
        break;
#if 0
    case 2:
        connectHead[5] = 0xFF;
        connectHead[6] = 0x4C;
        break;
#endif
    default:
        break;
    }
#endif
    pSpiFlash->read(pSpiFlash, Object.FontFileOs, (uint8_t *)&CurFile, sizeof(TaDownFile));
    uint32_t tmpOs = Object.TargetOs;	/*获取目标文件的烧写地址*/

#if 1
    pCom485->control(pCom485, RT_SERIAL_CMD_SETBPS, "2400-N-1");
    rt_thread_delay(50);

    /*首先发送波特率设置信息*/
    uint8_t num = 100;
    while(--num)
    {
        pCom485->write(pCom485, 0, connectHead, 7);
        //Delay_1ms(50);
        usart_GetData(pCom485, RxBuf, 1, 300);
        if(RxBuf[0] == ACK)
        {
            //printf("OK\n");
            break;
        }
    }
    if(!num)
    {
        //printf("Error\n");
        return 0;
    }
#endif

    rt_thread_delay(50);
    //pCom485->control(pCom485, RT_SERIAL_CMD_SETBPS, (void *)BpsStr[CurBps]);
    if(CurBps == 0)
    {
        pCom485->control(pCom485, RT_SERIAL_CMD_SETBPS, (void *)BpsStr[0]);
    }
    else
    {
        pCom485->control(pCom485, RT_SERIAL_CMD_SETBPS, (void *)BpsStr[1]);
    }
    rt_thread_delay(50);

    /*开始发送数据*/
    CurDataOs = Object.FontFileOs+ sizeof(TaDownFile);

#if 0
    uint8_t test[500] = {0};
    pSpiFlash->read(pSpiFlash, 0x0010000, test, 500);
    //st[0] = 0;
    SPI_FLASH_SectorErase(0x10000);
    pSpiFlash->read(pSpiFlash, 0x0010000, test, 500);
#endif

    if (CurFile.FileLen)
    {
        do
        {
            uint32_t ReadBytes;
            uint16_t CheckSum;
            //uint8_t str[16] = {'U', 'p', 'd', 'a', 't', 'i', 'n', 'g','.', '.', '.', '.', ' ', ' ', ' ', '%'};

            if (taD_CheckKey(TadKey_Cancel))
            {
                tad_status = Status_Idle;
                return 0;
            }
            /*4位数据头*/
            tmpData [0] = 0x99;
            tmpData [1] = 0;
            tmpData [2] = 0x29;
            tmpData [3] = 0xAB;

            /*烧写地址*/
            tmpData [4] = (uint8_t)(tmpOs >> 24);
            tmpData [5] = (uint8_t)(tmpOs >> 16);
            tmpData [6] = (uint8_t)(tmpOs >> 8);
            tmpData [7] = (uint8_t)tmpOs;
            tmpOs += 0x20;

            ReadBytes = CurFile.FileLen - TranferedBytes;
            if (ReadBytes > 32)
            {
                pSpiFlash->read(pSpiFlash, CurDataOs, tmpData + 8, 32);
            }
            else
            {
                pSpiFlash->read(pSpiFlash, CurDataOs, tmpData + 8, ReadBytes);
                memset (tmpData + 3 + ReadBytes, 0x00, 32 - ReadBytes);
            }
            CheckSum = xmodem_CheckSumAdd(tmpData, 40);
            //tmpData[40] = CheckSum >> 8;
            tmpData[40] = (uint8_t)(CheckSum & 0xff);

            for (ErrTryS = 0; ErrTryS < 3; ErrTryS++)
            {
                pCom485->control(pCom485, RT_SERIAL_CMD_ClearRxBuff, (void *)RT_NULL);
                pCom485->write(pCom485, 0, tmpData, 41);

                if (usart_GetData(pCom485, RxBuf, 1, 300))	//700
                {
                    if (ACK == RxBuf[0])
                    {
                        uint8_t Percent;
                        uint8_t temp;
#if 0
                        if(Index % 1000 == 0)
                        {
                            printf("%d\n",Index);
                        }
                        Index++;
#endif
                        CurDataOs += 32;
                        ErrTryS = 0;
                        TranferedBytes += 32;
                        Percent =(uint8_t)((float)TranferedBytes * 100 / CurFile.FileLen);
                        if(!(Percent % 10)  && Percent != temp)
                        {
                            pLcd1602->control(pLcd1602, DispProcessing, &Percent);
                            rt_sem_release(&lcd_writesem);
                            //printf("Percent:%d\n", Percent);
                            //printf("1:%d\n", lcd_writesem.value);
                            temp = Percent;
                        }

#if 0
                        if(TranferedBytes == 32)
                        {
                            pLcd1602->write(pLcd1602, 16,str, 16);
                        }
                        if(Percent != temp)
                        {
                            taD_Display(Percent);
                        }
                        temp = Percent;
#endif
                        if (Promp)
                        {
                            if (Flag)
                            {
                                GPIO_SetBits(GPIO_PORT_BEEP, GPIO_PIN_BEEP);
                                Flag = 0;
                            }
                            else
                            {
                                GPIO_ResetBits(GPIO_PORT_BEEP, GPIO_PIN_BEEP);
                                Flag = 1;
                            }
                            Promp--;
                        }
                        break;

                    }
                    else
                    {
                        rt_thread_delay(50);
                    }
                }
                else
                {
                    rt_thread_delay(50);
                }
            }
            if (3 <= ErrTryS)
            {
                return 0;
            }
        }
        while(TranferedBytes < CurFile.FileLen);
        tmpData[0] = EOT;
        pCom485->write(pCom485, 0, tmpData, 1);
        return 1;
    }

}

void taD_ConnectHandle()
{
    /*rt_sem_release(&lcd_writesem);
    rt_sem_release(&lcd_writesem);
    tad_status = Status_Idle;*/
#if 1
    if (SelectedObj.FileType == Type_Menu)//(taD_GetFileType() == Type_Menu)//FONT_FILE_T)
    {
        pCom485->control(pCom485, RT_SERIAL_CMD_ClearRxBuff, (void *)0);
        if(taD_FontDown(SelectedObj))
        {
            FwDownLoadFlag = 1;
            pLcd1602->control(pLcd1602, DispEnd, (void *)0);
            rt_thread_delay(500);
            pLcd1602->write(pLcd1602, 16, "Sucess! ++++++++", 16);
        }
        else
        {
            FwDownLoadFlag = 0;
            pLcd1602->control(pLcd1602, DispEnd, (void *)0);
            rt_thread_delay(500);
            pLcd1602->write(pLcd1602, 16, "Fail!!!!--------", 16);
        }
        pCom485->control(pCom485, RT_SERIAL_CMD_ClearRxBuff, (void *)0);
    }
    else if(SelectedObj.FileType == Type_Osd)
    {
        if(taD_FontDown(SelectedObj))
        {
            FwDownLoadFlag = 1;
            pLcd1602->control(pLcd1602, DispEnd, (void *)0);
            rt_thread_delay(500);
            pLcd1602->write(pLcd1602, 16, "Sucess! ++++++++", 16);
        }
        else
        {
            FwDownLoadFlag = 0;
            pLcd1602->control(pLcd1602, DispEnd, (void *)0);
            rt_thread_delay(500);
            pLcd1602->write(pLcd1602, 16, "Fail!!!!--------", 16);
        }
    }
    else
    {
        //taD_AutoUpdateOsd();
#if 1
        if (taD_ConnectBootLoader(2400, 115200, 200))
        {

            if(taD_FwDownloader(SelectedObj.FontFileOs))
            {
                FwDownLoadFlag = 1;
                pLcd1602->control(pLcd1602, DispEnd, (void *)0);
                rt_thread_delay(500);
                pLcd1602->write(pLcd1602, 16, "Sucess! ++++++++", 16);
                //rt_thread_delay(2000);  /*等待球机烧完程序之后重启*/
                pCom485->control(pCom485, RT_SERIAL_CMD_ClearRxBuff, (void *)0);

                if(AutoUpdateEn)
                {
                    uint8_t query[7] = {0x99, 0x00, 0x07, 0xAD, 0x00, 0x1C, 0x69};
                    uint8_t reply[11] = {0};
                    uint8_t times = 0;

                    pCom485->control(pCom485, RT_SERIAL_CMD_SETBPS, "2400-N-1");
                    pLcd1602->control(pLcd1602, DispEnd, (void *)0);
                    rt_thread_delay(500);
                    pLcd1602->write(pLcd1602, 0, "Auto update ....", 16);
                    pLcd1602->write(pLcd1602, 16, "Waiting ........", 16);

                    while(!usart_GetData(pCom485, reply, 11, 300) && times < 100)
                    {
                        rt_thread_delay(200);
                        times++;
                        pCom485->write(pCom485, 0, query, 7);
                    }

                    if(reply[0] == 0x99 && reply[10])
                    {
                        //printf("get status ok!\n");
                        if(taD_AutoUpdateOsd(reply))
                        {
                            pLcd1602->control(pLcd1602, DispEnd, (void *)0);
                            rt_thread_delay(500);
                            pLcd1602->write(pLcd1602, 16, "Sucess! ++++++++", 16);
                        }
                        else
                        {
                            pLcd1602->control(pLcd1602, DispEnd, (void *)0);
                            rt_thread_delay(500);
                            pLcd1602->write(pLcd1602, 16, "Fail!!!!--------", 16);
                        }
                    }

                    pCom485->control(pCom485, RT_SERIAL_CMD_SETBPS, (void *)BpsStr[CurBps]);

                    rt_thread_delay(50);

                }
            }
            else
            {
                FwDownLoadFlag = 0;
                pLcd1602->control(pLcd1602, DispEnd, (void *)0);
                rt_thread_delay(500);
                pLcd1602->write(pLcd1602, 16, "Fail!!!!--------", 16);
            }
        }
        else
        {
            FwDownLoadFlag = 0;
            pLcd1602->control(pLcd1602, DispEnd, (void *)0);
            rt_thread_delay(500);
            pLcd1602->write(pLcd1602, 16, "Connect Fail!!!!", 16);
        }
#endif
    }
    TaBeepThread_entry();
    tad_status = Status_Idle;
    pLcd1602->control(pLcd1602, DispEnd, (void *)0);
    rt_thread_delay(2000);
    pLcd1602->write(pLcd1602, -1, SelectedObj.FileName, 32);
#endif
    return;
}

uint8_t taD_AutoUpdateOsd(uint8_t* reply)
{
    if(!reply[5])	/*球机flash中无pinyin，更新*/
    {
        if(rt_mutex_take(LcdMutex, RT_WAITING_FOREVER) == RT_EOK)
        {
            pLcd1602->write(pLcd1602, 0, "pinyin_tol.bin..", 16);
            rt_mutex_release(LcdMutex);
        }

        if(!taD_AutoUpdateXfer(*thePinyinObj))
        {
            return 0;
        }
    }

    if(!reply[7])
    {
        if(rt_mutex_take(LcdMutex, RT_WAITING_FOREVER) == RT_EOK)
        {
            pLcd1602->write(pLcd1602, 0, "hzk14.bin......", 16);
            rt_mutex_release(LcdMutex);
        }

        if(!taD_AutoUpdateXfer(*theHzkobj))
        {
            return 0;
        }
    }

    if(!reply[6])
    {
        if(rt_mutex_take(LcdMutex, RT_WAITING_FOREVER) == RT_EOK)
        {
            pLcd1602->write(pLcd1602, 0, "asc16_spec.bin..", 16);
            rt_mutex_release(LcdMutex);
        }

        if(!taD_AutoUpdateXfer(*theASCIIObj))
        {
            return 0;
        }
    }

    if(!reply[8])
    {
        if(rt_mutex_take(LcdMutex, RT_WAITING_FOREVER) == RT_EOK)
        {
            pLcd1602->write(pLcd1602, 0, "TaMenuFile......", 16);
            rt_mutex_release(LcdMutex);
        }

        if(!taD_AutoUpdateXfer(*theMenuObj))
        {
            return 0;
        }
    }
    else if(reply[9] < (theMenuObj->FileName[19] - 48))
    {
        if(rt_mutex_take(LcdMutex, RT_WAITING_FOREVER) == RT_EOK)
        {
            pLcd1602->write(pLcd1602, 0, "TaMenuFile......", 16);
            rt_mutex_release(LcdMutex);
        }

        if(!taD_AutoUpdateXfer(*theMenuObj))
        {
            return 0;
        }
    }
    return 1;
}

const uint16_t SucessBeep[] = {500, 200, 1000, 200, 200, 100, 200, 100};
const uint16_t FailBeep[] = {2000, 100, 2000, 100, 2000, 100, 2000, 100};

void TaBeepThread_entry()
{
    uint8_t i, Flag = 1;
    for (i = 0; i < 8; i++)
    {
        if (Flag)
        {
            GPIO_SetBits(GPIO_PORT_BEEP, GPIO_PIN_BEEP);
            Flag = 0;
        }
        else
        {
            GPIO_ResetBits(GPIO_PORT_BEEP, GPIO_PIN_BEEP);
            Flag = 1;
        }
        if (FwDownLoadFlag)
        {
            rt_thread_delay(SucessBeep[i]);
        }
        else
        {
            rt_thread_delay(FailBeep[i]);
        }
    }
}

void TaDownLoaderInit()
{
    uint8_t FileNum, i;
    uint32_t tmpFileOs;

    I2c_Soft_Init(1);
    I2c_Soft_E2IcTypeSet(USE_24C256);

    tad_status = Status_Idle;
    taD_KeyPortInit();

    pSpiFlash = rt_device_find("flash");
    pCom232 = rt_device_find("uart1");
    pCom485 = rt_device_find("485");
    pLcd1602 = rt_device_find("Lcd1602");

    FileNum = ScanFlash();

    CurBps = ReadByte(BPS_SETTING_OS);
    if(CurBps > 2)
    {
        CurBps = 2;
    }
    AutoUpdateEn = ReadByte(AutoUpdate_SET_OS);
    if(AutoUpdateEn >= 2)
    {
        AutoUpdateEn = 0;
    }

    if (CurBps > 2)
    {
        CurBps = 2;
        WriteByte(BPS_SETTING_OS, CurBps);
    }
    pCom485->control(pCom485, RT_SERIAL_CMD_SETBPS, (void *)BpsStr[CurBps]);

    tmpFileOs = ReadDword(FILE_SEL_OS);

#if 0
    if (tmpFileOs % 0x40000)
        tmpFileOs = (tmpFileOs / 0x40000) * 0x40000;
    if (tmpFileOs > 0x1C0000)
        tmpFileOs = 0;
#endif

    for (i = 0; i < 10; i++)
    {
        if (tmpFileOs == flashobj[i].FontFileOs)
        {
            //SelectedObj.FontFileOs= tmpFileOs;
            memcpy(&SelectedObj, &flashobj[i], sizeof(ObjInfo));
            break;
        }
    }
    if (i >= 10)
    {
        //SelectedObj.FontFileOs = flashobj[0].FontFileOs;
        memcpy(&SelectedObj, &flashobj[i], sizeof(ObjInfo));
        WriteDword(FILE_SEL_OS, SelectedObj.FontFileOs);
    }
    //memcpy(SelectedObj.FileName, flashobj[i].FileName, 64);
    pLcd1602->write(pLcd1602, -1, SelectedObj.FileName, 32);


#if 0
    FwDownLoadFlag = 1;
    rt_thread_startup(BeepThread);
    rt_thread_delay(9000);
    FwDownLoadFlag = 0;
    rt_thread_delay(9000);
    BeepTimeOutProc((void *)0);
#endif


#if 0
    uint8_t pTestBuffer[32];
    {
        const uint8_t TestData[32] = {"We will be great!"};
        const uint8_t Addr[][32] =
        {
            {"Cur Address : \n    0x00000"},
            {"Cur Address : \n    0x40000"},
            {"Cur Address : \n    0x80000"},
            {"Cur Address : \n    0xC000"},
            {"Cur Address : \n    0x100000"},
            {"Cur Address : \n    0x140000"},
            {"Cur Address : \n    0x180000"},
            {"Cur Address : \n    0x1C0000"},
        };
        uint8_t i;

        for (i = 0; i < 8; i++)
        {
            pLcd1602->write(pLcd1602, -1, Addr[i], 32);

            rt_thread_delay(2000);

            pSpiFlash->write(pSpiFlash, i * 0x40000, TestData, 32);

            memset (pTestBuffer, 0, 32);

            pSpiFlash->read(pSpiFlash, i * 0x40000, pTestBuffer, 32);

            pLcd1602->write(pLcd1602, -1, pTestBuffer, 32);

            rt_thread_delay(4000);
        }
    }
#endif
    ComTimer = rt_timer_create("ComTO", ComTimeOutHandle, (void *)0, 2000, 1);		//3秒钟后若无串口数据则清除接收标记位
    //ComTimer = rt_timer_create("ComTimeOut", ComTimeOutHandle, (void *)0, 5000, 1);
}

extern struct rt_mutex SysInit;

void rt_TaDownLoader_thread_entry(void* parameter)
{
    rt_thread_delay(1000);
#if 1
    TaDownLoaderInit();

    while (1)
    {
        switch (tad_status)
        {
        case Status_Menu:
            taD_MenuHandle();
            break;
        case Status_WaitingConnect:
            taD_ConnectHandle();
            break;
        default:
            taD_IdleHandle();
            break;
        }
    }
#endif
}

void taD_Display(uint8_t percent)
{
    uint8_t str[3] = {0};//= {'U', 'p', 'd', 'a', 't', 'e', '.', '.','.', '.', '.', '.','%'};

    sprintf(str, "%d", percent);
    pLcd1602->write(pLcd1602, 28, str, 3);

}
