
#include "rtthread.h"
#include "stm32f10x_gpio.h"
#include "tad_LCD.h"
#include "lib_Misc.h"
#include "rtdef.h"
#include "taD_Downloader.h"

void lcd1602_FileNameRoll(void *parameter);
void lcd1602_ConnectDispThread(void *parameter);
void lcd1602_DispProcessing(uint8_t percent);

struct rt_semaphore lcd_writesem;

void LCD_GPIO_Init()
{
    GPIO_InitTypeDef Gpio_InitSt;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

    Gpio_InitSt.GPIO_Pin = LCD1602_RS_PIN | LCD1602_RW_PIN;
    Gpio_InitSt.GPIO_Mode = GPIO_Mode_Out_PP;
    Gpio_InitSt.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LCD1602_RS_PORT, &Gpio_InitSt);

    Gpio_InitSt.GPIO_Pin = LCD1602_EN_PIN | LCD1602_DB_MASK;
    Gpio_InitSt.GPIO_Mode = GPIO_Mode_Out_PP;
    Gpio_InitSt.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LCD1602_DB_PORT, &Gpio_InitSt);
}

unsigned char LCD_SWAP_High4Bits(unsigned char Data)
{
    uint8_t i, tmpData = 0;

    tmpData = Data & 0x0F;

    for (i = 0; i < 4; i++)
    {
        if (Data & (1 << (i + 4)))
        {
            tmpData |= (1 << (7 - i));
        }
        else
        {
            tmpData &= ~(1 << (7 - i));
        }
    }
    return tmpData;
}

void w_4bit_INIT_LCD1602(unsigned char ucCMD)
{
    uint16_t PORT_DATA, tmpCmd = 0;
    GPIO_ResetBits(LCD1602_RS_PORT, LCD1602_RS_PIN);		//RS_L;  //写入的是命令字
    GPIO_ResetBits(LCD1602_RW_PORT, LCD1602_RW_PIN);    	//RW_W;  //置为写状态

    PORT_DATA = GPIO_ReadInputData(LCD1602_DB_PORT);
    PORT_DATA &= (~LCD1602_DB_MASK);

    tmpCmd = (uint16_t)(LCD_SWAP_High4Bits(ucCMD) & 0xf0) << 2;
    PORT_DATA |= tmpCmd;
    GPIO_Write(LCD1602_DB_PORT, PORT_DATA);			//    写高四位
    //P1 = (P1 & 0x0F) | (ucCMD & 0xF0);  //注意不要改变P2低四位的状态

    LCD_en_write();    //En_Toggle();  //产生使能脉冲，使之在下降沿开始执行指令
}

void Wait_Until_Ready(void)
{
    GPIO_ResetBits(LCD1602_RS_PORT, LCD1602_RS_PIN);		//RS_L;  //所读为状态位
    GPIO_SetBits(LCD1602_RW_PORT, LCD1602_RW_PIN);    	//RW_R;  //设为读状态  En_H;
    Delay_1ms(1);		//delay_nus(10);
    //while(!(P1^7 == 0)); //不断循环，直至BF=0
    GPIO_ResetBits(LCD1602_EN_PORT, LCD1602_EN_PIN);		//En_L;
}

void LCD_init(void)      //
{
    LCD_GPIO_Init();
#if 1
    Delay_1ms(20);
    w_4bit_INIT_LCD1602(0x30);     //写第一次0x3N命令（N为任意值）
    Delay_1ms(10);       //至少延迟4.1ms
    w_4bit_INIT_LCD1602(0x30);    //写第二次0x3N命令（N为任意值）
    Delay_1us(200);       //至少延迟100us
    w_4bit_INIT_LCD1602(0x30);    //写第三次0x3N命令（N为任意值）
    Delay_1us(100);       //至少延迟40us
    w_4bit_INIT_LCD1602(0x20);    //设置为4位模式
    Delay_1us(100);        //至少延迟40us
    LCD_write_command(0x28);      //设置为4位模式，2行字符，5 x 7点阵/每字符
    Delay_1us(100);       //至少延迟40us
    LCD_write_command(0x06);     //写入新数据后光标右移，写入新数据后显示屏不移动
    Delay_1us(100);       //至少延迟40us
    LCD_write_command(0x0e);     //显示功能开，有光标，光标不闪烁
    Delay_1us(100);       //至少延迟40us
    LCD_write_command(0x01);     //清除液晶显示器
    Delay_1ms(10);          //至少延迟1.64ms
    GPIO_ResetBits(LCD1602_RW_PORT, LCD1602_RW_PIN);
    Delay_1ms(10);
#else
    LCD_write_command(0x33);
    Delay_1ms(5);

    LCD_write_command(0x28);
    Delay_1ms(5);

    LCD_write_command(0x28);
    Delay_1ms(5);

    LCD_write_command(0x28);
    Delay_1ms(5);

    LCD_en_write();
    Delay_1ms(5);

    LCD_write_command(0x28); //4
    Delay_1ms(5);

    LCD_write_command(0x0c); //    显示开
    Delay_1ms(5);

    LCD_write_command(0x01); //    清屏
    Delay_1ms(5);

    GPIO_ResetBits(LCD1602_RW_PORT, LCD1602_RW_PIN);
#endif
}

void LCD_en_write(void)    //液晶使能
{
    Delay_1ms(1);
    GPIO_SetBits(LCD1602_EN_PORT, LCD1602_EN_PIN);

    Delay_1ms(1);

    GPIO_ResetBits(LCD1602_EN_PORT, LCD1602_EN_PIN);
    Delay_1ms(1);
}

void LCD_write_command(unsigned char command)    //写指令
{
    uint16_t PORT_DATA, tmpCmd = 0;

    //LCD1602_RS=0; //RS=0
    GPIO_ResetBits(LCD1602_RS_PORT, LCD1602_RS_PIN);
    Delay_1ms(2);

    PORT_DATA = GPIO_ReadInputData(LCD1602_DB_PORT);
    PORT_DATA &= (~LCD1602_DB_MASK);
    //GPIO_Write(LCD1602_DB_PORT, PORT_DATA);

    tmpCmd = ((uint16_t)(LCD_SWAP_High4Bits(command) & 0xf0)) << 2;
    PORT_DATA |= tmpCmd;
    GPIO_Write(LCD1602_DB_PORT, PORT_DATA);		//    写高四位

    LCD_en_write();

    command=command<<4; //    低四位移到高四位

    PORT_DATA = GPIO_ReadInputData(LCD1602_DB_PORT);
    PORT_DATA &= (~LCD1602_DB_MASK);
    //GPIO_Write(LCD1602_DB_PORT, PORT_DATA);

    tmpCmd = 0;
    tmpCmd = ((uint16_t)(LCD_SWAP_High4Bits(command) & 0xf0)) << 2;
    PORT_DATA |= tmpCmd;
    GPIO_Write(LCD1602_DB_PORT, PORT_DATA);   //    写低四位
    //LCD_DATA|=command&0xf0;

    LCD_en_write();
}

void LCD_write_data(unsigned char Recdata) //写数据
{
    uint16_t PORT_DATA, tmpCmd = 0;
    Delay_1ms(2);

    //LCD1602_RS=1; //RS=1
    GPIO_SetBits(LCD1602_RS_PORT, LCD1602_RS_PIN);
    Delay_1ms(2);

    PORT_DATA = GPIO_ReadInputData(LCD1602_DB_PORT);
    PORT_DATA &= (~LCD1602_DB_MASK);

    tmpCmd = (uint16_t)(LCD_SWAP_High4Bits(Recdata) & 0xf0) << 2;
    PORT_DATA |= tmpCmd;
    GPIO_Write(LCD1602_DB_PORT, PORT_DATA);		//    写高四位
    //LCD_DATA&=0X0f; //    清高四位
    //LCD_DATA|=Recdata&0xf0; //    写高四位

    LCD_en_write();

    Recdata=Recdata<<4; //    低四位移到高四位

    PORT_DATA = GPIO_ReadInputData(LCD1602_DB_PORT);
    PORT_DATA &= (~LCD1602_DB_MASK);

    tmpCmd = 0;
    tmpCmd = (uint16_t)(LCD_SWAP_High4Bits(Recdata) & 0xf0) << 2;
    PORT_DATA |= tmpCmd;
    GPIO_Write(LCD1602_DB_PORT, PORT_DATA);   //    写低四位
    //LCD_DATA&=0X0f; //    清高四位
    //LCD_DATA|=Recdata&0xf0; //    写低四位

    LCD_en_write();
}

void LCD_set_xy( unsigned char x, unsigned char y ) //写地址函数
{
    unsigned char address;

    if (y == 0) address = 0x80 + x;
    else address = 0xc0 + x;
    LCD_write_command(address);
}

void LCD_write_char(unsigned char X,unsigned char Y,unsigned char Recdata) //列x=0~15,行y=0,1
{
    LCD_set_xy(X, Y); //    写地址

    LCD_write_data(Recdata);
}

int LCD_PutStr(unsigned char *DData,int pos, int size)
{
    unsigned char i;

    if(pos==-1)
    {
        LCD_write_command(0x01); //清屏
        rt_thread_delay(2);
        pos=0;
    }

    while ((*DData)!='\0' && size)
    {
        switch(*DData)
        {
        case '\n': //如果是\n，则换行
        {
            if(pos<17)
            {
                for(i=pos; i<16; i++) LCD_write_char(i%16, i/16, ' ');
                pos=16;
            }
            else
            {
                for(i=pos; i<32; i++) LCD_write_char(i%16, i/16, ' ');
                pos=0;
            }
            break;
        }
        case '\b': //如果是\b，则退格
        {
            if(pos>0) pos--;
            LCD_write_char(pos%16, pos/16, ' ');
            break;
        }
        default:
        {
            if((*DData)<0x20)
            {
                *DData=' ';
            }
            LCD_write_char(pos%16, pos/16, *DData);
            pos++;
            break;
        }
        }
        DData++;
        size--;
    }
    return(pos);
}


static struct rt_device lcd1602;

static LcdData_t  LcdUserData;

rt_mutex_t LcdMutex;

rt_thread_t  lcd_thread_filename;
rt_thread_t  lcd_thread_connect;

void lcd1602_FileNameRollStart()
{
    lcd_thread_filename = rt_thread_create("lcdFN", lcd1602_FileNameRoll, (void *)0, 1024, 7, 20);

    if (lcd_thread_filename != RT_NULL)
    {
        rt_thread_startup(lcd_thread_filename);
    }
}

void lcd1602_FileNameRollExit()
{
    rt_thread_delete(lcd_thread_filename);
}

void lcd1602_ConectStart()
{
    lcd_thread_connect = rt_thread_create("lcdFN", lcd1602_ConnectDispThread, (void *)0, 1024, 7, 20);

    if (lcd_thread_connect != RT_NULL)
    {
        rt_thread_startup(lcd_thread_connect);
    }
}

void lcd1602_ConectExit()
{
    rt_thread_delete(lcd_thread_connect);
}

static rt_err_t rt_lcd1602_init(rt_device_t dev)
{
    LCD_init();
    return RT_EOK;
}

static rt_err_t rt_lcd1602_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

static rt_err_t rt_lcd1602_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_size_t rt_lcd1602_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
    return 0;
}

static rt_size_t rt_lcd1602_write (rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
    uint32_t i, num;
    uint8_t *pBuf;
    pBuf = (uint8_t *)buffer;
    num = size;
    LCD_PutStr((unsigned char *)buffer, pos, size);
    for (i = 0; i < 64; i++)
    {
        if (*pBuf != 0)
        {
            LcdUserData.DispBuffer[i] = *(const uint8_t *)pBuf;
            pBuf++;
        }
        else
        {
            break;
        }
        num--;
        if (!num)
        {
            break;
        }
    }
    for (num = 0; num < 3; num++)
    {
        LcdUserData.DispBuffer[i] = ' ';
        i++;
        if (i >= 64)
        {
            break;
        }
    }
    LcdUserData.StrSize = i;
    return 0;
}

static rt_err_t rt_lcd1602_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    switch (cmd)
    {
    case DispClear:
        //LcdUserData.CurCmd = DispEnd;
        LCD_write_command(0x01);
        break;
    case DispConnecting:
        LcdUserData.CurCmd = DispConnecting;
        LcdUserData.CurStartIndex = 0;
        //if (lcd_thread_filename != RT_NULL)
            //rt_thread_resume(lcd_thread_filename);
        //if (lcd_thread_connect != RT_NULL)
            //rt_thread_resume(lcd_thread_connect);

        break;
    case DispProcessing:
        LcdUserData.CurCmd = DispProcessing;
        LcdUserData.Percent = *((uint8_t*)args);
        break;
    case DispEnd:

        LcdUserData.CurCmd = DispEnd;
        break;
    }
    return RT_EOK;
}

void rt_hw_lcd1602_init()
{
    LCD_init();

    LcdMutex = rt_mutex_create( "Lcd", RT_IPC_FLAG_FIFO);

    rt_sem_init(&(lcd_writesem), "lcdlock", 0, 0);
    
    LcdUserData.CurCmd = DispEnd;
    //lcd1602_FileNameRollStart();
    lcd1602_ConectStart();

    /* register sdcard device */
    lcd1602.type  = RT_Device_Class_Char;
    lcd1602.init 	= rt_lcd1602_init;
    lcd1602.open 	= rt_lcd1602_open;
    lcd1602.close = rt_lcd1602_close;
    lcd1602.read 	= rt_lcd1602_read;
    lcd1602.write = rt_lcd1602_write;
    lcd1602.control = rt_lcd1602_control;

    /* no private */
    lcd1602.user_data = &LcdUserData;

    rt_device_register(&lcd1602, "Lcd1602", RT_DEVICE_FLAG_WRONLY);

    lcd1602.write(&lcd1602, 0, "TA DownLoader Staring...", 0xff);

    /*
    if (lcd_thread_filename != RT_NULL)
    {
        rt_thread_startup(lcd_thread_filename);
        //rt_thread_suspend(lcd_thread_filename);
    }
    */
    //lcd_thread_connect =  rt_thread_create("lcdCon", lcd1602_ConnectDispThread, (void *)0, 1024, 7, 20);
    /*
    if (lcd_thread_connect != RT_NULL)
    {
        rt_thread_startup(lcd_thread_connect);
        rt_thread_suspend(lcd_thread_connect);
    }
    lcd1602.write(&lcd1602, 0, "TA DownLoader Staring...", 0xff);
    */
}

uint8_t tmpDispStr[17];

void lcd1602_FileNameRoll(void *parameter)
{
    uint8_t i, Index, StartIndex = 0;
    uint8_t tmpStr[17];
    rt_err_t rtResult;

    while (1)
    {
        while (!(LcdUserData.CurCmd == DispConnecting || LcdUserData.CurCmd == DispProcessing))
        {
            rt_thread_delay(100);	//(lcd_thread_filename);
        }
        memset (tmpDispStr, 0, 17);
        Index = LcdUserData.CurStartIndex;
        for (i = 0; i < 16; i++)
        {
            tmpDispStr[i] = LcdUserData.DispBuffer[Index];
            Index++;
            if (Index >= LcdUserData.StrSize)
            {
                Index = 0;
            }
        }
        tmpDispStr[16] = 0;
        rtResult = rt_mutex_take(LcdMutex, RT_WAITING_FOREVER);
        if (rtResult == RT_EOK)
        {
            LCD_PutStr(tmpDispStr, 0, 16);

            rt_mutex_release(LcdMutex);
        }
        if (LcdUserData.CurCmd == DispProcessing)
        {
            lcd1602_DispProcessing(LcdUserData.Percent);
        }
        rt_thread_delay(500);
        LcdUserData.CurStartIndex++;
        if (LcdUserData.CurStartIndex >= LcdUserData.StrSize)
        {
            LcdUserData.CurStartIndex = 0;
        }
    }
}

const uint8_t ConForwardStr[] = {"               >>>>>                "};
const uint8_t ConRevStr[] = {"               <<<<<               "};

void lcd1602_ConnectDispThread(void *parameter)
{
    uint8_t tmpStr[17];
    uint8_t i, j, k;
    rt_err_t rtResult;
    uint8_t str[16] = {'U', 'p', 'd', 'a', 't', 'i', 'n', 'g','.', '.', '.', '.', ' ', ' ', ' ', '%'};
    uint8_t connect[16] ={'C', 'o', 'n', 'n', 'e', 'c', 't', 'i', 'n', 'g', '.', '.', '.', ' ', ' ', ' '};
    //uint8_t clearbuf[] = {"                "};
    uint8_t percent[3] = {0};
    while(1)
    {
        if (rt_sem_take(&lcd_writesem, RT_WAITING_FOREVER) != RT_EOK) continue;
        //printf("2:%d\n", lcd_writesem.value);
        switch(LcdUserData.CurCmd)
        {
        case DispProcessing:
            pLcd1602->write(pLcd1602, 16,str, 16);
            sprintf(percent, "%d", LcdUserData.Percent);
            pLcd1602->write(pLcd1602, 28, percent, 3);
            break;
        case DispConnecting:
            pLcd1602->write(pLcd1602, 16, connect, 16);
            break;
        case DispEnd:
            //pLcd1602->write(pLcd1602, 16, clearbuf, 16);
            break;
        default:
            break;
        }
    }
#if 0
    while (1)
    {
        tmpStr[16] = 0;
        k = 19;
        for (i = 0; i < 16; i++)
        {
            for (j = 0; j < 16; j++)
            {
                tmpStr[j] = ConForwardStr[k + j];
            }
            k--;
            rtResult = rt_mutex_take(LcdMutex, RT_WAITING_FOREVER);
            if (rtResult == RT_EOK)
            {
                LCD_PutStr(tmpStr, 16, 16);
                rt_mutex_release(LcdMutex);
            }
            if (LcdUserData.CurCmd != DispConnecting)
            {
                while (LcdUserData.CurCmd != DispConnecting)
                {
                    rt_thread_delay(100);
                }
            }
            rt_thread_delay (400);
        }
        k = 0;
        for (i = 0; i < 16; i++)
        {
            for (j = 0; j < 16; j++)
            {
                tmpStr[j] = ConRevStr[k + j];
            }
            k++;
            rtResult = rt_mutex_take(LcdMutex, RT_WAITING_FOREVER);
            if (rtResult == RT_EOK)
            {
                LCD_PutStr(tmpStr, 16, 16);
                rt_mutex_release(LcdMutex);
            }
            if (LcdUserData.CurCmd != DispConnecting)
            {
                while (LcdUserData.CurCmd != DispConnecting)
                {
                    rt_thread_delay(100);
                }
            }
            rt_thread_delay (400);
        }

    }
#endif
}

void lcd1602_DispProcessing(uint8_t percent)
{
    uint8_t i, BlockNum = 0;
    uint8_t abyString[17];

    rt_err_t rtResult;
    //while (1)
    {
        BlockNum = ((uint32_t)percent * 16) / 99;
        for (i = 0; i < 16; i++)
        {
            if (i < BlockNum)
            {
                abyString [i] = '#';
            }
            else
            {
                abyString [i] = ' ';
            }
        }
        abyString[16] = 0;
        rtResult = rt_mutex_take(LcdMutex, RT_WAITING_FOREVER);
        if (rtResult == RT_EOK)
        {
            LCD_PutStr(abyString, 16, 16);
            rt_mutex_release(LcdMutex);
        }
    }
}

void lcd1602_thread_entry(void *parameter)
{
    while (1)
    {
        switch (LcdUserData.CurCmd)
        {
        case DispConnecting:
            break;
        case DispProcessing:

            break;
        case DispEnd:
            break;
        }
    }
}
