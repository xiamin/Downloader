
#ifndef __LCD1602_H
#define __LCD1602_H

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include <rtthread.h>

#define LCD1602_RS_PIN			GPIO_Pin_8
#define LCD1602_RS_PORT		GPIOB

#define LCD1602_EN_PIN			GPIO_Pin_1
#define LCD1602_EN_PORT		GPIOC

#define LCD1602_RW_PIN			GPIO_Pin_15
#define LCD1602_RW_PORT		GPIOB

#define LCD1602_DB_PORT		GPIOC
#define LCD1602_DB_MASK    0x03C0
#define LCD1602_DB_Offset	6

typedef enum _LCD1602_CMD_
{
    DispClear,
    DispConnecting,
    DispProcessing,
    DispEnd,
    DispNull
}Lcd1602Cmd_t;

typedef struct _Lcd_Data_ {
    Lcd1602Cmd_t CurCmd;
    uint8_t Percent;
    uint8_t CurStartIndex;
    uint8_t StrSize;
    uint8_t DispBuffer[64];
}LcdData_t;

void LCD_init(void);
void LCD_en_write(void);
void LCD_write_command(unsigned char command);

void LCD_write_data(unsigned char Recdata);
void LCD_set_xy( unsigned char x, unsigned char y );
void LCD_write_char(unsigned char X,unsigned char Y,unsigned char Recdata);
int LCD_PutStr(unsigned char *DData,int pos, int size);
rt_err_t rt_lcd1602_init(rt_device_t dev);
rt_err_t rt_lcd1602_open(rt_device_t dev, rt_uint16_t oflag);
rt_err_t rt_lcd1602_close(rt_device_t dev);
rt_size_t rt_lcd1602_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size);
rt_size_t rt_lcd1602_write (rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size);
rt_err_t rt_lcd1602_control(rt_device_t dev, rt_uint8_t cmd, void *args);
void rt_hw_lcd1602_init();

extern struct rt_semaphore lcd_writesem;
extern rt_mutex_t LcdMutex;
#endif
