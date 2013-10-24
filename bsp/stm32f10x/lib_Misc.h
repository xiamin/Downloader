
#ifndef __LIB_MISC_H_
#define __LIB_MISC_H_

/**********72Mhz下时钟计数值 ***********/
#define SYSCLK_ONE_SEC		(0x44AA200>>4)
#define SYSCLK_500_MS			(0x2255100>>4)
#define SYSCLK_250_MS			(0x112A800>>4)
#define SYSCLK_125_MS			(0x895400>>4)
#define SYSCLK_50_MS			(0x36EE80>>4)
#define SYSCLK_ONE_MS		(0x11940>>4)
#define SYSCLK_ONE_US		(0x48 >> 4)

typedef uint8_t         BYTE;
typedef uint8_t*        PBYTE;
typedef uint16_t        WORD;
typedef int16_t         SWORD;
typedef uint32_t        DWORD;
typedef int32_t         SDWORD;
typedef int8_t          CHAR;
typedef float           FLOAT;
typedef BYTE            BIT;

#define MEMSET memset
#define MEMCPY memcpy

#define CONST           const
/*#define TRUE            1
#define FALSE           0    */    //在stm32F10x.h中已有定义

/* #define BIT          bit */
#define VOID            void

VOID While_Delay(__IO DWORD wMs);
VOID While_DelayUs(__IO DWORD dwUs);
VOID Delay_1us(DWORD dwUs);
VOID Delay_10ms(BYTE by10ms);
VOID Delay_1ms(BYTE byms);

#endif
