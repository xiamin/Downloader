/*
 * File      : board.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      first implementation
 */

#include <rthw.h>
#include <rtthread.h>

#include "stm32f10x.h"
#include "board.h"

#ifdef RT_USING_SPI
#include "stm32f10x_spi.h"
#include "rt_stm32f10x_spi.h"
#endif

/**
 * @addtogroup STM32
 */

/*@{*/

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
#ifdef  VECT_TAB_RAM
	/* Set the Vector Table base location at 0x20000000 */
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */
	/* Set the Vector Table base location at 0x08000000 */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x2000);		//0x0
#endif
}

/**
 * This is the timer interrupt service routine.
 *
 */
void rt_hw_timer_handler(void)
{
	/* enter interrupt */
	rt_interrupt_enter();

	rt_tick_increase();

	/* leave interrupt */
	rt_interrupt_leave();
}

void drv_SysInit(void)
{
    __IO uint32_t i;

    SystemInit();

    for(i = 0; i < (0x36EE80>>4); i++);

    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
}

/**
 * This function will initial STM32 board.
 */
void rt_hw_board_init()
{
    drv_SysInit();
    
	/* NVIC Configuration */
	NVIC_Configuration();

    /* Configure the SysTick */
    SysTick_Config( SystemCoreClock / RT_TICK_PER_SECOND );

	rt_hw_usart_init();
#if defined(RT_USING_DEVICE) && defined(RT_USING_CONSOLE)
	rt_console_set_device(CONSOLE_DEVICE);
#endif


#ifdef RT_USING_SPI
    rt_stm32f10x_spi_init();

    #ifdef USING_SPI1
    /* attach spi10 : CS PA4 */
    {
        static struct rt_spi_device rt_spi_device;
        static struct stm32_spi_cs  stm32_spi_cs;
        GPIO_InitTypeDef GPIO_InitStructure;

        stm32_spi_cs.GPIOx = GPIOA;
        stm32_spi_cs.GPIO_Pin = GPIO_Pin_4;

        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_Init(stm32_spi_cs.GPIOx, &GPIO_InitStructure);

        GPIO_SetBits(stm32_spi_cs.GPIOx, stm32_spi_cs.GPIO_Pin);

        rt_spi_bus_attach_device(&rt_spi_device, "spi10", "spi1", (void*)&stm32_spi_cs);
    }

#endif
#endif /* #ifdef RT_USING_SPI */
}

/*@}*/
