
/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#include <board.h>
#include <rtthread.h>
#include "tad_LCD.h"
#include "tad_SpiFlash.h"
#include "tad_Downloader.h"

#ifdef RT_USING_SPI
#include <spi_flash_sst25vfxx.h>
#endif

#ifdef RT_USING_DFS
/* dfs init */
#include <dfs_init.h>
/* dfs filesystem:ELM filesystem init */
#include <dfs_elm.h>
/* dfs Filesystem APIs */
#include <dfs_fs.h>
#endif

struct rt_mutex Mutex1;

struct rt_mutex SysInit;
    
void rt_mutex_init_demo()
{
    rt_err_t rtResult;
    rt_mutex_init(&Mutex1, "ABC", 0);
    rtResult = rt_mutex_take(&Mutex1, RT_WAITING_FOREVER);
    if (rtResult == RT_EOK)
    {
        rt_thread_delay(2000);
        unsigned char i;
        for (i = 0; i < 3; i++)
        {
            unsigned char abyTmp[64];
            rt_device_t plcd1602;
            plcd1602 = rt_device_find("Lcd1602");
            sprintf(abyTmp, "TA Downloader Staring...  %d  s", i);
            plcd1602->write(plcd1602, -1, abyTmp, 0xff);
            rt_thread_delay(1000);
        }
        rt_mutex_release(&Mutex1);
    }
}

void rt_init_thread_entry(void* parameter)
{    
    rt_hw_SpiFlash_init();
    
    rt_hw_lcd1602_init();
    
    rt_mutex_init(&SysInit, "Init", 0);
    
    /* Filesystem Initialization */
#ifdef RT_USING_DFS
    {
        if(sst25vfxx_init("flash0", "spi10") != RT_EOK)
        {
                rt_kprintf("[error] No such spi flash!\r\n");
        }
        /* init the device filesystem */
        dfs_init();

#ifdef RT_USING_DFS_ELMFAT
        /* init the elm chan FatFs filesystam*/
        elm_init();

        /* mount sd card fat partition 1 as root directory */
#if 0
        if (dfs_mount("sd0", "/", "elm", 0, 0) == 0)
#else
        if (dfs_mount("flash0", "/", "elm", 0, 0) == 0)
#endif
        {
            rt_kprintf("File System initialized!\n");
        }
        else
            rt_kprintf("File System initialzation failed!\n");
#endif
    }
#endif

}

void rt_485_thread_entry(void *parameter)
{
    uint8_t byRxData[30];
    rt_device_t pCom485;
    pCom485 = rt_device_find("485");

    if (RT_NULL != pCom485)
    {
         rt_size_t RxNum;
         while (1)
         {
             memset (byRxData, 0, 30);
             pCom485->write(pCom485, 0, "This Is 485 Ask, Pls Reply!\n", 27);
             rt_thread_delay(1000);
             RxNum = pCom485->read(pCom485, 0, byRxData, 28);
             if (RxNum)
            {
                uint8_t byTmpRply[30];
                uint8_t *pTmp;
                pTmp = byTmpRply;
                memset(byTmpRply, 0, 30);
                sprintf(byTmpRply, "Get %d Bytes!", RxNum);
                do
                {
                    pCom485->write(pCom485, 0, pTmp, 1);
                    pTmp++;
                 }while (0 != *pTmp);
            }
             else
             {
                    pCom485->write(pCom485, 0, "No Data Rx!", 11);
             }
             rt_thread_delay(5000);
         }
    }
}

int rt_application_init()
{
    rt_thread_t init_thread;

    rt_thread_t rs485_thread;

    rt_thread_t Process_thread;
    
    rt_err_t result;

    
#if (RT_THREAD_PRIORITY_MAX == 32)
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 8, 20);
#else
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 80, 20);
#endif
    if (init_thread != RT_NULL)
        rt_thread_startup(init_thread);
#if 0
    rs485_thread = rt_thread_create("rs485",
                                   rt_485_thread_entry, RT_NULL,
                                   2048, 8, 20);
#if 0
    if (rs485_thread != RT_NULL)
        rt_thread_startup(rs485_thread);
#endif
#endif

    
    Process_thread = rt_thread_create("TaDown",
                                   rt_TaDownLoader_thread_entry, RT_NULL,
                                   8192, 1, 20);
    
    if (Process_thread != RT_NULL)
        rt_thread_startup(Process_thread);
    
    return 0;
}

/*@}*/
