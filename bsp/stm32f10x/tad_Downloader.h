
#ifndef __TAD_DOWNLOADER_H
#define __TAD_DOWNLOADER_H

#include <rtthread.h>

#define BPS_SETTING_OS	0x0002
#define FILE_SEL_OS		0x0016
#define AutoUpdate_SET_OS  0x0050
void rt_TaDownLoader_thread_entry(void* parameter);

extern rt_device_t pLcd1602;
#endif
