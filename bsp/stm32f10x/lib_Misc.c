
#include "lib_Misc.h"

VOID While_Delay(__IO DWORD wMs)
{
  __IO  DWORD i;

    while(wMs--)
    {
        for (i = 0; i < SYSCLK_ONE_MS; i++);
    }
}

VOID While_DelayUs(__IO DWORD dwUs)
{
    __IO DWORD i;

    while(dwUs--)
    {
        for (i = 0; i < SYSCLK_ONE_US; i++);
    }
}

VOID Delay_1us(DWORD dwUs)
{
    While_DelayUs(dwUs);
}

VOID Delay_10ms(BYTE by10ms)
{
    While_Delay(((WORD)by10ms) * 10);
}

VOID Delay_1ms(BYTE byms)
{
    While_Delay(byms);
}

