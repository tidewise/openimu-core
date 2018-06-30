#include "osapi.h"

void OSDisableHook()
{
    portENTER_CRITICAL();
}

void OSEnableHook()
{
	portEXIT_CRITICAL();
}

void OS_Delay(uint32_t msec)
{
    osDelay (msec);
}