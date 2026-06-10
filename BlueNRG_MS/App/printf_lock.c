/**
 * @file    printf_lock.c
 * @brief   Thread-safe printf wrapper using RTOS mutex.
 */

#include "printf_lock.h"

#include <stdarg.h>
#include <stdio.h>

#include "cmsis_os.h"

static osMutexId_t printf_mutex;

void PrintfLock_Init(void)
{
    printf_mutex = osMutexNew(NULL);
}

int PrintfLock_Printf(const char *format, ...)
{
    va_list args;
    int ret;

    if (!printf_mutex) {
        va_start(args, format);
        ret = vprintf(format, args);
        va_end(args);
        return ret;
    }

    osMutexAcquire(printf_mutex, osWaitForever);
    va_start(args, format);
    ret = vprintf(format, args);
    va_end(args);
    osMutexRelease(printf_mutex);

    return ret;
}
