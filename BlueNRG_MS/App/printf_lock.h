/**
 * @file    printf_lock.h
 * @brief   Thread-safe printf wrapper using RTOS mutex.
 */

#ifndef PRINTF_LOCK_H
#define PRINTF_LOCK_H

#include <stdio.h>

void PrintfLock_Init(void);
int  PrintfLock_Printf(const char *format, ...);

#endif
