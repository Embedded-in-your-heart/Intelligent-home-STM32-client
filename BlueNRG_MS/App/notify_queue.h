/**
  ******************************************************************************
  * @file    App/notify_queue.h
  * @brief   Producer/consumer queue between sensor tasks and BleTask.
  *
  *          - Sensor / audio tasks call NotifyQueue_Push* (non-blocking; drops
  *            on full queue).
  *          - BleTask calls NotifyQueue_Pump() each loop iteration to drain
  *            and forward to Home_*_Update() (the only ACI callers).
  ******************************************************************************
  */

#ifndef NOTIFY_QUEUE_H
#define NOTIFY_QUEUE_H

#include <stdint.h>
#include "gatt_db.h"   /* HomeCharId */

/** Create the underlying queue. Call once from MX_FREERTOS_Init() before
 *  any task that pushes / pumps starts.
 */
void NotifyQueue_Init(void);

/** Drain queued items and dispatch each to its Home_*_Update().
 *  MUST be called from BleTask (or any thread that owns the BlueNRG stack).
 */
void NotifyQueue_Pump(void);

/* Producers ----------------------------------------------------------------*/
void NotifyQueue_PushFloat(HomeCharId id, float value);
void NotifyQueue_PushU8(HomeCharId id, uint8_t value);
void NotifyQueue_PushU16(HomeCharId id, uint16_t value);

#endif /* NOTIFY_QUEUE_H */
