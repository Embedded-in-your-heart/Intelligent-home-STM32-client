/**
  ******************************************************************************
  * @file    App/notify_queue.c
  * @brief   Producer/consumer queue feeding aci_gatt_update_char_value().
  ******************************************************************************
  */

#include "notify_queue.h"

#include <string.h>

#include "cmsis_os.h"
#include "bluenrg_conf.h"   /* PRINTF */

/* On-wire payloads are at most 4 bytes (float32 / uint32). */
typedef struct {
    uint8_t id;
    uint8_t len;
    uint8_t payload[4];
} notify_msg_t;

#define QUEUE_LEN 16U

static osMessageQueueId_t queue;

/* Lifecycle ----------------------------------------------------------------*/

void NotifyQueue_Init(void)
{
    queue = osMessageQueueNew(QUEUE_LEN, sizeof(notify_msg_t), NULL);
    if (queue == NULL) {
        PRINTF("NotifyQueue alloc failed\n");
    }
}

/* Producers — never block. If the queue is full the sample is dropped; this
 * is preferable to backpressuring the sensor task, which would skew timing.
 */

void NotifyQueue_PushFloat(HomeCharId id, float v)
{
    if (queue == NULL) return;
    notify_msg_t m = { .id = (uint8_t)id, .len = 4U };
    memcpy(m.payload, &v, 4);
    (void)osMessageQueuePut(queue, &m, 0U, 0U);
}

void NotifyQueue_PushU8(HomeCharId id, uint8_t v)
{
    if (queue == NULL) return;
    notify_msg_t m = { .id = (uint8_t)id, .len = 1U };
    m.payload[0] = v;
    (void)osMessageQueuePut(queue, &m, 0U, 0U);
}

void NotifyQueue_PushU16(HomeCharId id, uint16_t v)
{
    if (queue == NULL) return;
    notify_msg_t m = { .id = (uint8_t)id, .len = 2U };
    m.payload[0] = (uint8_t)(v & 0xFFU);
    m.payload[1] = (uint8_t)((v >> 8) & 0xFFU);
    (void)osMessageQueuePut(queue, &m, 0U, 0U);
}

/* Consumer ----------------------------------------------------------------*/

void NotifyQueue_Pump(void)
{
    if (queue == NULL) return;

    notify_msg_t m;
    while (osMessageQueueGet(queue, &m, NULL, 0U) == osOK) {
        switch ((HomeCharId)m.id) {

        case HOME_CHAR_TEMPERATURE: {
            float v; memcpy(&v, m.payload, 4);
            Home_Temperature_Update(v);
            break;
        }
        case HOME_CHAR_HUMIDITY: {
            float v; memcpy(&v, m.payload, 4);
            Home_Humidity_Update(v);
            break;
        }
        case HOME_CHAR_ACCEL_MAG: {
            float v; memcpy(&v, m.payload, 4);
            Home_AccelMag_Update(v);
            break;
        }
        case HOME_CHAR_GYRO_MAG: {
            float v; memcpy(&v, m.payload, 4);
            Home_GyroMag_Update(v);
            break;
        }
        case HOME_CHAR_MOTION_ALERT:
            Home_MotionAlert_Update(m.payload[0]);
            break;

        case HOME_CHAR_MIC_LEVEL: {
            uint16_t v = (uint16_t)m.payload[0] | ((uint16_t)m.payload[1] << 8);
            Home_MicLevel_Update(v);
            break;
        }
        case HOME_CHAR_LOUD_ALERT:
            Home_LoudAlert_Update(m.payload[0]);
            break;

        case HOME_CHAR_LED1_STATE:
            Home_Led1State_Update(m.payload[0]);
            break;
        case HOME_CHAR_CONTROL_FLAG:
            Home_ControlFlag_Update(m.payload[0]);
            break;

        case HOME_CHAR_SOUND_CLASS:
            Home_SoundClass_Update(m.payload[0]);
            break;

        case HOME_CHAR_ALARM_DETECTED:
            Home_AlarmDetected_Update(m.payload[0]);
            break;

        case HOME_CHAR_MIC_DBA: {
            float v; memcpy(&v, m.payload, 4);
            Home_MicDBA_Update(v);
            break;
        }

        case HOME_CHAR_VIBRATION_RMS: {
            float v; memcpy(&v, m.payload, 4);
            Home_VibrationRMS_Update(v);
            break;
        }

        case HOME_CHAR_VIBRATION_ALERT:
            Home_VibrationAlert_Update(m.payload[0]);
            break;

        case HOME_CHAR_QUAKE_ALERT:
            Home_QuakeAlert_Update(m.payload[0]);
            break;

        default:
            /* Unknown id — drop silently. */
            break;
        }
    }
}
