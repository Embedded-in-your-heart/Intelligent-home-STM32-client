/**
  ******************************************************************************
  * @file    App/gatt_db.c
  * @brief   GATT table builder + characteristic update helpers for the
  *          Intelligent-home STM32 client.
  *
  *          UUID assignments are FIXED — once burned they must not change
  *          without updating the RPi-side contract (docs §12).
  ******************************************************************************
  */

#include <stdint.h>
#include <string.h>
#include "bluenrg_def.h"
#include "bluenrg_conf.h"
#include "bluenrg_gatt_aci.h"
#include "main.h"                   /* LED1_PIN_Pin / LED1_PIN_GPIO_Port */
#include "gatt_db.h"

/* UUID helper ---------------------------------------------------------------*/
#define COPY_UUID_128(uuid_struct, b15,b14,b13,b12,b11,b10,b9,b8,b7,b6,b5,b4,b3,b2,b1,b0) \
do { \
    (uuid_struct)[ 0] = (b0);  (uuid_struct)[ 1] = (b1);  \
    (uuid_struct)[ 2] = (b2);  (uuid_struct)[ 3] = (b3);  \
    (uuid_struct)[ 4] = (b4);  (uuid_struct)[ 5] = (b5);  \
    (uuid_struct)[ 6] = (b6);  (uuid_struct)[ 7] = (b7);  \
    (uuid_struct)[ 8] = (b8);  (uuid_struct)[ 9] = (b9);  \
    (uuid_struct)[10] = (b10); (uuid_struct)[11] = (b11); \
    (uuid_struct)[12] = (b12); (uuid_struct)[13] = (b13); \
    (uuid_struct)[14] = (b14); (uuid_struct)[15] = (b15); \
} while (0)

/* Common base: xxxxxxxx-8E22-4541-9D4C-21EDAE82ED19 -------------------------*/

/* Home Sensor Service ---- 1A220001-... */
#define COPY_HOME_SENSOR_SERVICE_UUID(u)  COPY_UUID_128(u, 0x1A,0x22,0x00,0x01, 0x8E,0x22, 0x45,0x41, 0x9D,0x4C, 0x21,0xED,0xAE,0x82,0xED,0x19)
#define COPY_TEMPERATURE_CHAR_UUID(u)     COPY_UUID_128(u, 0x1A,0x22,0x00,0x02, 0x8E,0x22, 0x45,0x41, 0x9D,0x4C, 0x21,0xED,0xAE,0x82,0xED,0x19)
#define COPY_HUMIDITY_CHAR_UUID(u)        COPY_UUID_128(u, 0x1A,0x22,0x00,0x03, 0x8E,0x22, 0x45,0x41, 0x9D,0x4C, 0x21,0xED,0xAE,0x82,0xED,0x19)
#define COPY_ACCEL_MAG_CHAR_UUID(u)       COPY_UUID_128(u, 0x1A,0x22,0x00,0x04, 0x8E,0x22, 0x45,0x41, 0x9D,0x4C, 0x21,0xED,0xAE,0x82,0xED,0x19)
#define COPY_GYRO_MAG_CHAR_UUID(u)        COPY_UUID_128(u, 0x1A,0x22,0x00,0x05, 0x8E,0x22, 0x45,0x41, 0x9D,0x4C, 0x21,0xED,0xAE,0x82,0xED,0x19)
#define COPY_MOTION_ALERT_CHAR_UUID(u)    COPY_UUID_128(u, 0x1A,0x22,0x00,0x06, 0x8E,0x22, 0x45,0x41, 0x9D,0x4C, 0x21,0xED,0xAE,0x82,0xED,0x19)
#define COPY_MIC_LEVEL_CHAR_UUID(u)       COPY_UUID_128(u, 0x1A,0x22,0x00,0x07, 0x8E,0x22, 0x45,0x41, 0x9D,0x4C, 0x21,0xED,0xAE,0x82,0xED,0x19)
#define COPY_LOUD_ALERT_CHAR_UUID(u)      COPY_UUID_128(u, 0x1A,0x22,0x00,0x08, 0x8E,0x22, 0x45,0x41, 0x9D,0x4C, 0x21,0xED,0xAE,0x82,0xED,0x19)
#define COPY_SOUND_CLASS_CHAR_UUID(u)     COPY_UUID_128(u, 0x1A,0x22,0x00,0x09, 0x8E,0x22, 0x45,0x41, 0x9D,0x4C, 0x21,0xED,0xAE,0x82,0xED,0x19)
#define COPY_ALARM_DETECTED_CHAR_UUID(u)  COPY_UUID_128(u, 0x1A,0x22,0x00,0x0A, 0x8E,0x22, 0x45,0x41, 0x9D,0x4C, 0x21,0xED,0xAE,0x82,0xED,0x19)
#define COPY_MIC_DBA_CHAR_UUID(u)         COPY_UUID_128(u, 0x1A,0x22,0x00,0x0B, 0x8E,0x22, 0x45,0x41, 0x9D,0x4C, 0x21,0xED,0xAE,0x82,0xED,0x19)
#define COPY_VIBRATION_RMS_CHAR_UUID(u)   COPY_UUID_128(u, 0x1A,0x22,0x00,0x0C, 0x8E,0x22, 0x45,0x41, 0x9D,0x4C, 0x21,0xED,0xAE,0x82,0xED,0x19)
#define COPY_VIBRATION_ALERT_CHAR_UUID(u) COPY_UUID_128(u, 0x1A,0x22,0x00,0x0D, 0x8E,0x22, 0x45,0x41, 0x9D,0x4C, 0x21,0xED,0xAE,0x82,0xED,0x19)
#define COPY_QUAKE_ALERT_CHAR_UUID(u)     COPY_UUID_128(u, 0x1A,0x22,0x00,0x0E, 0x8E,0x22, 0x45,0x41, 0x9D,0x4C, 0x21,0xED,0xAE,0x82,0xED,0x19)

/* Home Control Service ---- 1A22F001-... */
#define COPY_HOME_CONTROL_SERVICE_UUID(u) COPY_UUID_128(u, 0x1A,0x22,0xF0,0x01, 0x8E,0x22, 0x45,0x41, 0x9D,0x4C, 0x21,0xED,0xAE,0x82,0xED,0x19)
#define COPY_LED1_STATE_CHAR_UUID(u)      COPY_UUID_128(u, 0x1A,0x22,0xF0,0x02, 0x8E,0x22, 0x45,0x41, 0x9D,0x4C, 0x21,0xED,0xAE,0x82,0xED,0x19)
#define COPY_CONTROL_FLAG_CHAR_UUID(u)    COPY_UUID_128(u, 0x1A,0x22,0xF0,0x03, 0x8E,0x22, 0x45,0x41, 0x9D,0x4C, 0x21,0xED,0xAE,0x82,0xED,0x19)

/* Handles ------------------------------------------------------------------*/
static uint16_t home_sensor_service_handle;
static uint16_t temperature_char_handle;
static uint16_t humidity_char_handle;
static uint16_t accel_mag_char_handle;
static uint16_t gyro_mag_char_handle;
static uint16_t motion_alert_char_handle;
static uint16_t mic_level_char_handle;
static uint16_t loud_alert_char_handle;
static uint16_t sound_class_char_handle;
static uint16_t alarm_detected_char_handle;
static uint16_t mic_dba_char_handle;
static uint16_t vibration_rms_char_handle;
static uint16_t vibration_alert_char_handle;
static uint16_t quake_alert_char_handle;

static uint16_t home_control_service_handle;
static uint16_t led1_state_char_handle;
static uint16_t control_flag_char_handle;

/* Externals referenced from sensor.c -----------------------------------------*/
extern __IO uint16_t connection_handle;

/* ControlFlag latest value — reserved for future use; currently only logged. */
static volatile uint8_t g_control_flag;

/* Internal helpers ----------------------------------------------------------*/
static tBleStatus add_char(uint16_t service, const uint8_t uuid[16],
                           uint8_t value_len, uint8_t props, uint8_t evt_mask,
                           uint16_t *out_handle)
{
    Char_UUID_t c;
    memcpy(c.Char_UUID_128, uuid, 16);
    return aci_gatt_add_char(service, UUID_TYPE_128, c.Char_UUID_128,
                             value_len, props,
                             ATTR_PERMISSION_NONE, evt_mask,
                             16, 0, out_handle);
}

static tBleStatus update_float(uint16_t service, uint16_t ch, float v)
{
    /* Cortex-M is little-endian → direct memcpy gives float32_le on the wire. */
    uint8_t buf[4];
    memcpy(buf, &v, 4);
    return aci_gatt_update_char_value(service, ch, 0, 4, buf);
}

static tBleStatus update_u16(uint16_t service, uint16_t ch, uint16_t v)
{
    uint8_t buf[2] = { (uint8_t)v, (uint8_t)(v >> 8) };
    return aci_gatt_update_char_value(service, ch, 0, 2, buf);
}

static tBleStatus update_u8(uint16_t service, uint16_t ch, uint8_t v)
{
    return aci_gatt_update_char_value(service, ch, 0, 1, &v);
}

/* Service builders ----------------------------------------------------------*/

tBleStatus Add_HomeSensor_Service(void)
{
    Service_UUID_t s;
    uint8_t u[16];
    tBleStatus ret;

    COPY_HOME_SENSOR_SERVICE_UUID(u);
    memcpy(s.Service_UUID_128, u, 16);
    /* 1 attribute for service + 3 per char (decl, value, CCC) × 13 chars. */
    ret = aci_gatt_add_serv(UUID_TYPE_128, s.Service_UUID_128, PRIMARY_SERVICE,
                            1 + 3 * 13, &home_sensor_service_handle);
    if (ret != BLE_STATUS_SUCCESS) return ret;

    /* Display chars: Read + Notify, no event mask (we cache values). */
    const uint8_t DISPLAY_PROPS = CHAR_PROP_READ | CHAR_PROP_NOTIFY;
    const uint8_t DISPLAY_EVTS  = GATT_DONT_NOTIFY_EVENTS;

    COPY_TEMPERATURE_CHAR_UUID(u);
    ret = add_char(home_sensor_service_handle, u, 4, DISPLAY_PROPS, DISPLAY_EVTS, &temperature_char_handle);
    if (ret != BLE_STATUS_SUCCESS) return ret;

    COPY_HUMIDITY_CHAR_UUID(u);
    ret = add_char(home_sensor_service_handle, u, 4, DISPLAY_PROPS, DISPLAY_EVTS, &humidity_char_handle);
    if (ret != BLE_STATUS_SUCCESS) return ret;

    COPY_ACCEL_MAG_CHAR_UUID(u);
    ret = add_char(home_sensor_service_handle, u, 4, DISPLAY_PROPS, DISPLAY_EVTS, &accel_mag_char_handle);
    if (ret != BLE_STATUS_SUCCESS) return ret;

    COPY_GYRO_MAG_CHAR_UUID(u);
    ret = add_char(home_sensor_service_handle, u, 4, DISPLAY_PROPS, DISPLAY_EVTS, &gyro_mag_char_handle);
    if (ret != BLE_STATUS_SUCCESS) return ret;

    COPY_MOTION_ALERT_CHAR_UUID(u);
    ret = add_char(home_sensor_service_handle, u, 1, DISPLAY_PROPS, DISPLAY_EVTS, &motion_alert_char_handle);
    if (ret != BLE_STATUS_SUCCESS) return ret;

    COPY_MIC_LEVEL_CHAR_UUID(u);
    ret = add_char(home_sensor_service_handle, u, 2, DISPLAY_PROPS, DISPLAY_EVTS, &mic_level_char_handle);
    if (ret != BLE_STATUS_SUCCESS) return ret;

    COPY_LOUD_ALERT_CHAR_UUID(u);
    ret = add_char(home_sensor_service_handle, u, 1, DISPLAY_PROPS, DISPLAY_EVTS, &loud_alert_char_handle);
    if (ret != BLE_STATUS_SUCCESS) return ret;

    COPY_SOUND_CLASS_CHAR_UUID(u);
    ret = add_char(home_sensor_service_handle, u, 1, DISPLAY_PROPS, DISPLAY_EVTS, &sound_class_char_handle);
    if (ret != BLE_STATUS_SUCCESS) return ret;

    COPY_ALARM_DETECTED_CHAR_UUID(u);
    ret = add_char(home_sensor_service_handle, u, 1, DISPLAY_PROPS, DISPLAY_EVTS, &alarm_detected_char_handle);
    if (ret != BLE_STATUS_SUCCESS) return ret;

    COPY_MIC_DBA_CHAR_UUID(u);
    ret = add_char(home_sensor_service_handle, u, 4, DISPLAY_PROPS, DISPLAY_EVTS, &mic_dba_char_handle);
    if (ret != BLE_STATUS_SUCCESS) return ret;

    COPY_VIBRATION_RMS_CHAR_UUID(u);
    ret = add_char(home_sensor_service_handle, u, 4, DISPLAY_PROPS, DISPLAY_EVTS, &vibration_rms_char_handle);
    if (ret != BLE_STATUS_SUCCESS) return ret;

    COPY_VIBRATION_ALERT_CHAR_UUID(u);
    ret = add_char(home_sensor_service_handle, u, 1, DISPLAY_PROPS, DISPLAY_EVTS, &vibration_alert_char_handle);
    if (ret != BLE_STATUS_SUCCESS) return ret;

    COPY_QUAKE_ALERT_CHAR_UUID(u);
    ret = add_char(home_sensor_service_handle, u, 1, DISPLAY_PROPS, DISPLAY_EVTS, &quake_alert_char_handle);
    if (ret != BLE_STATUS_SUCCESS) return ret;

    return BLE_STATUS_SUCCESS;
}

tBleStatus Add_HomeControl_Service(void)
{
    Service_UUID_t s;
    uint8_t u[16];
    tBleStatus ret;

    COPY_HOME_CONTROL_SERVICE_UUID(u);
    memcpy(s.Service_UUID_128, u, 16);
    ret = aci_gatt_add_serv(UUID_TYPE_128, s.Service_UUID_128, PRIMARY_SERVICE,
                            1 + 3 * 2, &home_control_service_handle);
    if (ret != BLE_STATUS_SUCCESS) return ret;

    /* Controller chars: Read + Write (with and without response). Notify on
     * attribute write so we receive EVT_BLUE_GATT_ATTRIBUTE_MODIFIED.
     */
    const uint8_t CONTROL_PROPS = CHAR_PROP_READ | CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP;
    const uint8_t CONTROL_EVTS  = GATT_NOTIFY_ATTRIBUTE_WRITE;

    COPY_LED1_STATE_CHAR_UUID(u);
    ret = add_char(home_control_service_handle, u, 1, CONTROL_PROPS, CONTROL_EVTS, &led1_state_char_handle);
    if (ret != BLE_STATUS_SUCCESS) return ret;

    COPY_CONTROL_FLAG_CHAR_UUID(u);
    ret = add_char(home_control_service_handle, u, 1, CONTROL_PROPS, CONTROL_EVTS, &control_flag_char_handle);
    if (ret != BLE_STATUS_SUCCESS) return ret;

    return BLE_STATUS_SUCCESS;
}

/* Per-characteristic updates ------------------------------------------------*/

tBleStatus Home_Temperature_Update(float c) { return update_float(home_sensor_service_handle, temperature_char_handle, c); }
tBleStatus Home_Humidity_Update(float p)    { return update_float(home_sensor_service_handle, humidity_char_handle,    p); }
tBleStatus Home_AccelMag_Update(float g)    { return update_float(home_sensor_service_handle, accel_mag_char_handle,   g); }
tBleStatus Home_GyroMag_Update(float d)     { return update_float(home_sensor_service_handle, gyro_mag_char_handle,    d); }
tBleStatus Home_MotionAlert_Update(uint8_t f){ return update_u8 (home_sensor_service_handle, motion_alert_char_handle,f); }
tBleStatus Home_MicLevel_Update(uint16_t l) { return update_u16(home_sensor_service_handle, mic_level_char_handle,    l); }
tBleStatus Home_LoudAlert_Update(uint8_t f) { return update_u8 (home_sensor_service_handle, loud_alert_char_handle,   f); }
tBleStatus Home_Led1State_Update(uint8_t s) { return update_u8 (home_control_service_handle, led1_state_char_handle,  s); }
tBleStatus Home_ControlFlag_Update(uint8_t f){ return update_u8(home_control_service_handle, control_flag_char_handle,f); }
tBleStatus Home_SoundClass_Update(uint8_t c) { return update_u8   (home_sensor_service_handle, sound_class_char_handle,    c); }
tBleStatus Home_AlarmDetected_Update(uint8_t f){ return update_u8 (home_sensor_service_handle, alarm_detected_char_handle, f); }
tBleStatus Home_MicDBA_Update(float d)       { return update_float(home_sensor_service_handle, mic_dba_char_handle,        d); }
tBleStatus Home_VibrationRMS_Update(float r) { return update_float(home_sensor_service_handle, vibration_rms_char_handle,   r); }
tBleStatus Home_VibrationAlert_Update(uint8_t f){ return update_u8(home_sensor_service_handle, vibration_alert_char_handle, f); }
tBleStatus Home_QuakeAlert_Update(uint8_t f) { return update_u8   (home_sensor_service_handle, quake_alert_char_handle,     f); }

/* Event callbacks -----------------------------------------------------------*/

void Read_Request_CB(uint16_t handle)
{
    /* All chars are configured with GATT_DONT_NOTIFY_EVENTS, so this callback
     * should not fire. If it ever does, simply allow the read to complete. */
    (void)handle;
    if (connection_handle != 0) {
        aci_gatt_allow_read(connection_handle);
    }
}

void Attribute_Modified_CB(uint16_t handle, uint8_t length, uint8_t *data)
{
    /* aci_gatt_attribute_modified_event reports the VALUE attribute handle,
     * which is (char_decl_handle + 1) in BlueNRG-MS.
     */
    if (length < 1) return;

    if (handle == led1_state_char_handle + 1) {
        uint8_t v = data[0] ? 1U : 0U;
        HAL_GPIO_WritePin(LED1_PIN_GPIO_Port, LED1_PIN_Pin,
                          v ? GPIO_PIN_SET : GPIO_PIN_RESET);
        Home_Led1State_Update(v);
        PRINTF("Write LED1State = %u\n", v);
    } else if (handle == control_flag_char_handle + 1) {
        g_control_flag = data[0];
        Home_ControlFlag_Update(g_control_flag);
        PRINTF("Write ControlFlag = 0x%02X\n", g_control_flag);
    }
}
