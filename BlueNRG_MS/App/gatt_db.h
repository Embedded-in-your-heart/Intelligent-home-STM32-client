/**
  ******************************************************************************
  * @file    App/gatt_db.h
  * @brief   Header file for App/gatt_db.c
  *
  *          GATT table for the Intelligent-home STM32 client.
  *          See docs/STM32 Client 開發文件.md §5.4 for the contract.
  ******************************************************************************
  */

#ifndef GATT_DB_H
#define GATT_DB_H

#include <stdint.h>
#include "bluenrg_def.h"

/* BlueNRG UUID unions (kept for ACI API compatibility) ----------------------*/
typedef union Service_UUID_t_s {
  uint16_t Service_UUID_16;
  uint8_t  Service_UUID_128[16];
} Service_UUID_t;

typedef union Char_UUID_t_s {
  uint16_t Char_UUID_16;
  uint8_t  Char_UUID_128[16];
} Char_UUID_t;

/* Characteristic identifier for the NotifyQueue dispatcher ------------------*/
typedef enum {
  HOME_CHAR_TEMPERATURE = 0,
  HOME_CHAR_HUMIDITY,
  HOME_CHAR_ACCEL_MAG,
  HOME_CHAR_GYRO_MAG,
  HOME_CHAR_MOTION_ALERT,
  HOME_CHAR_MIC_LEVEL,
  HOME_CHAR_LOUD_ALERT,
  HOME_CHAR_LED1_STATE,
  HOME_CHAR_CONTROL_FLAG,
  HOME_CHAR_SOUND_CLASS,
  HOME_CHAR_ALARM_DETECTED,
  HOME_CHAR_MIC_DBA,
  HOME_CHAR_VIBRATION_RMS,
  HOME_CHAR_VIBRATION_ALERT,
  HOME_CHAR_QUAKE_ALERT,
} HomeCharId;

/* Service registration ------------------------------------------------------*/
tBleStatus Add_HomeSensor_Service(void);
tBleStatus Add_HomeControl_Service(void);

/* Characteristic value updates ----------------------------------------------
 * Encode argument into the wire format from docs §5.4 and call
 * aci_gatt_update_char_value(). Returns the BlueNRG status.
 */
tBleStatus Home_Temperature_Update(float celsius);
tBleStatus Home_Humidity_Update(float percent);
tBleStatus Home_AccelMag_Update(float g);
tBleStatus Home_GyroMag_Update(float dps);
tBleStatus Home_MotionAlert_Update(uint8_t flag);
tBleStatus Home_MicLevel_Update(uint16_t level);
tBleStatus Home_LoudAlert_Update(uint8_t flag);
tBleStatus Home_Led1State_Update(uint8_t state);
tBleStatus Home_ControlFlag_Update(uint8_t flag);
tBleStatus Home_SoundClass_Update(uint8_t cls);
tBleStatus Home_AlarmDetected_Update(uint8_t flag);
tBleStatus Home_MicDBA_Update(float dba);
tBleStatus Home_VibrationRMS_Update(float rms_mg);
tBleStatus Home_VibrationAlert_Update(uint8_t flag);
tBleStatus Home_QuakeAlert_Update(uint8_t flag);

/* GATT event callbacks (called from sensor.c::user_notify) ------------------*/
void Read_Request_CB(uint16_t handle);
void Attribute_Modified_CB(uint16_t handle, uint8_t length, uint8_t *data);

#endif /* GATT_DB_H */
