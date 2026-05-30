/**
  ******************************************************************************
  * @file    App/sensor_task.c
  * @brief   HTS221 + LSM6DSL acquisition task — Milestone 2a (serial only).
  *
  *          - HTS221  at 8-bit I²C address 0xBE, polled at 1 Hz
  *          - LSM6DSL at 8-bit I²C address 0xD4, polled at 4 Hz
  *          - both reach the bus via BSP_I2C2_* (b_l475e_iot01a1_bus.c),
  *            wired through HTS221_IO_t / LSM6DSL_IO_t
  *          - readings are printed over USART1 VCP via PRINTF (BLE1_DEBUG=1)
  *
  *          BLE wiring is deferred to M2b (see docs §14).
  ******************************************************************************
  */

#include "sensor_task.h"

#include <math.h>

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

#include "b_l475e_iot01a1_bus.h"
#include "bluenrg_conf.h"           /* PRINTF */
#include "hts221.h"
#include "lsm6dsl.h"

/* I²C 8-bit addresses on B-L475E-IOT01A (write form; HAL toggles R/W bit). */
#define HTS221_I2C_ADDR_8BIT    0xBEU
#define LSM6DSL_I2C_ADDR_8BIT   0xD4U

/* Expected WHO_AM_I values from datasheets. */
#define HTS221_WHO_AM_I_VAL     0xBCU
#define LSM6DSL_WHO_AM_I_VAL    0x6AU

/* SensorTask tick: 250 ms → 4 Hz IMU; HTS221 sampled every 4 ticks. */
#define SENSOR_TICK_MS          250U

/* Sensor handles ------------------------------------------------------------*/
static HTS221_Object_t   hts221_obj;
static LSM6DSL_Object_t  lsm6dsl_obj;

/* Task plumbing -------------------------------------------------------------*/
static osThreadId_t sensorTaskHandle;
static const osThreadAttr_t sensorTask_attributes = {
    .name       = "SensorTask",
    .stack_size = 1024,                       /* printf-with-float + sqrtf */
    .priority   = (osPriority_t)osPriorityNormal,
};

/* Forward decls */
static void StartSensorTask(void *argument);
static int  hts221_handle_init(void);
static int  lsm6dsl_handle_init(void);

/* Public --------------------------------------------------------------------*/

void SensorTask_Create(void)
{
    sensorTaskHandle = osThreadNew(StartSensorTask, NULL, &sensorTask_attributes);
}

/* Internal ------------------------------------------------------------------*/

static int hts221_handle_init(void)
{
    HTS221_IO_t io = {
        .Init     = BSP_I2C2_Init,
        .DeInit   = BSP_I2C2_DeInit,
        .BusType  = HTS221_I2C_BUS,
        .Address  = HTS221_I2C_ADDR_8BIT,
        .WriteReg = BSP_I2C2_WriteReg,
        .ReadReg  = BSP_I2C2_ReadReg,
        .GetTick  = BSP_GetTick,
        .Delay    = HAL_Delay,
    };

    if (HTS221_RegisterBusIO(&hts221_obj, &io) != HTS221_OK) {
        PRINTF("HTS221 RegisterBusIO failed\n");
        return -1;
    }

    uint8_t id = 0;
    if (HTS221_ReadID(&hts221_obj, &id) != HTS221_OK) {
        PRINTF("HTS221 ReadID failed\n");
        return -1;
    }
    PRINTF("HTS221 WHO_AM_I = 0x%02X (expected 0x%02X)\n", id, HTS221_WHO_AM_I_VAL);
    if (id != HTS221_WHO_AM_I_VAL) return -1;

    if (HTS221_Init(&hts221_obj)        != HTS221_OK) return -1;
    if (HTS221_TEMP_Enable(&hts221_obj) != HTS221_OK) return -1;
    if (HTS221_HUM_Enable(&hts221_obj)  != HTS221_OK) return -1;
    return 0;
}

static int lsm6dsl_handle_init(void)
{
    LSM6DSL_IO_t io = {
        .Init     = BSP_I2C2_Init,
        .DeInit   = BSP_I2C2_DeInit,
        .BusType  = LSM6DSL_I2C_BUS,
        .Address  = LSM6DSL_I2C_ADDR_8BIT,
        .WriteReg = BSP_I2C2_WriteReg,
        .ReadReg  = BSP_I2C2_ReadReg,
        .GetTick  = BSP_GetTick,
        .Delay    = HAL_Delay,
    };

    if (LSM6DSL_RegisterBusIO(&lsm6dsl_obj, &io) != LSM6DSL_OK) {
        PRINTF("LSM6DSL RegisterBusIO failed\n");
        return -1;
    }

    uint8_t id = 0;
    if (LSM6DSL_ReadID(&lsm6dsl_obj, &id) != LSM6DSL_OK) {
        PRINTF("LSM6DSL ReadID failed\n");
        return -1;
    }
    PRINTF("LSM6DSL WHO_AM_I = 0x%02X (expected 0x%02X)\n", id, LSM6DSL_WHO_AM_I_VAL);
    if (id != LSM6DSL_WHO_AM_I_VAL) return -1;

    if (LSM6DSL_Init(&lsm6dsl_obj)         != LSM6DSL_OK) return -1;
    if (LSM6DSL_ACC_Enable(&lsm6dsl_obj)   != LSM6DSL_OK) return -1;
    if (LSM6DSL_GYRO_Enable(&lsm6dsl_obj)  != LSM6DSL_OK) return -1;
    return 0;
}

static void StartSensorTask(void *argument)
{
    (void)argument;

    /* Stagger after BLE init so first VCP lines from BleTask come first. */
    osDelay(500);

    int hts_ok = (hts221_handle_init()  == 0);
    int lsm_ok = (lsm6dsl_handle_init() == 0);

    if (!hts_ok && !lsm_ok) {
        PRINTF("SensorTask: both sensors failed to init; task idle.\n");
        for (;;) osDelay(1000);
    }
    PRINTF("SensorTask started (HTS221=%s, LSM6DSL=%s).\n",
           hts_ok ? "OK" : "FAIL",
           lsm_ok ? "OK" : "FAIL");

    uint32_t tick = 0;
    for (;;) {
        if (lsm_ok) {
            LSM6DSL_Axes_t a = {0}, g = {0};
            int ar = LSM6DSL_ACC_GetAxes(&lsm6dsl_obj,  &a);
            int gr = LSM6DSL_GYRO_GetAxes(&lsm6dsl_obj, &g);
            if (ar == LSM6DSL_OK && gr == LSM6DSL_OK) {
                /* mg / mdps → g / dps */
                float ax = a.x / 1000.0f, ay = a.y / 1000.0f, az = a.z / 1000.0f;
                float gx = g.x / 1000.0f, gy = g.y / 1000.0f, gz = g.z / 1000.0f;
                float a_mag = sqrtf(ax*ax + ay*ay + az*az);
                float g_mag = sqrtf(gx*gx + gy*gy + gz*gz);
                PRINTF("[imu] a=(%.2f,%.2f,%.2f)g |a|=%.2f  g=(%.0f,%.0f,%.0f)dps |g|=%.1f\n",
                       ax, ay, az, a_mag, gx, gy, gz, g_mag);
            } else {
                PRINTF("[imu] read failed (acc=%d gyro=%d)\n", ar, gr);
            }
        }

        /* HTS221 once per second. */
        if (hts_ok && (tick & 0x03U) == 0U) {
            float t = 0.0f, h = 0.0f;
            int tr = HTS221_TEMP_GetTemperature(&hts221_obj, &t);
            int hr = HTS221_HUM_GetHumidity(&hts221_obj,     &h);
            if (tr == HTS221_OK && hr == HTS221_OK) {
                PRINTF("[env] T=%.1fC H=%.1f%%\n", t, h);
            } else {
                PRINTF("[env] read failed (temp=%d hum=%d)\n", tr, hr);
            }
        }

        tick++;
        osDelay(SENSOR_TICK_MS);
    }
}
