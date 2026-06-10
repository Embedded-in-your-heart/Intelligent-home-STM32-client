/**
  ******************************************************************************
  * @file    App/sensor_task.c
  * @brief   HTS221 + LSM6DSL acquisition task.
  *
  *          - HTS221  at 8-bit I²C address 0xBE, polled at 1 Hz
  *          - LSM6DSL accelerometer at 104 Hz via FIFO (continuous mode);
  *            gyroscope polled at 4 Hz (not batched in FIFO).
  *          - FIFO drained every 250 ms; ~26 accel samples expected per drain.
  *          - Each drained accel sample is fed to ImuDsp; the last sample of
  *            each drain is used as the AccelMagnitude / MotionAlert source so
  *            existing BLE behaviour is structurally identical.
  *          - ImuDsp produces per-window VibrationRMS, VibrationAlert, and
  *            QuakeAlert characteristics.
  *
  *          BLE wiring: notify_queue.c / gatt_db.c (M2b+).
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
#include "lsm6dsl_reg.h"
#include "notify_queue.h"
#include "imu_dsp.h"

/* I²C 8-bit addresses on B-L475E-IOT01A (write form; HAL toggles R/W bit). */
#define HTS221_I2C_ADDR_8BIT    0xBEU
#define LSM6DSL_I2C_ADDR_8BIT   0xD4U

/* Expected WHO_AM_I values from datasheets. */
#define HTS221_WHO_AM_I_VAL     0xBCU
#define LSM6DSL_WHO_AM_I_VAL    0x6AU

/* SensorTask tick: 250 ms → 4 Hz poll rate; ~26 accel samples/drain at 104 Hz. */
#define SENSOR_TICK_MS          250U

/* Maximum FIFO samples readable in one drain.
 * LSM6DSL FIFO depth = 4096 bytes = 2048 16-bit words = ~682 XYZ triplets.
 * We cap the drain at 3× the expected count to avoid holding the task too long. */
#define FIFO_DRAIN_MAX          78U   /* 3 × 26 samples at 104 Hz / 4 Hz poll */

/* Removed FIFO_OVERRUN_LEVEL (was 2048U, unreachable: DIFF_FIFO is 11 bits, max 2047).
 * Overrun is now detected via the hardware STATUS2.over_run bit instead. */

/* MotionAlert thresholds + debounce (docs §5.5). */
#define MOTION_ACCEL_THR_G      1.8f
#define MOTION_GYRO_THR_DPS     250.0f
#define MOTION_HOLD_MS          100U     /* condition must persist this long */
#define MOTION_LOCKOUT_MS       1000U    /* min interval between state transitions */

/* VibrationAlert thresholds (calibration pending — estimated from lab bench). */
#define VIB_ON_THR_MG           30.0f   /* consecutive-high threshold in mg */
#define VIB_OFF_THR_MG          15.0f   /* consecutive-low  threshold in mg */
#define VIB_CONSEC_ON           5U      /* windows above VIB_ON  → assert 1 */
#define VIB_CONSEC_OFF          10U     /* windows below VIB_OFF → assert 0 */

/* QuakeAlert thresholds (calibration pending). */
#define QUAKE_THR_MG            20.0f   /* 1-10 Hz band RMS threshold in mg */
#define QUAKE_CONSEC_ON         3U      /* consecutive above → assert 1 */
#define QUAKE_CONSEC_OFF        5U      /* consecutive below → assert 0 */
#define QUAKE_LOCKOUT_MS        2000U   /* min ms between QuakeAlert transitions */

/* Sensor handles ------------------------------------------------------------*/
static HTS221_Object_t   hts221_obj;
static LSM6DSL_Object_t  lsm6dsl_obj;

/* Task plumbing -------------------------------------------------------------*/
static osThreadId_t sensorTaskHandle;
static const osThreadAttr_t sensorTask_attributes = {
    .name       = "SensorTask",
    .stack_size = 2048,            /* FIFO burst drain + DSP floating-point + biquad VFP frames */
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

    /* Basic init: sets default ODR/FS values via the component driver. */
    if (LSM6DSL_Init(&lsm6dsl_obj)        != LSM6DSL_OK) return -1;

    /* --- Accelerometer: 104 Hz ODR, ±2 g full-scale ---
     * Register CTRL1_XL: ODR_XL[3:0] = 0100b (104 Hz), FS_XL[1:0] = 00b (±2 g).
     * LSM6DSL_ACC_SetOutputDataRate(104.0f) writes ODR_XL = 4 (LSM6DSL_XL_ODR_104Hz).
     * LSM6DSL_ACC_SetFullScale(2) writes FS_XL = 0 (LSM6DSL_2g).
     * Both use the high-level API; no direct register access needed. */
    if (LSM6DSL_ACC_Enable(&lsm6dsl_obj)              != LSM6DSL_OK) return -1;
    if (LSM6DSL_ACC_SetOutputDataRate(&lsm6dsl_obj, 104.0f) != LSM6DSL_OK) return -1;
    if (LSM6DSL_ACC_SetFullScale(&lsm6dsl_obj, 2)     != LSM6DSL_OK) return -1;

    /* --- Gyroscope: enable at default ODR (gyro NOT batched into FIFO). --- */
    if (LSM6DSL_GYRO_Enable(&lsm6dsl_obj) != LSM6DSL_OK) return -1;

    /* --- FIFO configuration ---
     * Step 1: Put FIFO in BYPASS mode before reconfiguring decimation/ODR
     *         (required by the LSM6DSL datasheet, section 7.7).
     *         LSM6DSL_BYPASS_MODE = 0. */
    if (LSM6DSL_FIFO_Set_Mode(&lsm6dsl_obj, (uint8_t)LSM6DSL_BYPASS_MODE) != LSM6DSL_OK) {
        PRINTF("LSM6DSL FIFO bypass failed\n");
        return -1;
    }

    /* Step 2: Batch accelerometer with no decimation (every sample stored).
     *         Register FIFO_CTRL3: DEC_FIFO_XL[2:0] = 001b (no decimation).
     *         High-level: LSM6DSL_FIFO_ACC_Set_Decimation(LSM6DSL_FIFO_XL_NO_DEC = 1). */
    if (LSM6DSL_FIFO_ACC_Set_Decimation(&lsm6dsl_obj,
                                         (uint8_t)LSM6DSL_FIFO_XL_NO_DEC) != LSM6DSL_OK) {
        PRINTF("LSM6DSL FIFO accel decimation failed\n");
        return -1;
    }

    /* Step 3: Do NOT batch gyroscope into FIFO (gyro polled separately at 4 Hz).
     *         Register FIFO_CTRL3: DEC_FIFO_GYRO[2:0] = 000b (disabled).
     *         High-level: LSM6DSL_FIFO_GYRO_Set_Decimation(LSM6DSL_FIFO_GY_DISABLE = 0). */
    if (LSM6DSL_FIFO_GYRO_Set_Decimation(&lsm6dsl_obj,
                                          (uint8_t)LSM6DSL_FIFO_GY_DISABLE) != LSM6DSL_OK) {
        PRINTF("LSM6DSL FIFO gyro decimation failed\n");
        return -1;
    }

    /* Step 4: Set FIFO output data rate to 104 Hz.
     *         Register FIFO_CTRL5: ODR_FIFO[3:0] = 0100b (104 Hz).
     *         High-level: LSM6DSL_FIFO_Set_ODR_Value(104.0f). */
    if (LSM6DSL_FIFO_Set_ODR_Value(&lsm6dsl_obj, 104.0f) != LSM6DSL_OK) {
        PRINTF("LSM6DSL FIFO ODR failed\n");
        return -1;
    }

    /* Step 5: Enable continuous (stream) mode — oldest data overwritten on overflow.
     *         Register FIFO_CTRL5: FIFO_MODE[2:0] = 110b.
     *         LSM6DSL_STREAM_MODE = 6. */
    if (LSM6DSL_FIFO_Set_Mode(&lsm6dsl_obj, (uint8_t)LSM6DSL_STREAM_MODE) != LSM6DSL_OK) {
        PRINTF("LSM6DSL FIFO stream mode failed\n");
        return -1;
    }

    /* Initialise DSP module (filter state, accumulators, warmup counter). */
    ImuDsp_Init();

    return 0;
}

/**
 * @brief  Drain one X/Y/Z accel triplet from the FIFO using the high-level API.
 *
 *         The high-level LSM6DSL_FIFO_ACC_Get_Axis() reads one 16-bit word and
 *         applies sensor sensitivity, returning mg.  Three consecutive calls
 *         consume the X, Y, Z words of one sample slot.
 *
 * @param  ax_mg  output X acceleration in mg
 * @param  ay_mg  output Y acceleration in mg
 * @param  az_mg  output Z acceleration in mg
 * @return 0 on success, -1 on read error
 */
static int fifo_read_xyz_mg(float *ax_mg, float *ay_mg, float *az_mg)
{
    int32_t xraw = 0, yraw = 0, zraw = 0;

    /* LSM6DSL_FIFO_ACC_Get_Axis reads one 16-bit word from FIFO_DATA_OUT and
     * converts via sensitivity (mg/LSB).  Three calls = one XYZ triplet.
     *
     * On a partial read failure the FIFO pointer has already advanced past the
     * successfully consumed words.  Discard the remaining word(s) of the same
     * triplet so that the pointer stays aligned on word-0 (X) of the next
     * triplet.  A discarded read failure is intentionally ignored here because
     * a BYPASS→STREAM reset in the caller would also realign the FIFO. */
    if (LSM6DSL_FIFO_ACC_Get_Axis(&lsm6dsl_obj, &xraw) != LSM6DSL_OK) {
        /* X failed: Y and Z words not yet consumed — nothing to flush. */
        return -1;
    }
    if (LSM6DSL_FIFO_ACC_Get_Axis(&lsm6dsl_obj, &yraw) != LSM6DSL_OK) {
        /* X consumed, Y failed: discard the Z word to realign. */
        (void)LSM6DSL_FIFO_ACC_Get_Axis(&lsm6dsl_obj, &zraw);
        return -1;
    }
    if (LSM6DSL_FIFO_ACC_Get_Axis(&lsm6dsl_obj, &zraw) != LSM6DSL_OK) {
        /* X and Y consumed, Z failed: pointer already at next triplet. */
        return -1;
    }

    /* LSM6DSL_FIFO_ACC_Get_Axis returns mg as int32_t (cast to float). */
    *ax_mg = (float)xraw;
    *ay_mg = (float)yraw;
    *az_mg = (float)zraw;
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

    /* MotionAlert state machine (statics — preserved across iterations). */
    static uint32_t motion_high_since    = 0U;
    static uint32_t motion_lockout_until = 0U;
    static uint8_t  motion_state         = 0U;

    /* VibrationAlert state machine. */
    static uint32_t vib_consec_on        = 0U;
    static uint32_t vib_consec_off       = 0U;
    static uint8_t  vib_alert_state      = 0U;

    /* QuakeAlert state machine. */
    static uint32_t quake_consec_on      = 0U;
    static uint32_t quake_consec_off     = 0U;
    static uint32_t quake_lockout_until  = 0U;
    static uint8_t  quake_alert_state    = 0U;

    /* Flag set on first FIFO-empty tick to log the fallback message once. */
    static int      fifo_fallback_logged = 0;

    uint32_t tick = 0;
    for (;;) {
        /* IMU @ 4 Hz */
        if (lsm_ok) {
            /* ------------------------------------------------------------------
             * FIFO drain: read all complete XYZ triplets accumulated since the
             * last tick (~26 at 104 Hz / 4 Hz poll rate).
             *
             * The last drained sample is also used for AccelMagnitude /
             * MotionAlert so existing BLE behaviour is structurally identical.
             * ------------------------------------------------------------------ */
            uint16_t fifo_level = 0U;
            int      have_fifo  = 0;     /* 1 if at least one FIFO sample was drained */

            /* Last drained accel values (used for AccelMagnitude / MotionAlert). */
            float last_ax_mg = 0.0f, last_ay_mg = 0.0f, last_az_mg = 0.0f;

            if (LSM6DSL_FIFO_Get_Num_Samples(&lsm6dsl_obj, &fifo_level) == LSM6DSL_OK) {

                /* --- Overrun guard: check STATUS2.over_run hardware bit.
                 * DIFF_FIFO is an 11-bit field (max 2047) so a level-based
                 * threshold of 2048 is unreachable; use the dedicated bit
                 * instead.  On overrun in continuous mode the oldest samples
                 * have already been silently discarded; a BYPASS→STREAM reset
                 * clears the FIFO and restores a known-good start state. --- */
                lsm6dsl_fifo_status2_t status2;
                if (lsm6dsl_read_reg(&lsm6dsl_obj.Ctx,
                                     LSM6DSL_FIFO_STATUS2,
                                     (uint8_t *)&status2, 1) == 0 &&
                    status2.over_run) {
                    PRINTF("[imu] FIFO overrun (level=%u); resetting FIFO\n",
                           (unsigned)fifo_level);
                    /* Bypass then re-enable stream mode to flush and restart. */
                    (void)LSM6DSL_FIFO_Set_Mode(&lsm6dsl_obj,
                                                (uint8_t)LSM6DSL_BYPASS_MODE);
                    (void)LSM6DSL_FIFO_Set_Mode(&lsm6dsl_obj,
                                                (uint8_t)LSM6DSL_STREAM_MODE);
                    fifo_level = 0U;
                }

                /* --- Normal drain: each XYZ triplet consumes 3 FIFO words. ---
                 * fifo_level is a word count; divide by 3 to get triplet count.
                 * Cap at FIFO_DRAIN_MAX to bound worst-case CPU time. */
                uint16_t triplets = fifo_level / 3U;
                if (triplets > FIFO_DRAIN_MAX) {
                    triplets = FIFO_DRAIN_MAX;
                }

                for (uint16_t i = 0U; i < triplets; i++) {
                    float ax_mg = 0.0f, ay_mg = 0.0f, az_mg = 0.0f;
                    if (fifo_read_xyz_mg(&ax_mg, &ay_mg, &az_mg) != 0) {
                        break;  /* read error — stop draining this tick */
                    }
                    ImuDsp_FeedSample(ax_mg, ay_mg, az_mg);

                    /* Keep track of the last successfully drained sample. */
                    last_ax_mg = ax_mg;
                    last_ay_mg = ay_mg;
                    last_az_mg = az_mg;
                    have_fifo  = 1;
                }
            }

            /* --- Fallback: if FIFO has never reported data, use direct read. ---
             * This preserves AccelMagnitude / MotionAlert even if the FIFO
             * configuration needs on-hardware tuning. */
            if (!have_fifo) {
                if (!fifo_fallback_logged) {
                    PRINTF("[imu] FIFO empty, falling back to direct reads\n");
                    fifo_fallback_logged = 1;
                }
                LSM6DSL_Axes_t a_direct = {0};
                if (LSM6DSL_ACC_GetAxes(&lsm6dsl_obj, &a_direct) == LSM6DSL_OK) {
                    last_ax_mg = (float)a_direct.x;
                    last_ay_mg = (float)a_direct.y;
                    last_az_mg = (float)a_direct.z;
                    have_fifo  = 1;   /* treat fallback value as valid */
                }
            }

            /* ------------------------------------------------------------------
             * AccelMagnitude + MotionAlert (uses the last drained/fallback sample).
             * Semantics are bit-for-bit identical to the pre-FIFO implementation:
             * the value is in g, includes the static gravity component, and is
             * pushed to BLE at 4 Hz.
             * ------------------------------------------------------------------ */
            if (have_fifo) {
                /* Convert mg → g for existing MotionAlert thresholds. */
                float ax = last_ax_mg / 1000.0f;
                float ay = last_ay_mg / 1000.0f;
                float az = last_az_mg / 1000.0f;
                float a_mag = sqrtf(ax*ax + ay*ay + az*az);

                /* Gyro: still polled directly (not batched in FIFO). */
                LSM6DSL_Axes_t g = {0};
                float g_mag = 0.0f;
                if (LSM6DSL_GYRO_GetAxes(&lsm6dsl_obj, &g) == LSM6DSL_OK) {
                    float gx = g.x / 1000.0f;
                    float gy = g.y / 1000.0f;
                    float gz = g.z / 1000.0f;
                    g_mag = sqrtf(gx*gx + gy*gy + gz*gz);
                }

                NotifyQueue_PushFloat(HOME_CHAR_ACCEL_MAG, a_mag);
                NotifyQueue_PushFloat(HOME_CHAR_GYRO_MAG,  g_mag);

                /* MotionAlert: edge debounce + lockout. */
                uint32_t now  = HAL_GetTick();
                int      cond = (a_mag > MOTION_ACCEL_THR_G ||
                                 g_mag > MOTION_GYRO_THR_DPS) ? 1 : 0;
                if (cond) {
                    if (motion_high_since == 0U) motion_high_since = now;
                    if (motion_state == 0U &&
                        (now - motion_high_since) >= MOTION_HOLD_MS &&
                        now >= motion_lockout_until) {
                        motion_state = 1U;
                        NotifyQueue_PushU8(HOME_CHAR_MOTION_ALERT, 1U);
                        motion_lockout_until = now + MOTION_LOCKOUT_MS;
                        PRINTF("[motion] ALERT (|a|=%.2f |g|=%.0f)\n", a_mag, g_mag);
                    }
                } else {
                    motion_high_since = 0U;
                    if (motion_state == 1U && now >= motion_lockout_until) {
                        motion_state = 0U;
                        NotifyQueue_PushU8(HOME_CHAR_MOTION_ALERT, 0U);
                        PRINTF("[motion] clear\n");
                    }
                }
            }

            /* ------------------------------------------------------------------
             * IMU DSP: check for completed 1 s window; process VibrationAlert
             * and QuakeAlert hysteresis state machines.
             * ------------------------------------------------------------------ */
            ImuDspResult dsp_result;
            if (ImuDsp_WindowReady(&dsp_result)) {

                /* Always push VibrationRMS even during warm-up. */
                NotifyQueue_PushFloat(HOME_CHAR_VIBRATION_RMS,
                                      dsp_result.vib_rms_mg);

                /* Throttled periodic debug print (every 4 ticks, ~1 Hz). */
                if ((tick & 0x03U) == 0U) {
                    PRINTF("[imu] vib_rms=%.2f mg  quake_rms=%.2f mg  warm=%u\n",
                           (double)dsp_result.vib_rms_mg,
                           (double)dsp_result.quake_rms_mg,
                           (unsigned)dsp_result.warmed_up);
                }

                /* Alert state machines are suppressed during warm-up. */
                if (dsp_result.warmed_up) {

                    /* VibrationAlert: >=VIB_CONSEC_ON windows above VIB_ON_THR_MG
                     * asserts 1; >=VIB_CONSEC_OFF windows below VIB_OFF_THR_MG
                     * de-asserts to 0.  No lockout timer needed per spec. */
                    if (dsp_result.vib_rms_mg > VIB_ON_THR_MG) {
                        vib_consec_on++;
                        vib_consec_off = 0U;
                        if (vib_alert_state == 0U &&
                            vib_consec_on >= VIB_CONSEC_ON) {
                            vib_alert_state = 1U;
                            NotifyQueue_PushU8(HOME_CHAR_VIBRATION_ALERT, 1U);
                            PRINTF("[vib] ALERT (vib_rms=%.2f mg)\n",
                                   (double)dsp_result.vib_rms_mg);
                        }
                    } else if (dsp_result.vib_rms_mg < VIB_OFF_THR_MG) {
                        vib_consec_off++;
                        vib_consec_on = 0U;
                        if (vib_alert_state == 1U &&
                            vib_consec_off >= VIB_CONSEC_OFF) {
                            vib_alert_state = 0U;
                            NotifyQueue_PushU8(HOME_CHAR_VIBRATION_ALERT, 0U);
                            PRINTF("[vib] clear\n");
                        }
                    } else {
                        /* Value between VIB_OFF and VIB_ON: do not advance either
                         * counter; hysteresis band holds current state. */
                    }

                    /* QuakeAlert: >=QUAKE_CONSEC_ON windows above QUAKE_THR_MG
                     * asserts 1; >=QUAKE_CONSEC_OFF below de-asserts to 0.
                     * 2 s lockout between any state transition (AlarmDetected
                     * pattern from audio_task.c). */
                    uint32_t now = HAL_GetTick();
                    if (dsp_result.quake_rms_mg > QUAKE_THR_MG) {
                        quake_consec_on++;
                        quake_consec_off = 0U;
                        if (quake_alert_state == 0U &&
                            quake_consec_on >= QUAKE_CONSEC_ON &&
                            now >= quake_lockout_until) {
                            quake_alert_state = 1U;
                            NotifyQueue_PushU8(HOME_CHAR_QUAKE_ALERT, 1U);
                            quake_lockout_until = now + QUAKE_LOCKOUT_MS;
                            PRINTF("[quake] ALERT (quake_rms=%.2f mg)\n",
                                   (double)dsp_result.quake_rms_mg);
                        }
                    } else {
                        quake_consec_off++;
                        quake_consec_on = 0U;
                        if (quake_alert_state == 1U &&
                            quake_consec_off >= QUAKE_CONSEC_OFF &&
                            now >= quake_lockout_until) {
                            quake_alert_state = 0U;
                            NotifyQueue_PushU8(HOME_CHAR_QUAKE_ALERT, 0U);
                            quake_lockout_until = now + QUAKE_LOCKOUT_MS;
                            PRINTF("[quake] clear\n");
                        }
                    }
                }
            }

            /* Periodic [imu] axes + magnitude debug print (once per second). */
            if ((tick & 0x03U) == 0U && have_fifo) {
                float ax = last_ax_mg / 1000.0f;
                float ay = last_ay_mg / 1000.0f;
                float az = last_az_mg / 1000.0f;
                float a_mag = sqrtf(ax*ax + ay*ay + az*az);
                PRINTF("[imu] a=(%.2f,%.2f,%.2f)g |a|=%.2f\n", ax, ay, az, a_mag);
            }
        }

        /* HTS221 @ 1 Hz: read, push, print. */
        if (hts_ok && (tick & 0x03U) == 0U) {
            float t = 0.0f, h = 0.0f;
            int tr = HTS221_TEMP_GetTemperature(&hts221_obj, &t);
            int hr = HTS221_HUM_GetHumidity(&hts221_obj,     &h);
            if (tr == HTS221_OK && hr == HTS221_OK) {
                NotifyQueue_PushFloat(HOME_CHAR_TEMPERATURE, t);
                NotifyQueue_PushFloat(HOME_CHAR_HUMIDITY,    h);
                PRINTF("[env] T=%.1fC H=%.1f%%\n", t, h);
            } else {
                PRINTF("[env] read failed (temp=%d hum=%d)\n", tr, hr);
            }
        }

        tick++;
        osDelay(SENSOR_TICK_MS);
    }
}
