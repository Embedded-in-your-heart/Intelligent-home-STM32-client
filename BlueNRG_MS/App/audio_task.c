/**
  ******************************************************************************
  * @file    App/audio_task.c
  * @brief   MP34DT01 PDM microphone → MicLevel + LoudAlert over BLE.
  *
  *          DMA + interrupt-driven production path. DFSDM1 filter 0 streams
  *          24-bit-signed audio at ~8 kHz into a circular DMA buffer
  *          (DMA1_Channel4). The HAL half/full conversion callbacks run in the
  *          DMA IRQ and merely signal AudioTask via thread flags; the task
  *          blocks until a half completes, computes RMS over that 200 ms
  *          window, normalises to MicLevel and applies the LoudAlert state
  *          machine (docs §5.5). CPU cost is near zero — the DMA captures
  *          every sample autonomously (100 % coverage vs ~5 % for polling).
  *
  *          History: an earlier revision used DMA but appeared to stall; that
  *          was traced (issue #1) to a faulty dev board — the DMA config and
  *          generated MSP were correct all along (CSELR.C4S=0 = DFSDM1_FLT0
  *          per RM0351 Rev 9 Table 44). Swapping boards restored DMA, so this
  *          replaces the interim polling workaround.
  *
  *          Calibration note: the polling path computed each sample as
  *          HAL_DFSDM_FilterGetRegularValue() >> 8, and that HAL call already
  *          returns the sign-extended 24-bit sample (raw FLTRDATAR / 256). The
  *          DMA buffer instead holds the *raw* FLTRDATAR, so to reproduce the
  *          identical sample magnitude — and keep MIC_SCALE_NUM / LOUD_THRESHOLD
  *          valid — we shift the raw word right by 16: raw >> 16 == sample24 >> 8.
  ******************************************************************************
  */

#include "audio_task.h"

#include <math.h>
#include <stdint.h>

#include "cmsis_os.h"
#include "stm32l4xx_hal.h"

#include "dfsdm.h"                   /* hdfsdm1_filter0 */
#include "bluenrg_conf.h"            /* PRINTF */
#include "notify_queue.h"
#include "audio_dsp.h"               /* AudioDsp_Init, AudioDsp_Process */

/* Circular DMA buffer: two halves, each a 200 ms window @ ~8 kHz.
 * A half-complete (HT) and full-complete (TC) event therefore arrives every
 * 200 ms, matching the documented MicLevel cadence (docs §5.4). */
#define HALF_SAMPLES        1600U
#define DMA_BUF_LEN         (HALF_SAMPLES * 2U)

/* Thread flags raised from the DMA IRQ callbacks. */
#define FLAG_HALF0          0x01U     /* first half (indices 0..HALF-1) ready  */
#define FLAG_HALF1          0x02U     /* second half (indices HALF..2*HALF) ok  */
#define FLAG_ANY            (FLAG_HALF0 | FLAG_HALF1)
#define WAIT_TIMEOUT_MS     1000U

/* Raw FLTRDATAR → calibrated sample. See header note. */
#define SAMPLE_SHIFT        16

/* MicLevel normalisation (calibrated 2026-05-31 against the polling path):
 *   quiet baseline rms ≈ 0..1, sustained loud speech rms ≈ 100..130.
 * Scale ×8 puts loud speech near full scale; peaks clamp at 1023. */
#define MIC_SCALE_NUM       8U
#define MIC_LEVEL_MAX       1023U

/* LoudAlert threshold + debounce (docs §5.5; threshold tuned 2026-05-31). */
#define LOUD_THRESHOLD      400U     /* MicLevel above this counts as "loud" */
#define LOUD_HOLD_MS        200U     /* must persist this long to assert     */
#define LOUD_LOCKOUT_MS     1000U    /* min interval between state changes   */

/* AlarmDetected debounce parameters (BLE contract). */
#define ALARM_CONSEC_ON     3U       /* consecutive alarm windows to assert AlarmDetected=1 */
#define ALARM_CONSEC_OFF    5U       /* consecutive non-alarm windows to de-assert */
#define ALARM_LOCKOUT_MS    2000U    /* minimum ms between AlarmDetected state changes */

/* DMA target buffer (BSS, zeroed at boot). Filled by DMA1_Channel4. */
static int32_t dma_buf[DMA_BUF_LEN];

/* Task plumbing -------------------------------------------------------------*/
static osThreadId_t audioTaskHandle;
static const osThreadAttr_t audioTask_attributes = {
    .name       = "AudioTask",
    .stack_size = 1536,
    .priority   = (osPriority_t)osPriorityNormal,
};

static void     StartAudioTask(void *argument);
static uint32_t rms_of_half(const int32_t *half);

/* Public --------------------------------------------------------------------*/

void AudioTask_Create(void)
{
    audioTaskHandle = osThreadNew(StartAudioTask, NULL, &audioTask_attributes);
}

/* DMA HT/TC callbacks — run in DMA1_Channel4 IRQ context.
 *
 * They only flag the task: the half just completed is no longer being written
 * by DMA (which is now filling the other half), so AudioTask can read it
 * without a race. The heavy sum-of-squares loop stays out of the ISR.
 * osThreadFlagsSet is ISR-safe under CMSIS-RTOS2.
 */
void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *h)
{
    if (h->Instance == DFSDM1_Filter0) {
        (void)osThreadFlagsSet(audioTaskHandle, FLAG_HALF0);
    }
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *h)
{
    if (h->Instance == DFSDM1_Filter0) {
        (void)osThreadFlagsSet(audioTaskHandle, FLAG_HALF1);
    }
}

/* Internal ------------------------------------------------------------------*/

static uint32_t rms_of_half(const int32_t *half)
{
    int64_t sumsq = 0;
    for (uint32_t i = 0U; i < HALF_SAMPLES; i++) {
        int32_t s = half[i] >> SAMPLE_SHIFT;     /* raw FLTRDATAR → sample24>>8 */
        sumsq += (int64_t)s * (int64_t)s;
    }
    return (uint32_t)sqrtf((float)((double)sumsq / (double)HALF_SAMPLES));
}

static void StartAudioTask(void *argument)
{
    (void)argument;

    /* Let BLE + SensorTask emit their boot logs first. */
    osDelay(700);

    /* Initialise DSP module (Hanning table, FFT, biquad) before DMA starts. */
    AudioDsp_Init();

    if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, dma_buf, DMA_BUF_LEN)
            != HAL_OK) {
        PRINTF("DFSDM DMA start FAILED; AudioTask idle.\r\n");
        for (;;) osDelay(1000);
    }
    PRINTF("AudioTask started (DFSDM + DMA interrupt, %u-sample window @ ~8 kHz).\r\n",
           (unsigned)HALF_SAMPLES);

    /* LoudAlert state (preserved across iterations). */
    uint32_t loud_high_since    = 0U;
    uint32_t loud_lockout_until = 0U;
    uint8_t  loud_state         = 0U;

    /* AlarmDetected state machine state. */
    uint32_t alarm_consec_on     = 0U;   /* consecutive alarm-tone windows */
    uint32_t alarm_consec_off    = 0U;   /* consecutive non-alarm windows  */
    uint32_t alarm_lockout_until = 0U;   /* tick deadline for next change  */
    uint8_t  alarm_state         = 0U;   /* current AlarmDetected value    */

    /* Throttle [mic] PRINTF to ~1.25 Hz (every 4th 200 ms window). */
    uint32_t print_tick = 0U;

    for (;;) {
        uint32_t flags = osThreadFlagsWait(FLAG_ANY, osFlagsWaitAny, WAIT_TIMEOUT_MS);
        if (flags & osFlagsError) {
            PRINTF("[mic] TIMEOUT (no DMA half-complete)\r\n");
            continue;
        }

        /* Process whichever half/halves completed, oldest (HALF0) first. */
        for (uint32_t half = 0U; half < 2U; half++) {
            uint32_t bit = (half == 0U) ? FLAG_HALF0 : FLAG_HALF1;
            if (!(flags & bit)) {
                continue;
            }
            const int32_t *buf = (half == 0U) ? &dma_buf[0] : &dma_buf[HALF_SAMPLES];

            uint32_t rms       = rms_of_half(buf);
            uint32_t mic_level = rms * MIC_SCALE_NUM;
            if (mic_level > MIC_LEVEL_MAX) {
                mic_level = MIC_LEVEL_MAX;
            }

            /* Always push MicLevel — zero is a valid "silent" reading. */
            NotifyQueue_PushU16(HOME_CHAR_MIC_LEVEL, (uint16_t)mic_level);

            /* LoudAlert state machine — same pattern as MotionAlert. */
            uint32_t now  = HAL_GetTick();
            int      cond = (mic_level > LOUD_THRESHOLD) ? 1 : 0;

            if (cond) {
                if (loud_high_since == 0U) loud_high_since = now;
                if (loud_state == 0U &&
                    (now - loud_high_since) >= LOUD_HOLD_MS &&
                    now >= loud_lockout_until) {
                    loud_state = 1U;
                    NotifyQueue_PushU8(HOME_CHAR_LOUD_ALERT, 1U);
                    loud_lockout_until = now + LOUD_LOCKOUT_MS;
                    PRINTF("[loud] ALERT (mic=%lu)\r\n", (unsigned long)mic_level);
                }
            } else {
                loud_high_since = 0U;
                if (loud_state == 1U && now >= loud_lockout_until) {
                    loud_state = 0U;
                    NotifyQueue_PushU8(HOME_CHAR_LOUD_ALERT, 0U);
                    PRINTF("[loud] clear\r\n");
                }
            }

            /* --- DSP: A-weighted level, sound class, alarm-tone detection --- */
            AudioDspResult res;
            AudioDsp_Process(buf, HALF_SAMPLES, &res);

            /* Push MicDBA every window (always). */
            NotifyQueue_PushFloat(HOME_CHAR_MIC_DBA, res.dba);

            /* AlarmDetected state machine (BLE contract):
             *   - >= ALARM_CONSEC_ON  consecutive alarm windows  → assert 1
             *   - >= ALARM_CONSEC_OFF consecutive non-alarm windows → assert 0
             *   - 2 s lockout between any state transitions
             * Mirrors the style of the LoudAlert block above. */
            now = HAL_GetTick();   /* refresh tick after DSP processing */
            if (res.is_alarm_tone) {
                alarm_consec_on++;
                alarm_consec_off = 0U;
                if (alarm_state == 0U &&
                    alarm_consec_on >= ALARM_CONSEC_ON &&
                    now >= alarm_lockout_until) {
                    alarm_state = 1U;
                    NotifyQueue_PushU8(HOME_CHAR_ALARM_DETECTED, 1U);
                    alarm_lockout_until = now + ALARM_LOCKOUT_MS;
                    PRINTF("[alarm] DETECTED (dba=%.1f)\r\n",
                           (double)res.dba);
                }
            } else {
                alarm_consec_off++;
                alarm_consec_on = 0U;
                if (alarm_state == 1U &&
                    alarm_consec_off >= ALARM_CONSEC_OFF &&
                    now >= alarm_lockout_until) {
                    alarm_state = 0U;
                    NotifyQueue_PushU8(HOME_CHAR_ALARM_DETECTED, 0U);
                    alarm_lockout_until = now + ALARM_LOCKOUT_MS;
                    PRINTF("[alarm] clear\r\n");
                }
            }

            /* Throttled debug print (~1.25 Hz). */
            if ((print_tick++ & 0x3U) == 0U) {
                PRINTF("[mic] rms=%lu  lvl=%lu  dba=%.1f\r\n",
                       (unsigned long)rms, (unsigned long)mic_level,
                       (double)res.dba);
            }
        }
    }
}
