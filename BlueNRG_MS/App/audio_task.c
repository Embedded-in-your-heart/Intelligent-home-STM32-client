/**
  ******************************************************************************
  * @file    App/audio_task.c
  * @brief   MP34DT01 PDM microphone → MicLevel + LoudAlert over BLE.
  *
  *          Polling-mode production path. DMA is bypassed because of the
  *          CubeMX MSP-init bug (CSELR.C4S=0 → ADC2 instead of DFSDM1_FLT0,
  *          see docs §15.1). Microphone hardware itself is verified healthy
  *          via polling on 2026-05-31 — see docs §15.2.6 / §15.2.7.
  *
  *          Each 200 ms tick:
  *           1. burst-poll 80 samples (~10 ms of audio @ ~8 kHz)
  *           2. compute RMS over the burst
  *           3. normalise to MicLevel ∈ [0, 1023] and push via NotifyQueue
  *           4. apply LoudAlert hold/lockout state machine (docs §5.5)
  *           5. print one [mic] line per second to VCP for visibility
  *
  *          Burst sampling captures ~5 % of the audio stream — enough for
  *          energy / event detection at low CPU cost. DMA-based 100 % capture
  *          is deferred to a future fix of §15.1 (Phase 4 optimisation, not
  *          a blocker for Phase 1/2/3 functionality).
  ******************************************************************************
  */

#include "audio_task.h"

#include <math.h>
#include <stdint.h>
#include <limits.h>

#include "cmsis_os.h"
#include "stm32l4xx_hal.h"

#include "dfsdm.h"                   /* hdfsdm1_filter0 */
#include "bluenrg_conf.h"            /* PRINTF */
#include "notify_queue.h"

/* Polling cadence ----------------------------------------------------------*/
#define BURST_SAMPLES       80U
#define BURST_INTERVAL_MS   200U
#define POLL_TIMEOUT_MS     20U

/* MicLevel normalisation.
 * Empirical calibration from polling health check (2026-05-31):
 *   quiet baseline rms  ≈ 0..1
 *   sustained loud speech rms ≈ 100..130
 *   peaks (clap / shout) untested but expected > 300
 * Scale ×8 puts loud speech at ~0.8 of full scale; peaks clamp at 1023.
 * Tune MIC_SCALE_NUM for noisier / quieter rooms. */
#define MIC_SCALE_NUM       8U
#define MIC_LEVEL_MAX       1023U

/* LoudAlert thresholds + debounce (docs §5.5). */
#define LOUD_THRESHOLD      400U     /* MicLevel above this counts as "loud" */
#define LOUD_HOLD_MS        200U     /* must persist this long to assert     */
#define LOUD_LOCKOUT_MS     1000U    /* min interval between state changes   */

/* Task plumbing -------------------------------------------------------------*/
static osThreadId_t audioTaskHandle;
static const osThreadAttr_t audioTask_attributes = {
    .name       = "AudioTask",
    .stack_size = 1024,
    .priority   = (osPriority_t)osPriorityNormal,
};

static void StartAudioTask(void *argument);

/* Public --------------------------------------------------------------------*/

void AudioTask_Create(void)
{
    audioTaskHandle = osThreadNew(StartAudioTask, NULL, &audioTask_attributes);
}

/* Internal ------------------------------------------------------------------*/

static void StartAudioTask(void *argument)
{
    (void)argument;

    osDelay(700);

    if (HAL_DFSDM_FilterRegularStart(&hdfsdm1_filter0) != HAL_OK) {
        PRINTF("DFSDM polling start FAILED; AudioTask idle.\n");
        for (;;) osDelay(1000);
    }
    PRINTF("AudioTask started (polling → BLE, %u samples / %u ms).\n",
           (unsigned)BURST_SAMPLES, (unsigned)BURST_INTERVAL_MS);

    /* LoudAlert state (preserved across iterations). */
    static uint32_t loud_high_since    = 0U;
    static uint32_t loud_lockout_until = 0U;
    static uint8_t  loud_state         = 0U;

    /* Throttle [mic] PRINTF to ~1 Hz (every 5 bursts). */
    static uint32_t print_tick = 0U;

    for (;;) {
        int64_t  sumsq = 0;
        uint32_t got   = 0U;

        for (uint32_t i = 0U; i < BURST_SAMPLES; i++) {
            if (HAL_DFSDM_FilterPollForRegConversion(&hdfsdm1_filter0,
                                                     POLL_TIMEOUT_MS) != HAL_OK) {
                break;
            }
            uint32_t ch;
            int32_t  v = HAL_DFSDM_FilterGetRegularValue(&hdfsdm1_filter0, &ch);
            int32_t  s = v >> 8;
            sumsq += (int64_t)s * (int64_t)s;
            got++;
        }

        uint32_t rms       = 0U;
        uint32_t mic_level = 0U;

        if (got > 0U) {
            rms       = (uint32_t)sqrtf((float)((double)sumsq / (double)got));
            mic_level = rms * MIC_SCALE_NUM;
            if (mic_level > MIC_LEVEL_MAX) mic_level = MIC_LEVEL_MAX;
        }

        /* Always push MicLevel — zero is a valid "silent" reading. */
        NotifyQueue_PushU16(HOME_CHAR_MIC_LEVEL, (uint16_t)mic_level);

        /* LoudAlert state machine — same pattern as MotionAlert in sensor_task.c. */
        uint32_t now = HAL_GetTick();
        int      cond = (mic_level > LOUD_THRESHOLD) ? 1 : 0;

        if (cond) {
            if (loud_high_since == 0U) loud_high_since = now;
            if (loud_state == 0U &&
                (now - loud_high_since) >= LOUD_HOLD_MS &&
                now >= loud_lockout_until) {
                loud_state = 1U;
                NotifyQueue_PushU8(HOME_CHAR_LOUD_ALERT, 1U);
                loud_lockout_until = now + LOUD_LOCKOUT_MS;
                PRINTF("[loud] ALERT (mic=%lu)\n", (unsigned long)mic_level);
            }
        } else {
            loud_high_since = 0U;
            if (loud_state == 1U && now >= loud_lockout_until) {
                loud_state = 0U;
                NotifyQueue_PushU8(HOME_CHAR_LOUD_ALERT, 0U);
                PRINTF("[loud] clear\n");
            }
        }

        /* 1 Hz serial trace. */
        if ((print_tick++ & 0x3U) == 0U) {
            if (got > 0U) {
                PRINTF("[mic] rms=%lu  lvl=%lu\n",
                       (unsigned long)rms, (unsigned long)mic_level);
            } else {
                PRINTF("[mic] TIMEOUT\n");
            }
        }

        osDelay(BURST_INTERVAL_MS);
    }
}
