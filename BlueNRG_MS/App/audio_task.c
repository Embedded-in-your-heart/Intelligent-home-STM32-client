/**
  ******************************************************************************
  * @file    App/audio_task.c
  * @brief   MP34DT01 PDM microphone — polling-mode HEALTH CHECK only.
  *
  *          Bypasses DMA entirely. The DMA path is broken by the CubeMX
  *          MSP-init bug documented in docs §15.1 (DMA1_Channel4's
  *          CSELR.C4S=0 maps to ADC2, not DFSDM1_FLT0). Until that is
  *          fixed, this task reads FLTRDATAR directly through
  *          HAL_DFSDM_FilterPollForRegConversion so we can confirm the
  *          microphone itself produces a valid PDM-decimated signal.
  *
  *          Per 200 ms tick we poll a ~10 ms burst (80 samples @ ~8 kHz),
  *          compute RMS / min / max, and print one line over USART1 VCP.
  *          Sampling 80 / (200 × 8) ≈ 5 % of the audio stream — enough to
  *          observe quiet baseline / clap / speech but light on CPU so
  *          BleTask and SensorTask are not starved.
  *
  *          BLE wiring (MicLevel / LoudAlert) is deferred to M3b, and
  *          ultimately depends on the DMA bug being resolved (§15.2.8).
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

/* 80 samples × 125 µs ≈ 10 ms of audio per burst. */
#define BURST_SAMPLES       80U
#define BURST_INTERVAL_MS   200U
#define POLL_TIMEOUT_MS     20U      /* per-sample wait — plenty at 8 kHz */

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

    /* Let BLE + SensorTask emit their boot logs first. */
    osDelay(700);

    /* Polling mode: filter pushes results to FLTRDATAR; CPU reads on demand. */
    if (HAL_DFSDM_FilterRegularStart(&hdfsdm1_filter0) != HAL_OK) {
        PRINTF("DFSDM polling start FAILED; AudioTask idle.\n");
        for (;;) osDelay(1000);
    }
    PRINTF("AudioTask started (polling, %u samples / %u ms burst).\n",
           (unsigned)BURST_SAMPLES, (unsigned)BURST_INTERVAL_MS);

    for (;;) {
        int64_t  sumsq   = 0;
        int32_t  min_v   = INT32_MAX;
        int32_t  max_v   = INT32_MIN;
        uint32_t got     = 0U;
        uint32_t ch_seen = 0xFFU;    /* sentinel — should become 1 */

        for (uint32_t i = 0U; i < BURST_SAMPLES; i++) {
            if (HAL_DFSDM_FilterPollForRegConversion(&hdfsdm1_filter0,
                                                     POLL_TIMEOUT_MS) != HAL_OK) {
                break;                /* timeout — filter not delivering */
            }
            uint32_t ch;
            int32_t  v = HAL_DFSDM_FilterGetRegularValue(&hdfsdm1_filter0, &ch);
            /* FLTRDATAR holds 24-bit signed in bits [31:8]; low 8 bits = channel ID. */
            int32_t  s = v >> 8;

            sumsq += (int64_t)s * (int64_t)s;
            if (s < min_v) min_v = s;
            if (s > max_v) max_v = s;
            got++;
            ch_seen = ch;
        }

        if (got > 0U) {
            uint32_t rms = (uint32_t)sqrtf((float)((double)sumsq / (double)got));
            PRINTF("[mic-poll] n=%lu ch=%lu  rms=%lu  min=%ld  max=%ld\n",
                   (unsigned long)got, (unsigned long)ch_seen,
                   (unsigned long)rms,
                   (long)min_v, (long)max_v);
        } else {
            PRINTF("[mic-poll] TIMEOUT (no sample from filter)\n");
        }

        osDelay(BURST_INTERVAL_MS);
    }
}
