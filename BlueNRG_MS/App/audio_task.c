/**
  ******************************************************************************
  * @file    App/audio_task.c
  * @brief   MP34DT01 PDM microphone → RMS energy — Milestone 3a (serial only).
  *
  *          DFSDM1 (filter 0, channels 1/2 in SPI/internal-clock PDM mode)
  *          streams 24-bit-signed audio at ~8 kHz into a 800-sample circular
  *          buffer via DMA1_Channel4. The two HAL conversion callbacks (half
  *          and full) accumulate sum-of-squares for the just-finished half.
  *          AudioTask wakes every 200 ms, snapshots the accumulator, computes
  *          RMS = sqrt(sumsq / count), and prints it over USART1 VCP.
  *
  *          BLE wiring (MicLevel / LoudAlert) is deferred to M3b.
  ******************************************************************************
  */

#include "audio_task.h"

#include <math.h>
#include <stdint.h>

#include "cmsis_os.h"
#include "stm32l4xx_hal.h"

#include "dfsdm.h"                   /* hdfsdm1_filter0 */
#include "bluenrg_conf.h"            /* PRINTF */

/* DMA buffer: 800 samples * 4 byte = 3.2 KB; half = 50 ms at 8 kHz. */
#define DMA_BUF_LEN     800U
#define DMA_HALF_LEN    (DMA_BUF_LEN / 2U)

/* Window between RMS prints. */
#define WINDOW_MS       200U

/* DMA target buffer (BSS, zeroed at boot). */
static int32_t dma_buf[DMA_BUF_LEN];

/* Shared accumulator (written from DMA IRQ, drained from AudioTask). */
static volatile int64_t  acc_sumsq;
static volatile uint32_t acc_count;

/* Task plumbing -------------------------------------------------------------*/
static osThreadId_t audioTaskHandle;
static const osThreadAttr_t audioTask_attributes = {
    .name       = "AudioTask",
    .stack_size = 1024,
    .priority   = (osPriority_t)osPriorityNormal,
};

static void StartAudioTask(void *argument);
static inline void accumulate_half(const int32_t *p);
static void dump_dfsdm_state(const char *tag);

/* Public --------------------------------------------------------------------*/

void AudioTask_Create(void)
{
    audioTaskHandle = osThreadNew(StartAudioTask, NULL, &audioTask_attributes);
}

/* HAL DFSDM regular-conversion callbacks (override the weak defaults) -------
 *
 * Each callback owns one half of the circular DMA buffer. The other half is
 * still being filled by DMA — safe to read.
 */

void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *h)
{
    if (h->Instance == DFSDM1_Filter0) {
        accumulate_half(&dma_buf[0]);
    }
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *h)
{
    if (h->Instance == DFSDM1_Filter0) {
        accumulate_half(&dma_buf[DMA_HALF_LEN]);
    }
}

/* Internal ------------------------------------------------------------------*/

static void dump_dfsdm_state(const char *tag)
{
    uint32_t ch0  = DFSDM1_Channel0->CHCFGR1;
    uint32_t ch1  = DFSDM1_Channel1->CHCFGR1;
    uint32_t ch2  = DFSDM1_Channel2->CHCFGR1;
    uint32_t cr1  = DFSDM1_Filter0->FLTCR1;
    uint32_t cr2  = DFSDM1_Filter0->FLTCR2;
    uint32_t isr  = DFSDM1_Filter0->FLTISR;
    uint32_t cndtr = hdma_dfsdm1_flt0.Instance->CNDTR;
    uint32_t ccr   = hdma_dfsdm1_flt0.Instance->CCR;
    uint32_t cselr = DMA1_CSELR->CSELR;
    uint32_t c4s   = (cselr >> 12) & 0xFU;

    PRINTF("[diag:%s]\n", tag);
    PRINTF("  CH0 CHCFGR1=%08lX  DFSDMEN=%lu CKOUTDIV=%lu\n",
           (unsigned long)ch0,
           (unsigned long)((ch0 >> 31) & 1U),       /* DFSDMEN is bit 31 */
           (unsigned long)((ch0 >> 16) & 0xFFU));
    PRINTF("  CH1 CHCFGR1=%08lX  CHEN=%lu\n",
           (unsigned long)ch1, (unsigned long)((ch1 >> 7) & 1U));
    PRINTF("  CH2 CHCFGR1=%08lX  CHEN=%lu\n",
           (unsigned long)ch2, (unsigned long)((ch2 >> 7) & 1U));
    PRINTF("  FLT0 CR1=%08lX  DFEN=%lu RDMAEN=%lu RCONT=%lu RCH=%lu RSWSTART=%lu\n",
           (unsigned long)cr1,
           (unsigned long)((cr1 >> 0)  & 1U),
           (unsigned long)((cr1 >> 21) & 1U),
           (unsigned long)((cr1 >> 18) & 1U),
           (unsigned long)((cr1 >> 24) & 7U),
           (unsigned long)((cr1 >> 17) & 1U));
    PRINTF("  FLT0 CR2=%08lX  ISR=%08lX\n",
           (unsigned long)cr2, (unsigned long)isr);
    PRINTF("  DMA1 CCR=%08lX EN=%lu HTIE=%lu TCIE=%lu CIRC=%lu  CNDTR=%lu  CSELR.C4S=%lu\n",
           (unsigned long)ccr,
           (unsigned long)((ccr >> 0) & 1U),
           (unsigned long)((ccr >> 2) & 1U),
           (unsigned long)((ccr >> 1) & 1U),
           (unsigned long)((ccr >> 5) & 1U),
           (unsigned long)cndtr,
           (unsigned long)c4s);
}

static inline void accumulate_half(const int32_t *p)
{
    /* DFSDM filter stores 24-bit signed data in bits [31:8] of FLTRDATAR;
     * the low 8 bits carry channel index. Arithmetic right-shift extracts
     * the sign-extended 24-bit sample. */
    int64_t s = 0;
    for (uint32_t i = 0; i < DMA_HALF_LEN; i++) {
        int32_t v = p[i] >> 8;
        s += (int64_t)v * (int64_t)v;
    }

    /* ISR-only writer. The Task-side reader uses a critical section to take
     * a consistent snapshot, so no synchronisation needed here. */
    acc_sumsq += s;
    acc_count += DMA_HALF_LEN;
}

static void StartAudioTask(void *argument)
{
    (void)argument;

    /* Let BLE + SensorTask emit their boot logs first. */
    osDelay(700);

    /* CubeMX-generated MX_DFSDM1_Init programs hdma_dfsdm1_flt0.Init.Request = 0
     * (DMA_REQUEST_0), which on STM32L475 maps DMA1_Channel4 to ADC2 — NOT
     * DFSDM1_FLT0. The filter produces samples (FLTISR.ROVRF flag goes high)
     * but DMA never receives a request → CNDTR stays at its initial value
     * forever. Per RM0351 Rev 9 Table 41, DFSDM1_FLT0 on DMA1_Channel4 is
     * C4S=7. Re-init the DMA handle with the corrected request — this routes
     * through HAL's own CSELR writer and survives future CubeMX regen as long
     * as the fix stays in our user code. */
    hdma_dfsdm1_flt0.Init.Request = 7U;        /* DFSDM1_FLT0 = C4S 7 */
    (void)HAL_DMA_Init(&hdma_dfsdm1_flt0);
    HAL_DFSDM_FilterMspInit(&hdfsdm1_filter0);
    PRINTF("[patch] CSELR=%08lX  C4S=%lu (want 7)\n",
           (unsigned long)DMA1_CSELR->CSELR,
           (unsigned long)((DMA1_CSELR->CSELR >> 12) & 0xFU));

    dump_dfsdm_state("before-start");
    
    if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, dma_buf, DMA_BUF_LEN) != HAL_OK) {
        PRINTF("DFSDM FilterRegularStart_DMA failed; AudioTask idle.\n");
        for (;;) osDelay(1000);
    }
    PRINTF("AudioTask started (DFSDM running, %lu samples/window @ ~8 kHz).\n",
           (unsigned long)((WINDOW_MS * 8000U) / 1000U));

    dump_dfsdm_state("after-start");
    osDelay(300);
    dump_dfsdm_state("after-300ms");

    for (;;) {
        osDelay(WINDOW_MS);

        /* Snapshot + reset accumulator with IRQ disabled so the
         * sumsq / count pair is consistent. */
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        int64_t  sumsq = acc_sumsq;
        uint32_t count = acc_count;
        acc_sumsq = 0;
        acc_count = 0;
        __set_PRIMASK(primask);

        if (count > 0U) {
            /* Mean of squares — fits comfortably in float for 24-bit-aligned data. */
            float mean = (float)((double)sumsq / (double)count);
            float rms  = sqrtf(mean);
            PRINTF("[mic] rms=%lu (n=%lu)\n",
                   (unsigned long)rms, (unsigned long)count);
        } else {
            PRINTF("[mic] no samples (DMA stalled?)\n");
        }
    }
}
