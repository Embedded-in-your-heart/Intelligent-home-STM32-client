/**
  ******************************************************************************
  * @file    App/audio_dsp.h
  * @brief   Audio DSP module: A-weighted dB level and alarm-tone detection
  *          from a 200 ms DFSDM window.
  *
  *          The caller (audio_task.c) passes one 200 ms half of the circular
  *          DMA buffer (1600 int32_t samples at ~8 kHz).  On return the struct
  *          holds:
  *            - dba:          A-weighted level in dBSPL (float, calibration-pending)
  *            - is_alarm_tone: 1 when a narrow-band alarm tone (2800–3600 Hz) is
  *                            detected in this window via FFT peak ratio, else 0
  *
  *          All thresholds / offsets marked "(calibration pending)" are guesses
  *          that should be measured against real room conditions before shipping.
  ******************************************************************************
  */

#ifndef AUDIO_DSP_H
#define AUDIO_DSP_H

#include <stdint.h>

/* A-weighting dB calibration offset (additive, applied after 20*log10).
 * Default 30.0 dB is a rough estimate; measure against a calibrated SPL meter
 * and adjust AUDIO_DBA_CAL_OFFSET before shipping.  (calibration pending)    */
#define AUDIO_DBA_CAL_OFFSET  30.0f

/**
 * @brief Result produced by AudioDsp_Process() for a single 200 ms window.
 */
typedef struct {
    float   dba;           /**< A-weighted level in dB (calibration-pending offset applied) */
    float   rms_weighted;  /**< raw A-weighted RMS (MicLevel sample scale) — serial diagnostics */
    uint8_t is_alarm_tone; /**< 1 when a narrow-band alarm tone (2800–3600 Hz) is detected */
} AudioDspResult;

/**
 * @brief Initialise static work state (Hanning table, FFT instance, biquad
 *        instance).  Call once before the first AudioDsp_Process() call.
 *        Must not be called from an ISR.
 */
void AudioDsp_Init(void);

/**
 * @brief Process one 200 ms DFSDM half-buffer and produce an AudioDspResult.
 *
 * @param raw  Pointer to n raw FLTRDATAR words (int32_t).
 *             Calibrated sample = raw[i] >> 16  (same shift as audio_task.c).
 * @param n    Number of samples.  Must be 1600.
 * @param out  Caller-allocated result struct; never NULL.
 */
void AudioDsp_Process(const int32_t *raw, uint32_t n, AudioDspResult *out);

#endif /* AUDIO_DSP_H */
