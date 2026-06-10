/**
  ******************************************************************************
  * @file    App/audio_dsp.h
  * @brief   Audio DSP module: A-weighted dB level, FFT-based sound class, and
  *          alarm-tone flag from a 200 ms DFSDM window.
  *
  *          The caller (audio_task.c) passes one 200 ms half of the circular
  *          DMA buffer (1600 int32_t samples at ~8 kHz).  On return the struct
  *          holds:
  *            - dba:         A-weighted level in dBSPL (float, calibration-pending)
  *            - sound_class: 0=quiet 1=speech 2=clap 3=alarm 4=other
  *            - is_alarm_tone: 1 when class == 3 for this window
  *
  *          All thresholds / offsets marked "(calibration pending)" are guesses
  *          that should be measured against real room conditions before shipping.
  ******************************************************************************
  */

#ifndef AUDIO_DSP_H
#define AUDIO_DSP_H

#include <stdint.h>

/* Sound-class constants (BLE contract: SoundClass characteristic) ----------*/
#define AUDIO_CLASS_QUIET   0U   /**< Essentially silent — MicLevel-scale rms < 50  */
#define AUDIO_CLASS_SPEECH  1U   /**< Voice-dominated — mid-band energy fraction > 0.55  */
#define AUDIO_CLASS_CLAP    2U   /**< Impulsive transient — sudden >4× RMS rise  */
#define AUDIO_CLASS_ALARM   3U   /**< Narrow-band tone 2800–3600 Hz — peak_ratio > 0.30  */
#define AUDIO_CLASS_OTHER   4U   /**< Does not match any above rule  */

/* A-weighting dB calibration offset (additive, applied after 20*log10).
 * Default 30.0 dB is a rough estimate; measure against a calibrated SPL meter
 * and adjust AUDIO_DBA_CAL_OFFSET before shipping.  (calibration pending)    */
#define AUDIO_DBA_CAL_OFFSET  30.0f

/**
 * @brief Result produced by AudioDsp_Process() for a single 200 ms window.
 */
typedef struct {
    float   dba;           /**< A-weighted level in dB (calibration-pending offset applied) */
    uint8_t sound_class;   /**< AUDIO_CLASS_* value for this window */
    uint8_t is_alarm_tone; /**< 1 when sound_class == AUDIO_CLASS_ALARM, else 0 */
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
