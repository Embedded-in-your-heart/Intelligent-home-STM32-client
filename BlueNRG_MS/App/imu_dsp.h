/**
  ******************************************************************************
  * @file    App/imu_dsp.h
  * @brief   IMU DSP module: vibration RMS and seismic band RMS from a
  *          104-sample (1 s) accelerometer window.
  *
  *          The caller (sensor_task.c) feeds one sample at a time at ~104 Hz
  *          via ImuDsp_FeedSample().  ImuDsp_WindowReady() returns 1 once per
  *          completed 104-sample window and fills an ImuDspResult.
  *
  *          Signal chain (per window):
  *            - Per-axis 2nd-order Butterworth high-pass (fc = 0.4 Hz) removes
  *              gravity component, yielding linear acceleration per axis.
  *            - Linear acceleration magnitude: |a_lin| = sqrt(hx^2+hy^2+hz^2).
  *            - Vibration path: RMS of |a_lin| over the 104-sample window
  *              → ImuDspResult.vib_rms_mg.
  *            - Quake path: 2nd-order Butterworth band-pass (1–10 Hz) applied
  *              to |a_lin|; RMS of band signal over the window
  *              → ImuDspResult.quake_rms_mg.
  *            - warmed_up is 0 for the first 5 s (5 × 104 = 520 samples) after
  *              ImuDsp_Init() to allow filter state to settle.
  *
  *          All thresholds for VibrationAlert / QuakeAlert hysteresis are
  *          defined and evaluated by the caller; this module only produces
  *          the per-window RMS values.
  *
  *          All parameters marked "(calibration pending)" are design-time
  *          estimates and should be validated against the target hardware.
  ******************************************************************************
  */

#ifndef IMU_DSP_H
#define IMU_DSP_H

#include <stdint.h>

/**
 * @brief Result produced by ImuDsp_WindowReady() for a single 1 s window.
 */
typedef struct {
    float   vib_rms_mg;    /* 1 s window RMS of high-passed |a|, in mg */
    float   quake_rms_mg;  /* 1 s window RMS of the 1-10 Hz band signal, in mg */
    uint8_t warmed_up;     /* 0 during the 5 s post-init settling period */
} ImuDspResult;

/**
 * @brief Initialise static filter state and sample counters.
 *        Call once before the first ImuDsp_FeedSample() call.
 *        Must not be called from an ISR.
 */
void ImuDsp_Init(void);

/**
 * @brief Feed one accelerometer sample (called at ~104 Hz).
 *
 * @param ax_mg  X-axis acceleration in milli-g (raw sensor output).
 * @param ay_mg  Y-axis acceleration in milli-g.
 * @param az_mg  Z-axis acceleration in milli-g.
 */
void ImuDsp_FeedSample(float ax_mg, float ay_mg, float az_mg);

/**
 * @brief Poll for a completed 1 s window result.
 *
 * @param out  Caller-allocated result struct; written only when returning 1.
 *             Must not be NULL when return value may be 1.
 * @return 1 once per completed 104-sample window, 0 otherwise.
 */
int ImuDsp_WindowReady(ImuDspResult *out);

#endif /* IMU_DSP_H */
