/**
  ******************************************************************************
  * @file    App/imu_dsp.c
  * @brief   IMU DSP: vibration RMS and seismic band RMS from a streaming
  *          104 Hz accelerometer feed, aggregated over 1 s windows.
  *
  *          Called by sensor_task.c once per LSM6DSL accelerometer sample
  *          (~104 Hz).  ImuDsp_WindowReady() returns 1 and fills an
  *          ImuDspResult once per completed 104-sample (1 s) window.
  *
  *  Signal chain
  *  ============
  *  Per sample:
  *    1. Per-axis high-pass biquad (DF1) removes gravity → linear accel.
  *    2. Linear accel magnitude: mag = sqrt(hx^2 + hy^2 + hz^2).
  *    3. Vibration path: accumulate mag^2 into vib_sum_sq.
  *    4. Quake path: band-pass biquad (DF1) applied to mag scalar
  *                   → band sample; accumulate band^2 into quake_sum_sq.
  *
  *  Per completed window (104 samples):
  *    5. vib_rms  = sqrt(vib_sum_sq  / WINDOW_SIZE)
  *    6. quake_rms = sqrt(quake_sum_sq / WINDOW_SIZE)
  *    7. Set warmed_up once total sample count >= WARMUP_SAMPLES.
  *    8. Reset accumulators and sample counter.
  *
  *  High-pass filter derivation
  *  ===========================
  *  Filter type: 2nd-order Butterworth high-pass
  *  Cutoff:      fc = 0.4 Hz,  fs = 104 Hz
  *  Design:      bilinear transform with pre-warping at fc
  *               wc_d = 2*fs*tan(pi*fc/fs)
  *  Analog HP prototype: H_HP(s) = s^2 / (s^2 + sqrt(2)*wc*s + wc^2)
  *
  *  Band-pass filter derivation
  *  ===========================
  *  Filter type: 2nd-order Butterworth band-pass (1st-order LP->BP transform)
  *  Band:        1–10 Hz,  fs = 104 Hz
  *  Design:      bilinear transform with pre-warped band edges
  *               wl_d = 2*fs*tan(pi*f_lo/fs),  wu_d = 2*fs*tan(pi*f_hi/fs)
  *               w0_sq = wl_d*wu_d,  bw = wu_d - wl_d
  *  Analog BP prototype: H_BP(s) = bw*s / (s^2 + bw*s + w0_sq)
  *
  *  CMSIS DF1 sign convention
  *  =========================
  *  Recurrence: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2]
  *                           - a1*y[n-1] - a2*y[n-2]
  *  Coefficient array order per stage: { b0, b1, b2, -a1_std, -a2_std }
  *  i.e. the a-coefficients stored are the NEGATED standard denominator
  *  coefficients.  This matches the CMSIS-DSP V1.6.0 documentation.
  ******************************************************************************
  */

#include "imu_dsp.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "arm_math.h"

/* =========================================================================
 * Compile-time constants
 * ========================================================================= */

/** Number of accelerometer samples per 1 s window at nominal 104 Hz ODR. */
#define WINDOW_SIZE     104U

/** Number of windows to discard during filter settling (5 s warm-up). */
#define WARMUP_WINDOWS  5U

/** Total samples before warmed_up is set. */
#define WARMUP_SAMPLES  (WARMUP_WINDOWS * WINDOW_SIZE)   /* 520 */

/* =========================================================================
 * High-pass filter coefficients
 *
 * 2nd-order Butterworth HPF: fc = 0.4 Hz, fs = 104 Hz
 * Bilinear transform with pre-warping.
 * -3 dB at exactly 0.4 Hz; < -24 dB at 0.1 Hz; passband flat above 1 Hz.
 *
 * CMSIS DF1 order per stage: { b0, b1, b2, neg_a1, neg_a2 }
 * where neg_a1 = -(a1_standard),  neg_a2 = -(a2_standard).
 *
 * Standard coefficients:
 *   b0 =  0.9830571504,  b1 = -1.9661143007,  b2 = 0.9830571504
 *   a1 = -1.9658272199,  a2 =  0.9664013815
 * Stored as (b0, b1, b2, -a1, -a2):
 *   { 0.9830571504, -1.9661143007, 0.9830571504, 1.9658272199, -0.9664013815 }
 * ========================================================================= */
#define HP_NUM_STAGES   1U

static const float32_t hp_coeffs[HP_NUM_STAGES * 5U] = {
    /* Stage 1 — 2nd-order Butterworth HP, fc=0.4 Hz, fs=104 Hz */
    +0.9830571504f, -1.9661143007f, +0.9830571504f, +1.9658272199f, -0.9664013815f,
};

/* =========================================================================
 * Band-pass filter coefficients
 *
 * 2nd-order Butterworth BPF: 1–10 Hz, fs = 104 Hz
 * Bilinear transform with pre-warped band-edge frequencies.
 * -3 dB at 1 Hz and 10 Hz; peak near 3 Hz; > -1 dB over 2–8 Hz.
 *
 * CMSIS DF1 order per stage: { b0, b1, b2, neg_a1, neg_a2 }
 *
 * Standard coefficients:
 *   b0 =  0.2179990954,  b1 =  0.0,  b2 = -0.2179990954
 *   a1 = -1.5348234570,  a2 =  0.5640018092
 * Stored as (b0, b1, b2, -a1, -a2):
 *   { 0.2179990954, 0.0, -0.2179990954, 1.5348234570, -0.5640018092 }
 * ========================================================================= */
#define BP_NUM_STAGES   1U

static const float32_t bp_coeffs[BP_NUM_STAGES * 5U] = {
    /* Stage 1 — 2nd-order Butterworth BP, 1-10 Hz, fs=104 Hz */
    +0.2179990954f, +0.0f, -0.2179990954f, +1.5348234570f, -0.5640018092f,
};

/* =========================================================================
 * Static state (BSS — zero-initialised, never on stack)
 * ========================================================================= */

/* Biquad filter state: 4 words per stage (x[n-1], x[n-2], y[n-1], y[n-2]). */
static float32_t hp_state_x[HP_NUM_STAGES * 4U];   /* HP filter, X axis */
static float32_t hp_state_y[HP_NUM_STAGES * 4U];   /* HP filter, Y axis */
static float32_t hp_state_z[HP_NUM_STAGES * 4U];   /* HP filter, Z axis */
static float32_t bp_state  [BP_NUM_STAGES * 4U];   /* BP filter, magnitude */

/* CMSIS biquad instance handles. */
static arm_biquad_casd_df1_inst_f32 hp_inst_x;
static arm_biquad_casd_df1_inst_f32 hp_inst_y;
static arm_biquad_casd_df1_inst_f32 hp_inst_z;
static arm_biquad_casd_df1_inst_f32 bp_inst;

/* Window accumulation state. */
static float    s_vib_sum_sq;    /* sum of mag^2 over current window */
static float    s_quake_sum_sq;  /* sum of band_sample^2 over current window */
static uint32_t s_window_cnt;    /* samples accumulated in current window */
static uint32_t s_total_samples; /* total samples since ImuDsp_Init() */

/* Ready-window latch: set to 1 by FeedSample on window completion,
 * cleared to 0 by WindowReady after the result has been consumed. */
static volatile uint8_t s_window_ready;

/* Latched result written at window completion. */
static ImuDspResult s_result;

/* =========================================================================
 * Public functions
 * ========================================================================= */

void ImuDsp_Init(void)
{
    /* Zero all filter state explicitly (BSS handles static init but
     * re-initialisation calls must also clear prior filter history). */
    memset(hp_state_x, 0, sizeof(hp_state_x));
    memset(hp_state_y, 0, sizeof(hp_state_y));
    memset(hp_state_z, 0, sizeof(hp_state_z));
    memset(bp_state,   0, sizeof(bp_state));

    /* Initialise per-axis high-pass instances (all share the same coefficient
     * array; state arrays are separate so each axis has independent history). */
    arm_biquad_cascade_df1_init_f32(&hp_inst_x,
                                    (uint8_t)HP_NUM_STAGES,
                                    hp_coeffs,
                                    hp_state_x);
    arm_biquad_cascade_df1_init_f32(&hp_inst_y,
                                    (uint8_t)HP_NUM_STAGES,
                                    hp_coeffs,
                                    hp_state_y);
    arm_biquad_cascade_df1_init_f32(&hp_inst_z,
                                    (uint8_t)HP_NUM_STAGES,
                                    hp_coeffs,
                                    hp_state_z);

    /* Initialise band-pass instance for the magnitude scalar path. */
    arm_biquad_cascade_df1_init_f32(&bp_inst,
                                    (uint8_t)BP_NUM_STAGES,
                                    bp_coeffs,
                                    bp_state);

    /* Reset accumulators and counters. */
    s_vib_sum_sq    = 0.0f;
    s_quake_sum_sq  = 0.0f;
    s_window_cnt    = 0U;
    s_total_samples = 0U;
    s_window_ready  = 0U;
}

void ImuDsp_FeedSample(float ax_mg, float ay_mg, float az_mg)
{
    /* ------------------------------------------------------------------
     * 1. Per-axis high-pass filter (blockSize = 1 sample at a time).
     *    CMSIS processes one sample by passing blockSize=1.
     *    Output: linear acceleration in mg after gravity removal.
     * ------------------------------------------------------------------ */
    float hx, hy, hz;
    arm_biquad_cascade_df1_f32(&hp_inst_x, &ax_mg, &hx, 1U);
    arm_biquad_cascade_df1_f32(&hp_inst_y, &ay_mg, &hy, 1U);
    arm_biquad_cascade_df1_f32(&hp_inst_z, &az_mg, &hz, 1U);

    /* ------------------------------------------------------------------
     * 2. Linear acceleration magnitude (mg).
     * ------------------------------------------------------------------ */
    float mag = sqrtf(hx*hx + hy*hy + hz*hz);

    /* ------------------------------------------------------------------
     * 3. Vibration path: accumulate mag^2.
     * ------------------------------------------------------------------ */
    s_vib_sum_sq += mag * mag;

    /* ------------------------------------------------------------------
     * 4. Quake band-pass path: filter magnitude scalar, accumulate square.
     * ------------------------------------------------------------------ */
    float band_sample;
    arm_biquad_cascade_df1_f32(&bp_inst, &mag, &band_sample, 1U);
    s_quake_sum_sq += band_sample * band_sample;

    /* ------------------------------------------------------------------
     * 5. Advance counters.
     * ------------------------------------------------------------------ */
    s_window_cnt++;
    s_total_samples++;

    /* ------------------------------------------------------------------
     * 6. Window completion: compute RMS, latch result, reset accumulators.
     * ------------------------------------------------------------------ */
    if (s_window_cnt >= WINDOW_SIZE) {
        float inv_n = 1.0f / (float)WINDOW_SIZE;

        s_result.vib_rms_mg   = sqrtf(s_vib_sum_sq  * inv_n);
        s_result.quake_rms_mg = sqrtf(s_quake_sum_sq * inv_n);
        s_result.warmed_up    = (s_total_samples >= WARMUP_SAMPLES) ? 1U : 0U;

        /* Reset accumulators for the next window. */
        s_vib_sum_sq   = 0.0f;
        s_quake_sum_sq = 0.0f;
        s_window_cnt   = 0U;

        /* Signal that a result is available. */
        s_window_ready = 1U;
    }
}

int ImuDsp_WindowReady(ImuDspResult *out)
{
    if (!s_window_ready) {
        return 0;
    }

    *out = s_result;
    s_window_ready = 0U;
    return 1;
}
