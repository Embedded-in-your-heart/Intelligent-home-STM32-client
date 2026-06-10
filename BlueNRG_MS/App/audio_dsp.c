/**
  ******************************************************************************
  * @file    App/audio_dsp.c
  * @brief   Audio DSP: A-weighted dB level, FFT-based sound classification, and
  *          alarm-tone detection from a single 200 ms DFSDM window.
  *
  *          Called by audio_task.c once per DMA half-complete event (200 ms).
  *          Input: 1600 raw int32_t FLTRDATAR words at ~8 kHz.
  *          Calibrated sample = raw[i] >> 16  (same as audio_task.c SAMPLE_SHIFT).
  *
  *  Signal chain
  *  ============
  *  1. Convert all 1600 raw samples to float (via >> 16).
  *  2. A-weighting biquad filter (3-stage cascade DF1) → weighted_buf[1600].
  *  3. Weighted RMS → dBa via 20*log10f(max(rms,1.0)) + AUDIO_DBA_CAL_OFFSET.
  *  4. FFT path: first 1024 samples × Hanning window → arm_rfft_fast_f32 →
  *     512 magnitude bins.
  *  5. Spectral feature extraction: E_total, E_mid, peak_ratio.
  *  6. Rule-based classifier → sound_class + is_alarm_tone.
  *
  *  A-weighting derivation
  *  ======================
  *  Transfer function: H_A(s) = K * s^4 / [(s+p1)^2 * (s+p2) * (s+p3) * (s+p4)^2]
  *  IEC 61672-1 pole frequencies:
  *    p1 = 2*pi*20.598997 Hz   (double; low-frequency rolloff)
  *    p2 = 2*pi*107.65265 Hz   (single)
  *    p3 = 2*pi*737.86223 Hz   (single)
  *    p4 = 2*pi*12194.217 Hz   (double; high-frequency rolloff)
  *  Bilinear transform with K = 2*fs = 16000 rad/sample, no pre-warping
  *  (the filter is normalized to exactly 0 dB at 1 kHz post-cascade gain).
  *  3-stage DF1 biquad decomposition:
  *    Stage 1: 2 zeros at DC (s=0), double pole at p1 — low-frequency HPF shape
  *    Stage 2: 2 zeros at DC (s=0), double pole at p4 — high-frequency HPF shape
  *    Stage 3: Nyquist zeros, single poles p2 and p3  — mid-band LPF shape
  *  Overall gain normalised to 0 dB at 1 kHz (factor applied to Stage 1 b coeffs).
  *  Note: at fs=8 kHz (Nyquist=4 kHz), the bilinear stage 3 has double zeros at
  *  Nyquist, causing ~2 dB deviation above 3 kHz versus the IEC reference curve.
  *  This is a fundamental constraint at this sample rate and is acceptable given
  *  the "calibration pending" status of the level estimate.
  ******************************************************************************
  */

#include "audio_dsp.h"

#include <math.h>
#include <stdint.h>

#include "arm_math.h"

/* =========================================================================
 * Compile-time constants
 * ========================================================================= */

#define HALF_SAMPLES        1600U   /* samples per 200 ms window */
#define FFT_SIZE            1024U   /* RFFT length (must be power of 2) */
#define FFT_MAG_SIZE        (FFT_SIZE / 2U)  /* 512 magnitude bins */
#define SAMPLE_SHIFT        16      /* raw FLTRDATAR → calibrated sample (same as audio_task.c) */
#define FS                  8000.0f /* nominal sample rate (Hz) */
#define BIN_HZ              (FS / (float)FFT_SIZE)  /* 7.8125 Hz per bin */

/* Bin index helpers — round down so ranges are conservative */
#define HZ_TO_BIN(f)        ((uint32_t)((f) / BIN_HZ))

/* Spectral feature bands */
#define BIN_MID_LO          HZ_TO_BIN(300.0f)    /* 300 Hz  → bin 38  */
#define BIN_MID_HI          HZ_TO_BIN(3000.0f)   /* 3000 Hz → bin 384 */
#define BIN_PEAK_LO         HZ_TO_BIN(2500.0f)   /* 2500 Hz → bin 320 */
#define BIN_PEAK_HI         HZ_TO_BIN(3800.0f)   /* 3800 Hz → bin 486 */

/* Alarm-tone peak location (Hz) used in classifier */
#define ALARM_PEAK_LO_HZ    2800.0f
#define ALARM_PEAK_HI_HZ    3600.0f
#define ALARM_PEAK_LO_BIN   HZ_TO_BIN(ALARM_PEAK_LO_HZ)   /* bin 358 */
#define ALARM_PEAK_HI_BIN   HZ_TO_BIN(ALARM_PEAK_HI_HZ)   /* bin 460 */

/* =========================================================================
 * Classifier thresholds — all marked "calibration pending"
 * These were set by analysis of the algorithm design; validate against
 * real room conditions and adjust as needed.
 * ========================================================================= */

/** (calibration pending) MicLevel-scale RMS threshold for silence.
 *  MicLevel-scale RMS = integer_rms * MIC_SCALE_NUM; quiet baseline ~0–8.
 *  Threshold of 50 corresponds to MicLevel ~400 (well below LOUD_THRESHOLD). */
#define THRESH_QUIET_RMS        50U

/** (calibration pending) Peak-energy-ratio threshold for alarm-tone detection.
 *  peak_ratio = sum_of_mag_sq(peak_bin±1) / E_total. */
#define THRESH_ALARM_PEAK_RATIO 0.30f

/** (calibration pending) Mid-band energy fraction threshold for speech. */
#define THRESH_SPEECH_MID_RATIO 0.55f

/** (calibration pending) Clap onset ratio: current RMS must exceed
 *  previous-window RMS by this factor (4×) to be flagged as clap. */
#define THRESH_CLAP_RATIO       4.0f

/** (calibration pending) Previous-window MicLevel-scale RMS must be below
 *  this value for a clap to be declared (ensures onset from quiet/low level). */
#define THRESH_CLAP_PREV_MAX    200U

/* =========================================================================
 * A-weighting biquad coefficients (CMSIS DF1 order: b0,b1,b2,a1,a2 per stage)
 *
 * Derivation: bilinear transform of the IEC 61672-1 A-weighting analog filter
 *   H_A(s) = K * s^4 / [(s+p1)^2 * (s+p2) * (s+p3) * (s+p4)^2]
 * at fs = 8000 Hz, K = 2*fs = 16000 (bilinear warp constant).
 * Pole frequencies: p1=20.598997 Hz, p2=107.65265 Hz,
 *                   p3=737.86223 Hz, p4=12194.217 Hz.
 *
 * CMSIS sign convention: y[n] = b0*x[n]+b1*x[n-1]+b2*x[n-2]
 *                               - a1*y[n-1] - a2*y[n-2]
 * So the stored a1 = -(standard denominator z^{-1} coefficient),
 *    stored a2 = -(standard denominator z^{-2} coefficient).
 *
 * Stage 1: 2 zeros at DC, double pole at p1 (20.6 Hz); gain normalized to
 *          0 dB at 1 kHz (normalization factor 2314.5 folded into b0,b1,b2).
 * Stage 2: 2 zeros at DC, double pole at p4 (12194 Hz).
 * Stage 3: double Nyquist zero (from bilinear LP section), poles at p2+p3.
 *
 * Known limitation: Nyquist zeros in Stage 3 cause ~2 dB deviation above
 * 3 kHz from the IEC reference. Unavoidable at fs=8 kHz.
 * ========================================================================= */
#define AW_NUM_STAGES   3U

static const float32_t aw_coeffs[AW_NUM_STAGES * 5U] = {
    /* Stage 1 — zeros at DC, double pole p1=20.6 Hz; gain normalised */
    +2277.51772140f, -4555.03544280f, +2277.51772140f, +1.96790281f, -0.96816037f,
    /* Stage 2 — zeros at DC, double pole p4=12194 Hz */
    +0.02984312f,    -0.05968624f,    +0.02984312f,    -1.30899353f, -0.42836602f,
    /* Stage 3 — Nyquist zeros, poles p2=107.7 Hz and p3=737.9 Hz */
    +0.00911233f,    +0.01822465f,    +0.00911233f,    +1.46955791f, -0.50600722f,
};

/* =========================================================================
 * Static work buffers (BSS — zero-initialised, never on stack)
 * ========================================================================= */

/* Hanning window table (1024 values computed in AudioDsp_Init). */
static float32_t hanning[FFT_SIZE];

/* Float conversion of raw samples for the A-weighting path (all 1600). */
static float32_t float_buf[HALF_SAMPLES];

/* A-weighting filter output (same length as float_buf). */
static float32_t weighted_buf[HALF_SAMPLES];

/* Biquad filter state: 4 words per stage. */
static float32_t aw_state[AW_NUM_STAGES * 4U];

/* FFT input (first 1024 samples after windowing). */
static float32_t fft_in[FFT_SIZE];

/* FFT output (complex interleaved, same size as input for RFFT). */
static float32_t fft_out[FFT_SIZE];

/* Magnitude spectrum (512 bins). */
static float32_t mag[FFT_MAG_SIZE];

/* CMSIS DSP instance handles. */
static arm_rfft_fast_instance_f32 fft_inst;
static arm_biquad_casd_df1_inst_f32 aw_inst;

/* Per-module state: previous window integer RMS (MicLevel-scale, i.e. raw RMS,
 * not multiplied by MIC_SCALE_NUM).  Used by the clap detector to identify
 * sudden onset transients. Initialised to 0 (silent). */
static uint32_t s_prev_rms = 0U;

/* =========================================================================
 * Public functions
 * ========================================================================= */

void AudioDsp_Init(void)
{
    /* Build Hanning window: w[n] = 0.5 * (1 - cos(2*pi*n / (N-1))) */
    for (uint32_t n = 0U; n < FFT_SIZE; n++) {
        hanning[n] = 0.5f * (1.0f - cosf((2.0f * 3.14159265358979f * (float)n)
                                          / (float)(FFT_SIZE - 1U)));
    }

    /* Initialise CMSIS RFFT instance (1024-point real FFT). */
    (void)arm_rfft_fast_init_f32(&fft_inst, (uint16_t)FFT_SIZE);

    /* Initialise A-weighting biquad cascade (state zeroed by BSS). */
    arm_biquad_cascade_df1_init_f32(&aw_inst,
                                    (uint8_t)AW_NUM_STAGES,
                                    aw_coeffs,
                                    aw_state);
}

void AudioDsp_Process(const int32_t *raw, uint32_t n, AudioDspResult *out)
{
    /* ------------------------------------------------------------------
     * 1. Convert raw FLTRDATAR → calibrated float samples.
     *    raw >> 16 reproduces the same magnitude as audio_task.c's
     *    signed sample (FLTRDATAR sign-extended 24-bit value >> 8).
     * ------------------------------------------------------------------ */
    for (uint32_t i = 0U; i < n; i++) {
        float_buf[i] = (float32_t)(raw[i] >> SAMPLE_SHIFT);
    }

    /* ------------------------------------------------------------------
     * 2. A-weighting biquad filter applied to all 1600 samples.
     *    The filter state persists across calls so the IIR history is
     *    maintained between consecutive 200 ms windows.
     * ------------------------------------------------------------------ */
    arm_biquad_cascade_df1_f32(&aw_inst, float_buf, weighted_buf, n);

    /* ------------------------------------------------------------------
     * 3. Weighted RMS and dBa level.
     * ------------------------------------------------------------------ */
    float sum_sq = 0.0f;
    for (uint32_t i = 0U; i < n; i++) {
        sum_sq += weighted_buf[i] * weighted_buf[i];
    }
    float rms_weighted = sqrtf(sum_sq / (float)n);
    out->dba = 20.0f * log10f((rms_weighted > 1.0f) ? rms_weighted : 1.0f)
               + AUDIO_DBA_CAL_OFFSET;

    /* ------------------------------------------------------------------
     * 4. Compute integer RMS (unweighted, MicLevel-scale) for the quiet
     *    threshold and the clap detector.
     *    MicLevel-scale: same as audio_task.c  (rms of raw >> 16 samples).
     * ------------------------------------------------------------------ */
    int64_t int_sumsq = 0;
    for (uint32_t i = 0U; i < n; i++) {
        int32_t s = raw[i] >> SAMPLE_SHIFT;
        int_sumsq += (int64_t)s * (int64_t)s;
    }
    uint32_t cur_rms = (uint32_t)sqrtf((float)((double)int_sumsq / (double)n));

    /* ------------------------------------------------------------------
     * 5. FFT path: window first 1024 samples and compute magnitude spectrum.
     * ------------------------------------------------------------------ */
    for (uint32_t i = 0U; i < FFT_SIZE; i++) {
        fft_in[i] = float_buf[i] * hanning[i];
    }

    /* Forward real FFT; ifftFlag = 0 → forward transform. */
    arm_rfft_fast_f32(&fft_inst, fft_in, fft_out, 0);

    /* Compute magnitudes of the FFT_MAG_SIZE complex output pairs.
     * arm_rfft_fast_f32 packs the DC term in out[0] and the Nyquist term
     * in out[1]; the remaining pairs are at out[2..N-1] (re,im interleaved).
     * arm_cmplx_mag_f32 expects interleaved (re,im) pairs. */
    arm_cmplx_mag_f32(fft_out, mag, FFT_MAG_SIZE);

    /* ------------------------------------------------------------------
     * 6. Spectral feature extraction.
     *    Bin 0 is DC; E_total and ratios use bins 1..511 (excluding DC).
     * ------------------------------------------------------------------ */
    float E_total = 0.0f;
    float E_mid   = 0.0f;

    for (uint32_t b = 1U; b < FFT_MAG_SIZE; b++) {
        float m2 = mag[b] * mag[b];
        E_total += m2;
        if (b >= BIN_MID_LO && b <= BIN_MID_HI) {
            E_mid += m2;
        }
    }

    /* Peak search restricted to the alarm zone (2800–3600 Hz) so that a strong
     * out-of-zone component in the wider 2500–2800 Hz sub-band cannot shadow a
     * genuine alarm tone and produce a false-negative. */
    float    peak_mag2  = 0.0f;
    uint32_t peak_bin   = ALARM_PEAK_LO_BIN;
    for (uint32_t b = ALARM_PEAK_LO_BIN; b <= ALARM_PEAK_HI_BIN && b < FFT_MAG_SIZE; b++) {
        float m2 = mag[b] * mag[b];
        if (m2 > peak_mag2) {
            peak_mag2 = m2;
            peak_bin  = b;
        }
    }

    /* peak_ratio: energy of peak bin ±1 relative to total spectral energy. */
    float peak_energy = peak_mag2;
    if (peak_bin > 1U) {
        peak_energy += mag[peak_bin - 1U] * mag[peak_bin - 1U];
    }
    if (peak_bin + 1U < FFT_MAG_SIZE) {
        peak_energy += mag[peak_bin + 1U] * mag[peak_bin + 1U];
    }
    float peak_ratio = (E_total > 0.0f) ? (peak_energy / E_total) : 0.0f;

    /* Mid-band energy fraction (avoid divide-by-zero on dead silence). */
    float mid_ratio = (E_total > 0.0f) ? (E_mid / E_total) : 0.0f;

    /* Peak frequency in Hz. */
    float peak_hz = (float)peak_bin * BIN_HZ;

    /* ------------------------------------------------------------------
     * 7. Rule-based classifier.
     *    Priority order: quiet → alarm → clap → speech → other.
     *    Each constant is marked "(calibration pending)".
     * ------------------------------------------------------------------ */
    uint8_t cls;

    if (cur_rms < THRESH_QUIET_RMS) {
        /* Quiet: unweighted MicLevel-scale RMS below threshold. */
        cls = AUDIO_CLASS_QUIET;

    } else if ((peak_ratio > THRESH_ALARM_PEAK_RATIO) &&
               (peak_hz >= ALARM_PEAK_LO_HZ) &&
               (peak_hz <= ALARM_PEAK_HI_HZ)) {
        /* Alarm tone: narrow-band peak in the 2800–3600 Hz alarm zone. */
        cls = AUDIO_CLASS_ALARM;

    } else if ((s_prev_rms > 0U) &&
               ((float)cur_rms >= THRESH_CLAP_RATIO * (float)s_prev_rms) &&
               (s_prev_rms < THRESH_CLAP_PREV_MAX)) {
        /* Clap: sudden RMS onset (>=4x) from a relatively quiet prior window.
         * s_prev_rms > 0 guard avoids a spurious clap on the very first window
         * after init (where s_prev_rms is 0). */
        cls = AUDIO_CLASS_CLAP;

    } else if (mid_ratio > THRESH_SPEECH_MID_RATIO) {
        /* Speech: voice-band energy dominates. */
        cls = AUDIO_CLASS_SPEECH;

    } else {
        cls = AUDIO_CLASS_OTHER;
    }

    out->sound_class   = cls;
    out->is_alarm_tone = (cls == AUDIO_CLASS_ALARM) ? 1U : 0U;

    /* Update inter-window state for the next call. */
    s_prev_rms = cur_rms;
}
