#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "sys_ctrl.h"
#include "app_dbg.h"

#include "arm_math.h"
#include "arm_const_structs.h"

#include "anomal_detect.h"
#include "model/anomal_detection_v2.h"

// #define DBG
#define PI                      (3.14159265358979f)
#define AXES                    (3)
#define SCALE_AXES              (0.00010017430721f)
#define FILTER_CUTOFF           (26.05078125f)
#define SAMPLING_FREQ           (57.0f)
#define RAW_SAMPLES_PER_AXIS    (57)
#define FFT_OVERLAP             (0)
#define STRIDE                  (FFT_LENGTH - FFT_OVERLAP)
#define NUM_FEATURES_PER_AXIS   (FEATURES_PER_AXIS)

static const float NORM_MEAN[FEATURE_LEN] = { 0.0586f, 0.4755f, 5.5409f, 3.7967f, 14.5859f, -4.1574f, -4.7556f, -5.2411f, -5.7620f, -6.0528f, -6.1612f, -6.1891f, -6.3468f, -6.5078f, -6.5926f, -6.6836f, -6.7816f, -6.9130f, -6.8785f, -7.0390f, -7.2208f, -7.2761f, -7.2970f, -7.3757f, -7.5338f, -7.4643f, -7.4924f, -7.5741f, -7.5480f, -7.4518f, -7.3385f, -7.1376f, -6.7649f, -6.5661f, 0.0468f, 0.5964f, 3.4902f, 3.7023f, 13.7243f, -4.4533f, -4.6054f, -5.1303f, -5.6474f, -6.1638f, -6.2421f, -6.4006f, -6.8070f, -7.0178f, -7.0479f, -7.0998f, -7.3005f, -7.3457f, -7.3843f, -7.5963f, -7.6530f, -7.8540f, -7.8956f, -7.9056f, -8.0587f, -8.0023f, -7.8959f, -7.9568f, -7.9806f, -7.9766f, -7.9248f, -7.7791f, -7.7007f, -7.2788f, 0.1006f, -1.0255f, 9.4645f, 3.5938f, 12.7701f, -3.9725f, -4.3019f, -4.7267f, -5.3239f, -5.8861f, -5.9206f, -6.2117f, -6.5931f, -6.5662f, -6.6677f, -6.7609f, -6.9208f, -7.1089f, -7.0672f, -7.1649f, -7.3698f, -7.5621f, -7.7264f, -7.7001f, -7.8121f, -7.6266f, -7.7250f, -7.5851f, -7.5133f, -7.2930f, -7.0213f, -6.3920f, -5.4805f, -4.9888f };
static const float NORM_SCALE[FEATURE_LEN] = { 14.708091f, 0.573255f, 0.088403f, 1.158824f, 0.151946f, 0.669880f, 0.549284f, 0.617122f, 0.642359f, 0.633695f, 0.639974f, 0.699160f, 0.697252f, 0.647308f, 0.709952f, 0.773242f, 0.797247f, 0.705199f, 0.700342f, 0.770289f, 0.755161f, 0.753013f, 0.816021f, 0.801354f, 0.844577f, 0.848116f, 0.896805f, 0.968984f, 0.983985f, 1.130556f, 1.195234f, 1.329832f, 1.207803f, 0.958018f, 22.611515f, 0.775973f, 0.119119f, 1.145208f, 0.158521f, 0.530676f, 0.537879f, 0.602081f, 0.562465f, 0.594556f, 0.743569f, 0.793748f, 0.762398f, 0.712486f, 0.731755f, 0.764945f, 0.732312f, 0.763746f, 0.825320f, 0.819362f, 0.874816f, 0.868082f, 0.881688f, 0.883158f, 0.881275f, 0.938444f, 0.890540f, 0.944989f, 0.919841f, 1.033426f, 1.027562f, 1.205039f, 1.362809f, 1.850884f, 14.332201f, 0.538433f, 0.089410f, 1.338363f, 0.186132f, 0.538197f, 0.471324f, 0.526041f, 0.535097f, 0.529974f, 0.621606f, 0.646007f, 0.634009f, 0.730180f, 0.658079f, 0.682425f, 0.670333f, 0.644999f, 0.703918f, 0.761508f, 0.824990f, 0.738121f, 0.842310f, 0.834394f, 0.869348f, 1.008436f, 0.925600f, 0.980863f, 1.092788f, 1.196260f, 1.163300f, 1.383946f, 1.361114f, 1.356529f };

static const float BIQUAD_COEFFS_DF2T[3][5] = {
    {5.923946358216797e-01f, 1.184789271643359e+00f, 5.923946358216797e-01f, -1.532692598430516e+00f, -5.902995029924591e-01f},
    {1.000000000000000e+00f, 2.000000000000000e+00f, 1.000000000000000e+00f, -1.621707230405852e+00f, -6.826597878495740e-01f},
    {1.000000000000000e+00f, 2.000000000000000e+00f, 1.000000000000000e+00f, -1.803084587305347e+00f, -8.708542900131004e-01f},
};
#ifdef DBG
static const char *feat_name[NUM_FEATURES_PER_AXIS] = {
    "RMS", "Skewness", "Kurtosis", "Spec_Skew", "Spec_Kurt",
    "LogBin01", "LogBin02", "LogBin03", "LogBin04", "LogBin05",
    "LogBin06", "LogBin07", "LogBin08", "LogBin09", "LogBin10",
    "LogBin11", "LogBin12", "LogBin13", "LogBin14", "LogBin15",
    "LogBin16", "LogBin17", "LogBin18", "LogBin19", "LogBin20",
    "LogBin21", "LogBin22", "LogBin23", "LogBin24", "LogBin25",
    "LogBin26", "LogBin27", "LogBin28", "LogBin29"};
static const char *axis_name[AXES] = {"X", "Y", "Z"};
#endif
static const char *label[MAX_PREDICT_CLASS] = {"Down", "Idle", "Left", "Right", "Unknown", "Up"};

/*
 * Single scratch buffer (512B) shared across the entire DSP pipeline.
 * Lifetime phases per extract_feature call:
 *   1. S[0->56]  = deinterleaved axis signal (written by caller)
 *   2. S[0->56]  = filtered signal (biquad in-place)
 *   3. S[0->127] = FFT complex buffer (window+interleave in reverse order)
 *   4. S[0->32]  = PSD bins (computed from |FFT|², overwrites low indices)
 *   5. S[0->5]   = model output (during inference)
 */
static float S[2 * FFT_LENGTH];
static AnomalyConfidence_t confidence;

static void welch_max_hold_inplace(uint32_t n)
{
    float win_sum_sq = 0.0f;
    for (int i = 0; i < FFT_LENGTH; i++)
    {
        float w = 0.5f * (1.0f - cosf(2.0f * PI * i / FFT_LENGTH));
        win_sum_sq += w * w;
    }

    for (int i = FFT_LENGTH - 1; i >= 0; i--)
    {
        float w = 0.5f * (1.0f - cosf(2.0f * PI * i / FFT_LENGTH));
        S[2 * i]     = (i < (int)n) ? S[i] * w : 0.0f;
        S[2 * i + 1] = 0.0f;
    }

    arm_cfft_f32(&arm_cfft_sR_f32_len64, S, 0, 1);

    for (int k = 0; k < NUM_BINS; k++)
    {
        float re = S[2 * k];
        float im = S[2 * k + 1];
        S[k] = (re * re + im * im) / (FFT_LENGTH * win_sum_sq);
    }
}

static void extract_axis_features(uint32_t n, float state[3][2], float *out)
{
    /* Butterworth lowpass filter (3 stages, order 6) — in-place on S */
    arm_biquad_cascade_df2T_instance_f32 biquad;
    arm_biquad_cascade_df2T_init_f32(&biquad, 3, (float32_t *)BIQUAD_COEFFS_DF2T, (float32_t *)state);
    arm_biquad_cascade_df2T_f32(&biquad, S, S, n);

    /* Subtract mean — in-place on S */
    float mean_val = 0.0f;
    arm_mean_f32(S, n, &mean_val);
    arm_offset_f32(S, -mean_val, S, n);

    /* Time-domain stats from S[0..n-1] */
    float sum_sq = 0.0f;
    float sum_cube = 0.0f;
    float sum_four = 0.0f;
    for (uint32_t i = 0; i < n; i++)
    {
        float v = S[i];
        sum_sq += v * v;
        sum_cube += v * v * v;
        sum_four += v * v * v * v;
    }

    float rms = sqrtf(sum_sq / n);
    float variance = sum_sq / n;
    float std_val = (variance > 0.0f) ? sqrtf(variance) : 1e-10f;
    float std3 = std_val * std_val * std_val;
    float std4 = std3 * std_val;
    float skewness = (std3 > 0.0f) ? (sum_cube / n) / std3 : 0.0f;
    float kurtosis = (std4 > 0.0f) ? (sum_four / n) / std4 - 3.0f : -3.0f;

    /* Welch max-hold: S[0..56] → S[0..32] = PSD */
    welch_max_hold_inplace(n);

    /* Spectrum skewness and kurtosis from S[0..32] */
    float psd_mean = 0.0f;
    float psd_var = 0.0f;
    float spec_skew_num = 0.0f;
    float spec_kurt_num = 0.0f;
    for (int k = 0; k < NUM_BINS; k++)
    {
        float v = (S[k] > 1e-10f) ? S[k] : 1e-10f;
        S[k] = v; /* replace in-place for log10 pass */
        psd_mean += v;
    }
    psd_mean /= NUM_BINS;
    for (int k = 0; k < NUM_BINS; k++)
    {
        float d = S[k] - psd_mean;
        psd_var += d * d;
        spec_skew_num += d * d * d;
        spec_kurt_num += d * d * d * d;
    }
    psd_var /= NUM_BINS;
    spec_skew_num /= NUM_BINS;
    spec_kurt_num /= NUM_BINS;

    float psd_std = (psd_var > 0.0f) ? sqrtf(psd_var) : 1e-10f;
    float spec_skew = spec_skew_num / (psd_std * psd_std * psd_std + 1e-10f);
    float spec_kurt = spec_kurt_num / (psd_var * psd_var + 1e-10f) - 3.0f;

    /* Log10 of bins [1..29] */
    float cutoff_hz = FILTER_CUTOFF;
    float bin_resolution = SAMPLING_FREQ / FFT_LENGTH;
    int stop_bin = (int)(cutoff_hz / bin_resolution + 0.5f) + 1;
    if (stop_bin > NUM_BINS) stop_bin = NUM_BINS;
    int start_bin = 1;
    int num_bins_used = stop_bin - start_bin;

    out[RMS] = rms;
    out[Skewness] = skewness;
    out[Kurtosis] = kurtosis;
    out[Spec_Skew] = spec_skew;
    out[Spec_Kurt] = spec_kurt;

    for (int i = 0; i < num_bins_used; i++)
    {
        out[5 + i] = log10f(S[start_bin + i]);
    }
    for (int i = num_bins_used; i < 29; i++)
    {
        out[5 + i] = 0.0f;
    }
}

AnomalyInfer::AnomalyInfer()
{
    for (int i = 0; i < FEATURE_LEN; i++)
    {
        features[i] = 0.0f;
    }
    memset(filter_state, 0, sizeof(filter_state));
}

AnomalyInfer::~AnomalyInfer()
{
}

int AnomalyInfer::extract_feature(void *data, uint32_t len)
{
    memset(filter_state, 0, sizeof(filter_state));
    if (len != RAW_SAMPLES_PER_AXIS)
    {
        APP_DBG("Expected %d samples, got %d\n", RAW_SAMPLES_PER_AXIS, len);
        return -1;
    }

    float *raw = (float *)(data);

    for (int a = 0; a < AXES; a++)
    {
        /* Deinterleave axis a directly into S[0..56] — no axis_buf needed */
        for (uint32_t i = 0; i < len; i++)
        {
            S[i] = raw[i * AXES + a] * SCALE_AXES;
        }

        /* Extract features, write directly to features[] — no feat_per_axis needed */
        extract_axis_features(len, filter_state[a], &features[a * NUM_FEATURES_PER_AXIS]);
    }

    return 0;
}

int AnomalyInfer::inference(void *data, uint32_t len)
{
    return inference(data, len, NULL, 0);
}

int AnomalyInfer::inference(void *data, uint32_t len, float *output, uint32_t output_len) 
{
    uint32_t prev = sys_ctrl_millis();

    if (extract_feature(data, len) != 0)
    {
        return -1;
    }

    APP_DBG("- Anomaly Features:\n");
#if 0
    for (int axis = 0; axis < AXES; axis++)
    {
        APP_DBG("[%s] ", axis_name[axis]);
        for (int feature = 0; feature < 5; feature++)
        {
            xfprintf((void (*)(int))sys_ctrl_shell_put_char, "%s=%.4f ", feat_name[feature], features[axis * NUM_FEATURES_PER_AXIS + feature]);
        }
        xfprintf((void (*)(int))sys_ctrl_shell_put_char,"\n");
    }
#endif

    /* Normalize in-place into features[] — no norm_features[] needed */
    for (int i = 0; i < FEATURE_LEN; i++)
    {
        features[i] = (features[i] - NORM_MEAN[i]) * NORM_SCALE[i];
    }

    /* Model output to S[0..5] — no out[] needed */
    if (anomaly_model_regress(features, FEATURE_LEN, S, MAX_PREDICT_CLASS) != EmlOk)
    {
        return -1;
    }
    if (output != NULL)
    {
        for (int i = 0; i < output_len; i++)
        {
            ((float *)output)[i] = S[i];
        }
    }

    int predicted = -1;
    float max_prob = -1.0f;

    for (int i = 0; i < MAX_PREDICT_CLASS; i++)
    {
        if (S[i] >= confidence.confidence[i] &&
            S[i] > max_prob)
        {
            max_prob = S[i];
            predicted = i;
        }
    }


    APP_DBG("Pointer [%08X]\n", (unsigned int)this);
    APP_DBG("\tP(down)= %.3f\n", S[0]);
    APP_DBG("\tP(idle)= %.3f\n", S[1]);
    APP_DBG("\tP(left)= %.3f\n", S[2]);
    APP_DBG("\tP(right)= %.3f\n", S[3]);
    APP_DBG("\tP(unknown)= %.3f\n", S[4]);
    APP_DBG("\tP(up)= %.3f\n", S[5]);
    APP_DBG("----> Predict= %s\n\n", predicted == -1 ? "None" : label[predicted]);
    return predicted;
}

int AnomalyInfer::getMaxPredictClass() {
    return MAX_PREDICT_CLASS;
}

int AnomalyInfer::setConfidence(AnomalyConfidence_t conf) {
    confidence = conf;
    return 0;
}