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

static const float NORM_MEAN[FEATURE_LEN] = {
    0.0707f, 0.1017f, 0.6336f, 3.9594f, 15.8577f,
    -3.7143f, -4.0793f, -4.6231f, -5.1803f, -5.3968f, -5.5582f, -5.7482f,
    -5.7908f, -5.9411f, -6.0246f, -6.0436f, -6.1926f, -6.2183f, -6.1653f,
    -6.3814f, -6.5963f, -6.6439f, -6.8235f, -6.8305f, -6.9601f, -7.0049f,
    -7.1010f, -7.2648f, -7.3434f, -7.3639f, -7.1972f, -7.2405f, -6.9482f,
    -6.8395f,
    0.0584f, 0.1352f, 0.2814f, 3.7832f, 14.1806f,
    -3.7026f, -3.8287f, -4.4132f, -4.9772f, -5.5085f, -5.8884f, -5.9728f,
    -6.3200f, -6.5015f, -6.5674f, -6.5716f, -6.7420f, -6.7617f, -6.9731f,
    -7.0831f, -7.1272f, -7.4455f, -7.3209f, -7.4046f, -7.6022f, -7.6023f,
    -7.4591f, -7.4570f, -7.3593f, -7.5079f, -7.5350f, -7.4568f, -7.3711f,
    -7.3319f,
    0.1133f, -1.2058f, 6.7897f, 3.5335f, 12.3262f,
    -3.3633f, -3.6373f, -4.0590f, -4.6990f, -5.1845f, -5.4298f, -5.5962f,
    -5.9230f, -6.1712f, -6.2075f, -6.2415f, -6.4065f, -6.4954f, -6.4745f,
    -6.6841f, -6.9052f, -6.9925f, -7.3977f, -7.3509f, -7.4470f, -7.3698f,
    -7.3194f, -7.2180f, -7.2401f, -6.9366f, -6.6894f, -6.1391f, -5.2121f,
    -4.7022f
};
static const float NORM_SCALE[FEATURE_LEN] = {
    11.064430f, 1.044138f, 0.162903f, 1.105054f, 0.143653f,
    0.725056f, 0.763452f, 0.886557f, 0.762563f, 0.827567f, 0.923312f,
    0.861393f, 0.915608f, 0.736982f, 0.935255f, 1.048592f, 1.052417f,
    0.941605f, 0.986303f, 1.056883f, 0.918963f, 0.979220f, 0.884975f,
    0.942399f, 0.975708f, 1.127804f, 0.982284f, 1.054070f, 0.948902f,
    0.981468f, 1.005593f, 1.230760f, 1.806150f, 1.425285f,
    23.150253f, 1.253735f, 0.194856f, 1.226376f, 0.162881f,
    0.766996f, 0.843464f, 0.910259f, 0.880701f, 0.939269f, 1.130243f,
    1.040625f, 1.090283f, 1.039026f, 0.993515f, 1.162598f, 0.982409f,
    1.159441f, 1.157491f, 1.334299f, 1.279046f, 1.017169f, 1.220270f,
    1.422691f, 1.238375f, 1.270664f, 1.190924f, 1.284204f, 1.206082f,
    1.318616f, 1.078118f, 1.313829f, 1.704266f, 1.594878f,
    13.874351f, 0.814297f, 0.105385f, 1.166981f, 0.167863f,
    0.822398f, 0.659565f, 0.770936f, 0.710296f, 0.784725f, 0.936008f,
    0.969536f, 0.928511f, 0.940586f, 0.825773f, 0.969229f, 0.874957f,
    0.850875f, 1.031511f, 1.050176f, 1.227913f, 1.108558f, 0.989724f,
    1.036359f, 1.187044f, 1.160907f, 1.065947f, 1.324319f, 1.094280f,
    1.908193f, 2.726906f, 3.405504f, 10.027916f, 19.007414f
};

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
    for (int i = 0; i < MAX_PREDICT_CLASS; i++)
    {
        confidence.confidence[i] = 0.0f;
    }
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