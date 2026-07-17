#ifndef _ANOMAL_DETECT_H_
#define _ANOMAL_DETECT_H_

#include "nn_infer.h"
#include <stdint.h>

#define FEATURE_LEN (102)
#define FEATURES_PER_AXIS (34)
#define FFT_LENGTH (64)
#define NUM_BINS (FFT_LENGTH / 2 + 1)

#define MAX_PREDICT_CLASS (6)

enum FeatureType
{
    RMS = 0,
    Skewness,
    Kurtosis,
    Spec_Skew,
    Spec_Kurt,
};

typedef struct
{
    union
    {
        float confidence[MAX_PREDICT_CLASS];
        struct
        {
            float down;
            float idle;
            float left;
            float right;
            float unknown;
            float up;
        };
    };
} AnomalyConfidence_t;

class AnomalyInfer
{
private:
    float features[FEATURE_LEN];
    float filter_state[6][3][2]; // 6 stages for order-6 Butterworth
    int extract_feature(void *data, uint32_t len);

public:
    AnomalyInfer();
    ~AnomalyInfer();
    int inference(void *data, uint32_t len);
    int inference(void *data, uint32_t len, float *output, uint32_t output_len);
    int setConfidence(AnomalyConfidence_t conf);
    static int getMaxPredictClass();
};

#endif
