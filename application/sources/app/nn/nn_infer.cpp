#include "app_dbg.h"

#include "nn_infer.h"
#include "anomal_detect.h"

NNInfer::NNInfer(enum eModelName model)
{
    modelName = model;
    switch (model)
    {
    case AnomalyDetect:
        infer = new AnomalyInfer();
        break;
    default:
        break;
    }
    APP_DBG("Init NN for model %d, %08X\n", model, (unsigned int)infer);
}

NNInfer::~NNInfer()
{
    APP_DBG("Free NN: %08X\n", (unsigned int)infer);
    if (infer)
    {
        switch (modelName)
        {
        case AnomalyDetect:
        {
            AnomalyInfer *p = static_cast<AnomalyInfer *>(infer);
            delete p;
            break;
        }
        default:
            break;
        }
    }
}

const void *NNInfer::getInfer()
{
    return infer;
}

int NNInfer::inference(void *data, uint32_t len)
{
    switch (modelName)
    {
    case AnomalyDetect:
        return ((AnomalyInfer *)infer)->inference(data, len);
    default:
        break;
    }
    return -1;
}

int NNInfer::inference(void *data, uint32_t len, float *output, uint32_t output_len) {
    switch (modelName)
    {
    case AnomalyDetect:
        return ((AnomalyInfer *)infer)->inference(data, len, output, output_len);
    default:
        break;
    }
    return -1;
}

int NNInfer::getMaxPredictClass() {
    switch (modelName)
    {
    case AnomalyDetect:
        return AnomalyInfer::getMaxPredictClass();
    default:
        break;
    }
    return -1;
}