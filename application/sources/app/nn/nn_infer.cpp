#include "app_dbg.h"

#include "nn_infer.h"
#include "impact_detect.h"
NNInfer::NNInfer(enum eModelName model) {
    modelName = model;
    switch (model)
    {
    case ImpactDetect:
        infer = new ImpactInfer();
        break;
    
    default:
        break;
    }
    APP_DBG("Init NN for model %d, %08X\n", model, (unsigned int)infer);
}

NNInfer::~NNInfer() {
    APP_DBG("Free NN: %08X\n", (unsigned int)infer);
    if (infer) {
        ImpactInfer *p = static_cast<ImpactInfer*>(infer);
        delete p;
    }
}

const void* NNInfer::getInfer() {
    return infer;
}

int NNInfer::inference(void *data, uint32_t len) {
    switch (modelName) {
    case ImpactDetect:
        return ((ImpactInfer*)infer)->inference(data, len);
    
    default:
        break;
    }
    return -1;
}