# Motion Direct Classification

## 1. Overview
<div align="center">

![OverviewImage](image/title.png)

**Figure 1:** System block diagram - STM32L151 MCU connected to ICM-20948 IMU, 3-axis accelerometer data processed through DSP pipeline and classified by neural network

</div>

This system classifies motion using the ICM-20948 (9-DoF) sensor on an STM32L151 microcontroller. 3-axis accelerometer data (X, Y, Z) is collected, processed through a DSP pipeline, and fed into a small fully-connected neural network to classify 6 motion states.

### Why I choose **Machine Learing** instead of **Rule-Based Algorithm**
Machine Learning was chosen for this project because it can learn complex motion patterns from sensor data, providing higher classification accuracy, better robustness against variations, and easier scalability than traditional rule-based algorithms. Combined with DSP-based feature extraction and TinyML deployment, the solution enables efficient real-time motion classification on resource-constrained embedded devices.

## 2. Hardware

- **MCU**: STM32L151
- **IMU**: ICM-20948

## 3. Data Collection

Use the Edge Impulse Data Forwarder to stream sensor data from the device to Edge Impulse Studio:

```bash
edge-impulse-data-forwarder --serial-port /dev/ttyUSB0 --baud-rate 115200
```
or
```bash
edge-impulse-data-forwarder
```

The input is accelerometer values on all **3 axes (x, y, z)**.

<div align="center">

![Device connected successfully](image/edge-impulse-connect-device.png)

**Figure 2:** Terminal confirming successful

![Data collection UI](image/edge-impulse-collect-data.png)

**Figure 3:** Edge Impulse Studio data collection interface streaming accelerometer data from device

</div>

### Dataset

The dataset was exported from Edge Impulse located at: [Dataset](../trainning/motion_direct_classify)

<div align="center">

![Edge Impulse dataset view](image/edge-impulse-dataset.png)

**Figure 4:** Edge Impulse dataset overview with 6 motion classes: idle, left, right, up, down, unknown

</div>

It contains **6 classes**:

| Class | Label       |
|-------|-------------|
| 0     | Down        |
| 1     | Idle  |
| 2     | Left    |
| 3     | Right     |
| 4     | Unknown     |
| 5     | Up     |

## 4. DSP Pipeline

### Time-Domain Features:
- [RMS](https://en.wikipedia.org/wiki/Root_mean_square)
- [Skewness](https://en.wikipedia.org/wiki/Skewness)
- [Kurtosis](https://en.wikipedia.org/wiki/Kurtosis)
### Frequency-Domain Features:
- [FFT]((https://en.wikipedia.org/wiki/Fast_Fourier_transform)) Skewness
- [FFT]((https://en.wikipedia.org/wiki/Fast_Fourier_transform)) Kurtosis
- Log PSD

### Feature Vector Layout (34 features per axis)

| Index | Feature |
|-------|---------|
| 0     | RMS   |
| 1     | Skewness |
| 2     | Kurtosis |
| 3     | Spec_Skew |
| 4     | Spec_Kurt |
| 5-33  | Log PSD Bins (29 bins) |

**Total: 34 features/axis × 3 axes = 102 features**

### CMSIS-DSP
These CMSIS-DSP primitives run on the Cortex-M3 FPU
| Function | Purpose
|---|---|
| `arm_biquad_cascade_df2T_f32` | Applies a 26.05 Hz Butterworth low-pass filter |
| `arm_cfft_f32` | Computes the FFT of the preprocessed signal |
| `arm_mean_f32` | Computes the mean value |
| `arm_offset_f32` | Removes the DC offset by subtracting the mean from each sample before spectral analysis |

## 5. Model Architecture

Compact fully-connected neural network (FCNN):

```
Input:  102 floats (34 features/axis × 3 axes)
FC1:    20 units, ReLU              (20×102 + 20 = 2060)
FC2:    10 units, ReLU              (10×20 + 10 = 210)
FC3:    6 units, Softmax            (6×10  + 6  = 66)
Output: 6 class probabilities       Total: ~2336 floats
```

### Export C header use Emlearn
- File: [Model](../inference/motion_direct_classify/model/motion_direct_classify_model.h)
- Model contains weights + eml_net engine

## 6. Processing Flow

The system uses a **delta magnitude trigger** to detect motion in any direction, followed by a fixed-size window for inference.

#### Phase 1: Motion trigger (delta magnitude)

The system monitors the **change** in acceleration magnitude between consecutive EKF-filtered samples:

```
delta_mag = |mag_g(n) - mag_g(n-1)|
if delta_mag > 0.002g → trigger store data -> inference
```

This detects motion in **all directions** (up, down, left, right) because any movement changes the magnitude relative to the previous sample. 

#### Phase 2: Data collection

Once triggered, the ring buffer fills with 57 samples (1 second of data).

#### Phase 3: Inference

When the buffer stores 1s, the ML runs classification:

```
Buffer (1 second x 57 samples × 3 axes) → DSP Feature Extraction → Neural Network → Predicted Class
```

After inference completes, the system resets and waits for the next trigger.

```mermaid
sequenceDiagram
    participant Sensor
    participant EKF as EKF Filter
    participant Trig as Delta Trigger
    participant RB as Ring Buffer (57 samples)
    participant Task as Polling ML
    participant Infer as MotionDirectInfer::inference()
    participant DSP as Feature Extraction
    participant NN as Neural Network

    Sensor->>EKF: Raw accel (X, Y, Z)
    EKF-->>Trig: Filtered magnitude (mag_g)
    Trig->>Trig: prev_mag_g = mag_g (no trigger check)

    Note over Sensor: Monitoring
    loop Every 10ms poll
        Sensor->>EKF: Raw accel
        EKF-->>Trig: Filtered magnitude (mag_g)
        Trig->>Trig: delta = |mag_g - prev_mag_g|

        alt delta > 0.002g (motion detected)
            Note over Trig: Phase 2: Collect data
            loop 57 samples (1s)
                Sensor->>EKF: Raw accel
                EKF-->>RB: Filtered (filt_x, filt_y, filt_z)
            end

            Note over RB: Phase 3: Inference
            RB->>Task: Buffer full (57 samples)
            Task->>Infer: inference(buffer)
            Infer->>DSP: Extract 102 features
            DSP->>NN: FCNN classification
            NN-->>Task: Predicted class (idle/left/right/up/down/unknown)
            Task->>Task: Reset, wait for next trigger
        else delta <= 0.002g (no motion)
            Trig->>Trig: Continue monitoring
        end
    end
```
## 7. Loss, Accuracy, Confusion Matrix

<div align="center">

<img src="image/training_history.png" width="100%">

**Figure 5:** Training loss and accuracy

<img src="image/confusion-matrix-validation.png" width="100%">
**Figure 6:** Confusion matrix on validation set, showing per-class prediction accuracy

</div>

## 8. Video Demo

[![Demo](image/title.png)](https://github.com/user-attachments/assets/3420db6c-671e-4b4b-a058-6ce20dbc7669)

### [Configuration parameters](../trainning/Motion-Direct-Classify.ipynb)

| Parameter            | Value       | Description              |
|----------------------|-------------|--------------------------|
| axes                 | 3           | Number of axes (X, Y, Z) |
| scale_axes           | 0.00010017  | Raw data scaling factor  |
| filter_type          | low         | Filter type (lowpass)    |
| filter_cutoff        | 26.05 Hz    | Cutoff frequency         |
| filter_order         | 6           | Filter order             |
| fft_length           | 64          | FFT length               |
| do_fft_overlap       | false       | No overlap               |
| sampling_freq        | 57 Hz       | Sampling frequency       |
| raw_samples_per_axis | 57          | Samples per axis         |

## 9. Related Files

| File | Role | 
|------|------| 
| [Training Motion Direct Classify](../trainning/Motion-Direct-Classify.ipynb)   | Training pipeline |
| [Motion Direct Inference](../inference/motion_direct_classify)                        | Class header |
| [Model](../inference/motion_direct_classify/model/motion_direct_classify_model.h)       | Model weights (emlearn) |
| [Sensor](../../task_accel_sensor.cpp)                                 | ICM-20948 driver + ring buffer |

## 10. Reference
| Topic | Description |
| ----- | ----------- |
| [Emlearn](https://github.com/emlearn/emlearn) | Machine learning for microcontroller and embedded systems |
| [EdgeImpulse](https://www.edgeimpulse.com) | Collect Data |