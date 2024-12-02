# TensorRT

## Table of Contents
- [Step by Step Export Process](#1️⃣-step-by-step-export-process)
- [Code Execution Instructions](#2️⃣-code-execution-instructions)

  
## Introduction

TensorRT (NVIDIA TensorRT) is a high-performance deep learning inference library developed by NVIDIA. It optimizes and accelerates deep learning models for production deployment by:
  - **Layer Fusion:** Combines operations to reduce memory usage.
  - **Precision Calibration:** Uses lower precision (FP16/INT8) for faster computation.
  - **Kernel Auto-Tuning:** Selects optimal kernels for hardware.
  - **Dynamic Tensor Memory:** Manages memory efficiently to boost performance.
    
## 1️⃣ Step by Step Export Process
  - Install TensorRT and ONNX dependencies
  - Load the PyTorch model.
  - Define the desired parameters according to your requirements, including precision mode, image dimensions, workspace size, and other relevant configurations.
  - Convert the model to ONNX format.
  - Convert the ONNX model to a TensorRT engine.
  - Load and run the TensorRT engine for inference.
    
### CLI Command to export the ‘.pt files’ into TensorRT ‘.engine files’

```bash
yolo export model=<path_of_the_model> format=engine half=True device=0 workspace=12 imgsz=640
```

### Inference Time Comparison for Single and Multimodal Models

| Model                       | Without TensorRT    | With TensorRT        |
|-----------------------------|---------------------|----------------------|
| **Inference Time for Single Model** | 22 ms to 25 ms       | 5 ms to 10 ms         |
| **Inference Time for Multimodal**   | 75 ms to 85 ms       | 10 ms to 30 ms        |



## 2️⃣ Code Execution Instructions

Replace the  `.pt files` with `.engine files` and follow the steps

First, run the Perception file followed by the Navigation code.

```bash
sudo -S chmod 777 /dev/ttyUSB       # for giving the permission to the GNSS
roslaunch novatel_oem7_driver oem7_tty.launch oem7_tty_name:=/dev/ttyUSB0       # To do the roslaunch and start GNSS receiver
python P1-perception.py
python N1-navigation.py
```
If you don't want to run the Navigation code.

```bash
roscore
python P1-perception.py
```









