# Solio1 : Camera

## Table of Contents
- [Procedure to Run the Program](#procedure-to-run-the-program)
- [Hardware Configuration of and Connection to Orin/Laptop](#orin---computing-device)
- [Software Installation](#software-installation)
- [Perception Models for Autonomous Vehicles](#perception-models-for-autonomous-vehicles)
- [Navigation File](#navigation-file)
- [Record Waypoint for Pure Pursuit Algorithm](#record-waypoint-for-pure-pursuit-algorithm)
- [Directory Structure](#directory-structure)
- [FAQ](#FAQ)


<img src="https://github.com/nineRishav/Solio-S186/blob/master/media/vlcsnap.png" alt="TestbedImage">

## Procedure to Run the Program

First, run the Perception file followed by the Navigation code.

```bash
sudo -S chmod 777 /dev/ttyUSB*       # for giving the permission to the GNSS
roslaunch novatel_oem7_driver oem7_tty.launch oem7_tty_name:=/dev/ttyUSB0       # To do the roslaunch and start GNSS receiver
python P1-perception.py
python N1-navigation.py
```

## Orin - Computing Device
<img src="https://www.nvidia.com/content/dam/en-zz/Solutions/gtcf22/embedded-systems/jetson-orin/jetson-agx-orin-developer-kit-assets-2c50-p@2x.jpg" alt="Orin" width="500" height="300">

### Hardware Configuration of Orin

The NVIDIA Orin is a high-performance computing device designed for autonomous systems and AI applications. Follow these steps to configure the hardware for this project:

#### Power Supply
- Ensure that the Orin device is connected to a stable power supply with the appropriate voltage and current ratings as specified by NVIDIA.

#### Network Connection
- Connect the Orin device to the network via Ethernet to ensure it can communicate with other system components, such as the GNSS receiver and vehicle controller(MABX).

#### Peripheral Connections
- **GNSS Receiver:**
  - Connect the GNSS receiver to the Orin device via a serial port or USB port.

- **Camera:**
  - Connect the cameras (e.g., ZED camera) to the Orin device using USB-3

#### Display and Input Devices
- Connect a monitor to the Orin device using HDMI or DisplayPort.
- Connect a keyboard and mouse via USB for initial setup and configuration.

#### Storage : 64 GB
- Ensure that the Orin device has sufficient storage for the operating system, ROS, and other software dependencies. 

#### Memory : 32 GB 
#### Processor : ARMv8 Processor rec 1 x 12
#### Graphic Card : NVIDIA tegra Orin(nvgpu)/integrated
#### Cooling System
- Ensure that the Orin device is properly cooled using heatsinks or fans as recommended by NVIDIA to prevent overheating during operation.

```bash
sudo /usr/bin/jetson_clocks --fan
```

<img src="https://github.com/nineRishav/Solio-S186/blob/master/media/suzuki-Page-1.drawio.png" alt="IITH" width="600" height="600" />
Figure: Hardware Connection to run everything.

#### Debugging Tips: How to check if hardware connection is fine for vehicle to actuate
- GNSS + I/O device + Monitor + Yellow cable of MABX properly connected on fast blinking ORIN
- 3 green LEDs are Blinking on MABX with all tight connections
- USB of GNSS adequately connected to avoid roslaunch issue
- On the Slow start of the Autonomous vehicle, redo the cruise control

## Software Installation

- **Setup the Virtual Environment**

  - First, install the `virtualenv` package and create a new Python 3 virtual environment:
    
```bash
    sudo apt-get install  `virtualenv`
    python3 -m virtualenv -p python3 `chosen_venv_name`
```

  - Replace  `chosen_venv_name` with your desired virtual environment name.

- **Activate the Virtual Environment**

  - Next, activate the virtual environment:
    
```bash
    source `chosen_venv_name` /bin/activate
    Deactivate (for deactivating virtual environment)
```

- **ZED SDK INSTALLATION**
  
  - An internet connection is required to install the latest ZED SDK on the Nvidia AGX Orin.
  - Download the latest ZED SDK for AGX Orin (e.g. ZED SDK for L4T 34.1 Beta) from the Stereolabs website
  - Select zed sdk version according to your orin  jetpack version
  - To Check the jetpack version
    
  ```bash
     sudo apt-cache show nvidia-jetpack
  ```
  
  - Open a terminal and enter the following commands to install the SDK:
    
  ```bash
      cd ~/Downloads # replace with the correct folder if required
      chmod +x ZED_SDK*
      ./ZED_SDK_Tegra_L4T <l4t_version> _v<ZED_SDK_version>.run
  ```

  - Please replace `l4t_version` and `ZED_SDK_version` with the correct values, e.g. ZED_SDK_Tegra_L4T34.1_v3.7.4.run.
  - Follow the installer guide and answer the questions to correctly configure the ZED SDK and start to perceive the world in 3D.

  - Now that you’ve successfully set up the AGX Orin with your ZED camera

  - Reference: https://www.stereolabs.com/docs/installation/jetson/
  
- **INSTALL DESIRED VERSION OF Pytorch AND Torchvision**
    
    - **PyTorch v1.12.0**
    - JetPack 5.0 (L4T R34.1.0) / JetPack 5.0.1 (L4T R34.1.1) / JetPack 5.0.2 (L4T R35.1.0)
  ```bash
      Python 3.8 - torch-1.12.0a0+2c916ef.nv22.3-cp38-cp38-linux_aarch64.whl
      wget https://nvidia.box.com/shared/static/p57jwntv436lfrd78inwl7iml6p13fzh.whl -O torch-1.12.0a0+2c916ef.nv22.3-cp38-cp38-linux_aarch64.whl
      sudo apt-get install python3-pip libopenblas-base libopenmpi-dev libomp-dev
      pip3 install Cython
      pip3 install numpy torch-1.12.0a0+2c916ef.nv22.3-cp38-cp38-linux_aarch64.whl
  ```

    - **Torchvision v0.16.1**
    ```bash
        git clone --branch v0.16.1 https://github.com/pytorch/vision torchvision
        cd torchvision/
        export BUILD_VERSION=0.16.1
        export CUDA_HOME=/usr/local/cuda-11.4
        python3 setup.py install
    ```
    
  - Reference https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048    
    


## [Perception Models for Autonomous Vehicles](https://github.com/nineRishav/Solio-S186/blob/master/Solio1-Camera/P1_perception.py)


### Overview

This project integrates multiple perception models to enhance the situational awareness and decision-making capabilities of autonomous vehicles. The system uses a ZED camera for capturing images and depth information, and YOLO models for detecting traffic lights, traffic signs, collision warnings, speed bumps, potholes, and pedestrian intentions.

To run this file:
```bash
python P1-perception.py
```
### System Components

#### 1. ROS Node Initialization

- **Node Name**: Perception_models
- **Publishers**:
  - `Traffic_light_type`
  - `Traffic_light_depth`
  - `Traffic_sign_type`
  - `Traffic_sign_depth`
  - `CW_type`
  - `CW_depth`
  - `CW_side_dist`
  - `SB_Pothole_type`
  - `SB_Pothole_depth`
  - `/intention`

#### 2. YOLO Models

The following YOLO models are loaded onto the GPU:

- `model1`: Traffic Light Detection
- `model2`: Traffic Sign Detection
- `model3`: Collision Warning Detection
- `model4`: Speed Bump and Pothole Detection
- `model5`: Pedestrian Intention Detection

#### 3. ZED Camera Configuration

- **Resolution**: HD720
- **FPS**: 30
- **Depth Mode**: NEURAL
- **Coordinate System**: RIGHT_HANDED_Z_UP_X_FWD
- **Depth Maximum Distance**: 40 meters

#### 4. Image and Point Cloud Retrieval

- Retrieve left image and point cloud data from the ZED camera.

#### 5. Inference Execution

- Perform inference using the YOLO models in separate threads for efficiency.

#### 6. Object Detection and Tracking

- **Traffic Lights**: Detect and publish the nearest traffic light and its distance.
- **Traffic Signs**: Detect and publish the nearest traffic sign and its distance.
- **Collision Warnings**: Detect and process objects for potential collision warnings.
- **Speed Bumps and Potholes**: Detect and publish the nearest speed bump or pothole and its distance.
- **Pedestrian Intention**: Track and determine the intention of detected pedestrians (crossing, intending to cross, not crossing).

### Key Functions

#### Model Inference

- **`model_inference(frame, model)`**: Executes model inference on the provided frame.
- **`collisionWarning_inference_specific(frame, model)`**: Executes model inference for specific classes related to collision warnings.

#### Object Processing

- **`nearest_traffic_light_OR_sign(lights, depth)`**: Determines the nearest traffic light or sign.
- **`process_collision_warning(CW_objects, distance_to_object, side_distance)`**: Processes collision warnings based on detected objects and their distances.
- **`nearest_speed_bump_or_pothole(SB_Pothole_objects, distance_to_object)`**: Determines the nearest speed bump or pothole.

### Visualization

- Display the results using OpenCV windows, showing bounding boxes and labels for detected objects.

### ROS Publishers

- Publish detected objects and their distances to the respective ROS topics for further processing by the autonomous vehicle's navigation system.

### Running the Code

1. Ensure the ZED camera is connected and its serial number is set correctly.
2. Install the necessary libraries and dependencies.
3. Run the script to start the perception models and publish the detections to the ROS topics.



## [Navigation File](https://github.com/nineRishav/Solio-S186/blob/master/Solio1-Camera/N1-navigation.py)

### Autonomous Vehicle Navigation Using GNSS and Perception Data

This repository contains a Python script for navigating an autonomous vehicle using GNSS (Global Navigation Satellite System) data and perception data from various sensors. The system is designed to handle real-time navigation tasks, including collision warnings, traffic light and sign recognition, and pedestrian intention detection.

To run this file:
```bash
python N1-navigation.py
```
#### Features
- **GNSS Data Integration:** Uses GNSS data to determine vehicle position, velocity, and heading.
- **Perception Data Handling:** Processes collision warnings, traffic light and sign information, speed bumps, and pedestrian intentions.
- **Vehicle Control:** Sends control commands to the vehicle's controller via a UDP socket.
- **Logging:** Logs navigation and perception data for debugging and analysis.


#### Method Flow

Initialization
- ROS Node Setup
- Socket Setup
- Logging Setup

ROS Perception Subscribers
- Collision Warnings
- Traffic Lights and Signs
- Speed Bump & Pothole
- Pedestrian Intention

Callback Functions from GNSS
- Velocity Updates
- Heading Updates
- Latitude and Longitude Updates
- GNSS IMU Data

#### Navigation Variables and Their Functions

##### Speed and Turning Parameters

- **speed**: The base speed (in km/h) for the vehicle.
- **turning_factor**: The factor by which speed is reduced during turns.
- **STEER_GAIN**: Default gain for steering calculations. This value can change based on the bearing difference:
  - `STEER_GAIN = 250` when `abs(bearing_diff) < 1` (for very small bearing differences).
  - `STEER_GAIN = 1400` when `abs(bearing_diff) > 20` (for large bearing differences).

##### Waypoint Parameters

- **next_wp**: The number of waypoints ahead to consider when preparing for a turn.

##### Collision Warning (CW) Parameters

- **side_distance**: 1.1  
  The side distance (in meters) within which a collision warning is considered.
- **detecting_distance**: 35  
  The distance (in meters) within which collision warnings are detected.
- **stop_distance**: 10  
  The distance (in meters) within which the vehicle should stop for a collision warning.
- **caution_distance**: 20  
  The distance (in meters) within which the vehicle should slow down for a collision warning.

##### Traffic Light (TF) Parameters

- **threshold_distance**: 21  
  The distance (in meters) within which the vehicle should stop for a red traffic light.

##### Speed Bump + Pothole (SB+PT) Parameters

- **sb_threshold_distance**: 15  
  The distance (in meters) within which the vehicle should slow down for a speed bump.

## [Record waypoint for Pure Pursuit Algorithm](https://github.com/nineRishav/Solio-S186/blob/master/Solio1-Camera/waypoints/record_waypoints_solio.py)

Run the vehicle at speed 5-10 and during turning also maintain the same speed.
Run the below script under the waypoints folder 
```bash
python record_waypoints_solio.py <filename>
```

## Directory Structure

```
  
Solio1-Camera/
├── VehicleDynamics/
│   ├── Post-Processing/
|   |   └── SpeedBreaker/
|   |   └── pothole-inside/
|   |   └── .....other files
│   └── vehicle_dynamics.py
├── models_checkpoint/
├── waypoints/
├── N1-navigation.py
├── P1-perception.py
└──Segmentation_model_driving_space.py
```


## FAQ

### Q: All hardware connections are fine, but why isn't the vehicle actuating when running the navigation code?
**A:** Ensure the ORIN is blinking fast, the MABX does not show any errors, there are no ESP errors on the vehicle, and the Private CAN is ON. Additionally, start the perception system before running the navigation file to avoid any errors.

### Q: Why is the vehicle not following the designated path and moving erratically?
**A:** The vehicle's GNSS system likely needs calibration. Check the [pos_type](https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm#Position_VelocityType) status in `bestpos` topic of GNSS system.

### Q: Vehicle suddenly in the middle of path, not able to actuate properly on scenerio or going on the right path ? 
**A:** Check the `Roslaunch` on the terminal, possibly the wire was loose and rosclaunch terminal screen was terminated.
