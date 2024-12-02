# SLAM

## Table of Contents
- [Hardware Setup for Experiment](#0️⃣-hardware-setup-for-experiment)
- [Software Requirements](#1️⃣-software-requirements)
- [Map Generation and Optimization](#2️⃣-map-generation-and-optimization)
- [Data Collection and Path Planning](#3️⃣-data-collection-and-path-planning)
- [Code Execution Instructions](#4️⃣-code-execution-instructions)


## Introduction

Visual SLAM (Simultaneous Localization and Mapping) enables vehicles to create maps of unknown environments while tracking their location. We used RTAB-Map with a ZED 2i stereo camera for the self-driving system, integrating V-SLAM, real-time pose tracking via ROS topics, and a fine-tuned navigation system. Parameters like steer gain, bearing difference, heading, speed, turning factor, and steer output ensure smooth autonomous navigation. This setup opens possibilities for safer, more efficient self-driving cars.

#### Weather Impact on Luminosity:

  - **Cloudy and Sunny:**
    - 3 AM - 5 AM: 110-130 units
    - 10 AM - 1 PM: 90-100 units
    - 3 PM - 5 PM: 100-115 units
  - **Rainy:** 95-170 units

Below is the image of the SLAM architecture:

<br>

<img src="https://github.com/nineRishav/Solio-S186/blob/master/media/SLAM-architecture.png" alt="Architecture">
    
## 0️⃣ Hardware Setup for Experiment

The setup includes a ZED Camera positioned at the top and connected to a GPU via a USB connection. The GPU is integrated with several other components: it connects to a Micro-Auto Box via an Ethernet cable and to a Computing Platform Work Station through an unspecified type of connection. Additionally, a monitor is linked to the GPU via a DisplayPort (DP) cable and is connected to a battery-operated Uninterruptible Power Supply (UPS) for power backup. The Input/Output devices, including a mouse and keyboard, are connected to the Computing Platform Work Station.

Below is the image of the SLAM Hardware architecture:

<img src="https://github.com/nineRishav/Solio-S186/blob/master/media/SLAM-Hardware.png" alt="Architecture">


## 1️⃣ Software Requirements

RTAB-Map (Real-Time Appearance-Based Mapping) is an open-source Visual SLAM (Simultaneous Localization and Mapping) library designed for 3D environment mapping and localization. It uses a  ZED Camera, to capture the visual and depth information necessary for creating detailed maps of the surroundings.

  - **System Dependencies**
    
    - If you have **ROS** (Robot Operating System) already installed, the dependencies should be satisfied. You can follow the instructions on the [rtabmap_ros](https://github.com/introlab/rtabmap_ros) page
    
    - If ROS is **not** installed, you’ll need to install the following system dependencies based on your Ubuntu version:
    
    ```bash
    sudo apt-get update
    sudo apt-get install libsqlite3-dev libpcl-dev libopencv-dev git cmake libproj-dev libqt5svg5-dev
    ```
    - Install [zed-ros-wrapper](https://github.com/stereolabs/zed-ros-wrapper)

- **Download and Build RTAB-Map:**

  - Download the latest source from [GitHub](https://github.com/introlab/rtabmap_ros).
  - Unzip the downloaded file and navigate to the `rtabmap/build` directory:
    
    ```bash
    cd rtabmap/build
    ```
  - Build RTAB-Map:

    ```bash
    cmake ..
    make -j4
    sudo make install
    ```
- **Optional Dependencies:**
  - **SURF/SIFT Features:** To enable SURF/SIFT features in RTAB-Map, build OpenCV from source with nonfree module support.
  - **Graph Optimization Libraries:** Install g2o and GTSAM (version >= 4) if needed.

- **Prerequisites**
  - **OS:** Ubuntu 20.04
  - **ZED SDK Version:** 4.1 (CUDA 12.1)
    

## 2️⃣ Map Generation and optimization

The Stereo Vision Camera captures stereo images and provides visual data to the SLAM system. The RTABMAP SLAM module performs the following tasks: 

**Visual Odometry:** Estimates the pose (position and orientation) of the camera/vehicle by tracking visual features in the images. 

**3D Mapping:** Triangulates the 3D positions of visual features from the stereo images and constructs a point cloud map of the environment.

**IMU Integration:** Fuses data from the Inertial Measurement Unit (IMU) with the visual data to improve the accuracy and robustness of pose estimation and mapping. The Autonomous Actuation module subscribes to the localization poses estimated by RTABMAP SLAM, performs path planning, and generates control commands for the vehicle. The Vehicle executes the autonomous motion based on the control commands received from the Autonomous Actuation module.


## 3️⃣ Data Collection and Path planning

**Data Collection:** The ZED-ROS-Examples toolkit was utilized to record ROS bags, enabling the capture of data from various sensors, including the ZED stereo camera. The recorded ROS bags contain a comprehensive dataset with diverse topics, such as raw data, depth information, RGB images from both cameras, IMU data (orientation and acceleration), pose data, and point cloud data. 

**Data Analysis:** The RTABMAP simulation was employed for mapping the environment using the collected data. The primary objectives of the data analysis include: 
  - Evaluating the accuracy of the generated maps. 
  - Assessing the performance of localization within the mapped environment.
  - Identifying potential limitations or areas for improvement.
  - Validating the robustness and reliability of the overall system.

**Successful Map Creation:** The system demonstrated the ability to create highly detailed and accurate 3D maps of the test environment. These maps accurately captured the layout, contours, and intricate details of the surroundings.

**Precise Localization:** The system exhibited remarkable precision in localizing the vehicle's position within the mapped environment. The localization data obtained from the RTABMAP SLAM algorithm proved highly accurate and reliable. 

**Autonomous Actuation:** A custom code was developed and seamlessly integrated with the SLAM system, enabling autonomous actuation of the vehicle. This code bridged the gap between the SLAM algorithm localization data and the vehicle's control system. Through this integration, the system achieved successful autonomous navigation of the vehicle within the mapped environment.
  
  - Successful real-time creation of detailed 3D maps 
  - Accurate representation of the environment's layout and contours 
  - Precise localization within the mapped environment Integration with custom code for autonomous vehicle actuation

## 4️⃣ Code Execution Instructions

### Simulation and Mapping
- **Step:1** Create a new folder with desired folder name in the exisiting VSLAM folder

 - **Step:2** Open the terminal in the created folder and enter following command to initialize the camera
```bash
roslaunch zed_wrapper zed2i.launch
```
 - **Step:3** For making the map, record the camera data in rosbag file, open a new terminal tab and enter following command
```bash
rosbag record -a
```
   **debug** if the launch terminal starts showing errors after running recording step stop the launch terminal and restart with recording terminal already running.
-  **Step:4** Rename the recorded rosbag file to ```1.bag```
  
### Localization

-  **Step:1** Open terminal in the recorded file folder
-  **Step:2** For opening rviz
```bash
roslaunch rtabmap_launch rtabmap.launch    rtabmap_args:="--Vis/CorFlowMaxLevel 35 --Stereo/MaxDisparity 200 --Mem/ImagePreDecimation 2 --Mem/ImagePostDecimation 2 --RGBD/OptimizeMaxError 07.0 --Rtabmap/LoopThr 0.0014"   stereo_namespace:=/zed2i/zed_node    right_image_topic:=/zed2i/zed_node/right/image_rect_gray    stereo:=true    visual_odometry:=true    frame_id:=base_link    imu_topic:=/zed2i/zed_node/imu/data    wait_imu_to_init:=true use_sim_time:=true \ localization:=true
```
 -  **Step:3** Convert rosbag file to generate ```3db.txt``` file
```bash
rostopic echo /rtabmap/localization_pose | tee -a 3db.txt
```
**debug** if showing clock is published? in the step 2 command modify the argument as <use_sim_time:=false> then run step 3.
-  **Step:4** Play rosbag file in other terminal tab along with step 3.
```bash
rosbag play --clock 1.bag
```
-  **Step:5** For generating waypoints run
```bash
python waypoints.py
```
Above file will use ```3db.txt```file to generate waypoints with x and y coordinates

-  **Step:6** Above step will generate ```lastwp.txt```. Copy the path of this file in ```SLAMnavigation.py```

    
### Vehicle Actuation

  - **Step:1** For launching camera

```bash
roslaunch zed_wrapper zed2i.launch
```

  - **Step:2** For launching visualization through rviz
```bash
roslaunch rtabmap_launch rtabmap.launch    rtabmap_args:="--Vis/CorFlowMaxLevel 35 --Stereo/MaxDisparity 200 --Mem/ImagePreDecimation 2 --Mem/ImagePostDecimation 2 --RGBD/OptimizeMaxError 07.0 --Rtabmap/LoopThr 0.0014"   stereo_namespace:=/zed2i/zed_node    right_image_topic:=/zed2i/zed_node/right/image_rect_gray    stereo:=true    visual_odometry:=true    frame_id:=base_link    imu_topic:=/zed2i/zed_node/imu/data    wait_imu_to_init:=true localization:=true
```

  - **Step:3** Start navigation
```bash
python SLAMnavigation.py
``` 


    
    
