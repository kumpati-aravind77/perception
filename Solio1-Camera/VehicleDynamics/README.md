# Vehicle Dynamics

## Table of Contents
- [Hardware Connection for this Experiment](#hardware-connection-for-this-experiment)
- [Software Installation](#Software-Installation)
- [Navigation](#1-navigation)
- [Running the Vehicle Dynamics File](#2-running-the-vehicle-dynamics-file)
- [TiHAN Testbed Route](#tihan-testbed-route)


## Introduction

This study examines the effects of road irregularities like speed bumps and potholes on vehicle dynamics and driver behavior to enhance Autonomous Vehicle (AV) systems. By using Inertial Measurement Units (IMUs) and Tightly Coupled (TC) integration methods, it identifies correlations between obstacle type, vehicle speed, and driver response times and distances. The findings suggest improvements for adaptive cruise control and AV navigation systems, showing how AVs can better mimic human reactions to boost safety and efficiency. This research uses comprehensive behavioral data to develop more adaptive and intuitive autonomous driving technologies for real-world scenarios.

## 0️⃣ Hardware Connection for this Experiment
- Orin(Computing Platform)
- GNSS(Receiver)
- RTK
- Antenna
- Micro Auto Box

<img src="https://github.com/nineRishav/Solio-S186/blob/master/media/ArchitectureVD.png" alt="Architecture">

NOTE: Switch ON the Private CAN of the vehicle to access the Speed and Steering Angle of the vehicle.

## 1️⃣ Software Installation

Can be **SKIPPED** if the Software Installation in navigation is done.

<br/>

To retrieve GNSS data, you need to install the following packages:

  - **NovAtel USB Drivers**
  - **NovAtel Application Suite**

### Step 1: Install NovAtel USB Drivers

  - **Download and Extract the USB Drivers:**
    
    - Visit the [NovAtel Software Downloads](https://novatel.com/support/support-materials/software-downloads) page and download the NovAtel USB Drivers.
    - Extract the downloaded file.
      
  - **Install the USB Drivers:**
    
    - **Method 1: GUI Installation**
      
      - Double-click the .deb file to start the installation.
        
    - **Method 2: Terminal Installation**
      
      - Open a terminal.
      - Navigate to the directory containing the `.deb` file.
      - Run the command:
        ```bash
          sudo apt install ./path_to_deb_file
        ```
      (Replace path_to_deb_file with the actual path to the .deb file).

### Step 2: Install NovAtel Application Suite

  - **Download and Extract the Application Suite:**
    
      - Download the NovAtel Application Suite from the same [NovAtel Software Downloads]((https://novatel.com/support/support-materials/software-downloads)) page.
      - Extract the downloaded file.
        
  - **Run the Application Suite**
    
    - Right-click on NovAtelApplicationSuite_64bit.AppImage
    - Select the "Run" option from the context menu.
    - The NovAtel Application Suite interface will open.
      
  - **Select USB Option**
    
    - In the application interface, select any listed USB option.
      
  - **Grant USB Permissions**
    
    - Open a terminal and run:
      ```bash
          sudo chmod 777 /dev/ttyUSB*
      ```
    
  - **Close Everything**
    
    - Close all open applications and terminal windows.

  - **Install ROS Novatel Drivers:**
    
    - To avoid the `no module named 'novatel_oem7_msgs'` error, install the ROS Novatel drivers by running the following commands in the terminal:
      ```bash
        sudo apt install ros-noetic-novatel-oem7-driver
        sudo apt install ros-noetic-novatel-*
      ```

#### Check GNSS Receiver Messages

  - **Verify Messages**
    
      - Open a terminal and run:
          ```bash
            cat /dev/ttyUSB0
                  (or)
            cat /dev/ttyUSB2
          ```
  - **Receive Messages from GNSS**
    
    - To receive messages from the GNSS receiver, run:
      ```bash
        roslaunch novatel_oem7_driver oem7_tty.launch oem7_tty_name:=/dev/ttyUSB0
      ```
  
### ROS Noetic Installation

#### Prerequisites

Ensure you are using Ubuntu 20.04. The installation steps provided are tailored for this version.

#### Step 1: Set Up the ROS Repository

  - **Open a New Terminal:**
    
    - Press `Ctrl + Alt + T` or search for "Terminal" in the Ubuntu application launcher.
      
  - **Add the ROS Package Source:**
    
    - Run the following command to set up your computer to accept software from packages.ros.org: 
     ```bash
         sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
     ```
  - **Add the ROS APT Key:**
    
      - Run the following command to add the ROS apt key:
      ```bash
        sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
      ```
      
  - **Update the Package Index:**
    
    - Run the following command to update your package list:
      ```bash
        sudo apt update
      ```

#### Step 2: Install ROS Noetic

  - **Install ROS Desktop**
    
    -Run the following command to install the ROS Desktop package, which includes support for rqt, rviz, and other useful robotics packages:
    
    ```bash
      sudo apt install ros-noetic-desktop
    ```
    
    Note: The "ROS Desktop Full" package includes additional 2D/3D simulators and is not recommended for embedded platforms due to increased storage and computing requirements.

- **Source ROS Environment Variables:**
  
  - It is recommended to load the ROS environment variables automatically when you start a new shell session. Update your `.bashrc` file with the following commands
    
    ```bash
        echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
        source ~/.bashrc
    ```

#### Step 3: Install and Initialize rosdep

  - **Install rosdep and Other Dependencies:**
    
    - Run the following command to install rosdep and other required tools:
    ```bash
      sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    ```
    
  - **Initialize rosdep:**
    
    - Run the following commands to initialize rosdep and update its database:
    ```bash
      sudo rosdep init
      rosdep update
    ```

#### Conclusion

You have successfully installed ROS Noetic on Ubuntu 20.04. You can now proceed with developing and running your ROS applications. If you encounter any issues, refer to the official [ROS installation guide](http://wiki.ros.org/noetic/Installation/Ubuntu) for additional support.


## 2️⃣ Navigation
Start the Navigation of the vehicle

## [3️⃣ Running the Vehicle Dynamics File](https://github.com/nineRishav/Solio-S186/blob/master/Solio2-Camera/VehicleDynamics/vehicle_dynamics.py)

```bash
python vehicle_dynamics.py --name <driver_name>
```

## TiHAN Testbed Route

<img src="https://github.com/nineRishav/Solio-S186/blob/master/media/vehicledynamics-route.png" alt="TestBed Map" height="600">
