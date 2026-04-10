# 3D Reconstruction Project – Personal Thesis (Vito Perrucci)
This project implements a **3D reconstruction pipeline** using [nvblox] (https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/index.html).  
It is part of my personal thesis work and is designed to run inside the **Isaac ROS container** provided by NVIDIA, which simplifies the deployment of `nvblox` and its dependencies.

---

## Important Dates

- **Thesis Discussion:** March 23–25, 2026  
- **Graduation Announcement:** March 27, 2026

---

## Overview

The goal of this project is to provide a fully functional 3D reconstruction pipeline that can be easily launched and tested within the NVIDIA Isaac ROS environment.  
The container ensures all required libraries and tools are pre-installed, minimizing setup time and compatibility issues.

---

<!-- ![Demo](./gifs/) -->

<p align="center">
<strong>Depth camera and 3D LiDAR Reconstruction demos</strong><br>
  <img src="./gif/depth_reconstruction_gazebo.gif"  width="400"/>
  <img src="./gif/only lidar reco.gif"  width="400"/>
</p>

<p align="center">
    <strong>Coloration close up and complete demo overview with costmap</strong><br>
  <img src="./gif/lidar coloration close up.gif"  width="400"/>
  <img src="./gif/depth_costmap_compressed.gif"  width="400"/>
</p>

## Setup & Installation

Follow these steps to set up the environment and run the project:

### 1. Install the Isaac ROS container
Follow NVIDIA's official instructions to install the container.
(https://nvidia-isaac-ros.github.io/concepts/dev_env/index.html)

### 2. Mount the project folder inside the container
Clone this repository inside the container in a new folder "tizio_bot_pkg" (or outside but mounting the volume in it)
Edit the `run_dev.sh` script in the container directory.  
- Adjust the volume mount to point to your workspace (if needed).  
- Remove the `--rm` flag if you want the container **not to be removed** after it stops.

### 3. Launch the container
Run the `run_dev.sh` script to start the container (the script builds the container and automatically runs it).
After that is possible to close the container with the "exit" command and reopen it with "docker start ..."

### 4. Install `nvblox` inside the container
Follow the official [nvblox ROS instructions](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html) to install `nvblox` within the container.

### 5. Install project dependencies inside the container
Install the following ROS Humble packages:

```bash
sudo apt update
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-teleop-twist-joy \
    ros-humble-joy \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-slam-toolbox
```

### 6. Build with colcon
```bash
colcon build --packages-select tizio_bot && source install/setup.bash && clear
```

## 7. Run Simulation Example

This example demonstrates the 3D reconstruction pipeline running in a simulated environment. The setup features a mobile robot equipped with a **depth camera** navigating a specific map called **"room"**. 

When the script is launched, a pre-recorded **rosbag** starts automatically. This rosbag contains a guided exploration path that moves the robot around the environment to perform the reconstruction, as shown in the image below:

![Exploration Path](./gif/rosbag_path.png)

### Customization
You can modify the execution parameters by opening the `.sh` files inside the `examples` folder. Specifically, you can change:
* **Voxel Size**: Adjust the resolution of the reconstruction map.
* **Sensor Type**: You can switch the `sensors` parameter from `depth_camera` to `lidar_3D`.

---

### **IMPORTANT NOTES**

* **System Performance**: `nvblox` is a high-performance but computationally heavy framework. The stability of the simulation depends entirely on your hardware capabilities. If the application appears to freeze or hang during the first launch, it is highly recommended to **restart the script**.
* **Lidar 3D Limitations**: Occasionally, the `lidar_3D` sensor configuration may not perform optimally with the default rosbag path due to processing overhead or physics synchronization. In such cases, you can disable the automatic rosbag playback and move the robot manually using a **joystick** or **teleop_twist_keyboard**.
* **Manual Control**: For detailed instructions on how to manually drive the robot and configure different settings, refer to the following section.

---

## 8. Manual Robot Control

You can control the robot manually using either the keyboard or a joystick. 

### Joystick Configuration
The system is pre-configured for a **DualShock 4 (PS4)** controller. To use a physical controller, you must update your `run_dev.sh` file with the specific input device paths and rebuild the container.

**Controls:**
* **Movement:** Left Analog Stick.
* **Deadman's Switch:** For safety, the robot will only move if a shoulder button is held down. Hold **L1** for standard speed or **R1** for turbo speed.

> **Note:** If you are using a controller other than a DualShock 4, compatibility is not guaranteed. For a more reliable manual connection, we recommend using `teleop_twist_keyboard`.

### Launching Manual Mode
To launch the simulation without the automated rosbag playback, you can either comment out the rosbag execution line in your script or run the following command directly:

```bash
ros2 launch tizio_bot launch_sim.launch.py \
world:=src/tizio_bot_pkg/worlds/room.world \
nvblox_mode:=static \
sensors:=depth_camera \
use_nav2:=false \
voxel_size:=0.1
```
### Nav2

It's possible here (without enabling the play of the rosbag) to enable Nav2 changing the argument use_Nav2 and set it to 'true'. After that the Nav2 launcher is added to the project launch and it's possible to select a point into the map and let the robot select a path avoiding obstacles.

### **IMPORTANT NOTES**

* **Initial Pose Selection**: It's foundamental to know that for doing this is strictly needed to estimate the initial position of the robot at the start using the Robot Pose Estimation on the Rviz Visualizator.
* **Compatibility**: To use Nav2 the 3D lidar sensor configuration is needed. It's not possible to only use the depth camera for the correct implementation of Nav2. Matching the use of depth camera and Nav2 will led into an error that asks to use another sensor configuration or set the Nav2 parameter to false.



### 9. Real-World Examples

To test the system with real-world data, follow these steps to set up the necessary datasets:

1.  **Prepare the directory**: Create a folder named `rosbags` in your repository root.
2.  **Download the data**: Download the rosbags from this [Google Drive link](https://drive.google.com/file/d/18MX-pQo9-uL8bEZEudknRuvn3VyVqemE/view?usp=sharing).
3.  **Extract**: Unzip the files directly into the `rosbags` folder.

### Launching the Examples
You can run two different odometry configurations using the provided scripts:

* **KISS-ICP (Software Approach)**: Uses the KISS-ICP algorithm for LiDAR-based odometry.
    ```bash
    ./src/tizio_bot_pkg/examples/launch_real_data_kiss_icp.sh 
    ```

* **Vicon (Hardware Approach)**: Uses high-precision external motion capture data for odometry.
    ```bash
    ./src/tizio_bot_pkg/examples/launch_real_data_vicon.sh
    ```

---
**ATTENTION**

* Both examples use the same rosbag by default. You can experiment with different datasets by modifying the paths inside the `.sh` files.
* **Compatibility**: Rosbags recorded in simulation are **not** compatible with real-world launch scripts, and vice versa.