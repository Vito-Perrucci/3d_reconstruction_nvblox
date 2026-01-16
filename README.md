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
Clone this repository inside the container folder (or outside but mounting the volume in it)
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

### 6. Build with colcon the project and Run a simulations test
```bash
colcon build --packages-select tizio_bot && source install/setup.bash && clear && ^Cs2 launch tizio_bot launch_sim.launch.py world:=src/tizio_bot_pkg/worlds/cones.world nvblox_mode:=static sensors:=lidar_3D
```
this command runs a simulation test with a specific world in gazebo and the lidar_3D. It's possible to change lidar_3D with depth_camera for another sensor configuration

### 7. Move the robot
To move the robot it's possible to use turtle keyboard or a joystick. To use a joystick is needed to rebuild the container adding the input specification in the run_dev.sh file.
The joystick used is a dualshock4 (playstation 4 controller), the robot moves with the left stick but only if the L1 or R1 (higher speed) is continuously pressed.

