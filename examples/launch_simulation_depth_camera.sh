#!/bin/bash

# --- Path Configuration ---
# This script assumes it is being run from the workspace root.
PKG_NAME="tizio_bot_pkg"
BAG_PATH="src/$PKG_NAME/rosbags/registrazione_teleop"
WORLD_PATH="src/$PKG_NAME/worlds/room.world"

echo "================================================="
echo "   NVBLOX RECONSTRUCTION SYSTEM - AUTO START    "
echo "================================================="

# 1. Environment Setup
echo "--> Sourcing ROS 2 and Workspace..."
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "ERROR: 'install/setup.bash' not found."
    echo "Please run 'colcon build' in the workspace root first."
    exit 1
fi

# 2. Cleanup Gazebo (kills any hung processes from previous runs)
echo "--> Cleaning up old Gazebo instances..."
pkill -9 gzserver > /dev/null 2>&1
pkill -9 gzclient > /dev/null 2>&1
sleep 2

# 3. Start Simulation in background
echo "--> Launching Simulation and nvblox..."
ros2 launch tizio_bot launch_sim.launch.py \
    world:=$WORLD_PATH \
    nvblox_mode:=static \
    sensors:=depth_camera \
    use_nav2:=false \
    voxel_size:=0.1 &

# Save the Process ID (PID) to kill it later
SIM_PID=$!

# 4. Wait for Initialization
echo "--> Waiting 10 seconds for Gazebo and nvblox to stabilize..."
sleep 10 

# 5. Check and Play Rosbag
if [ -d "$BAG_PATH" ] || [ -f "$BAG_PATH/metadata.yaml" ]; then
    echo "--> Starting Rosbag playback: $BAG_PATH"
    ros2 bag play "$BAG_PATH"
else
    echo "ERROR: Rosbag not found at $BAG_PATH"
    kill $SIM_PID
    exit 1
fi

# 6. Cleanup
echo "================================================="
echo "Playback finished. Shutting down simulation..."
kill $SIM_PID
# Final check to ensure Gazebo closes
pkill -9 gzserver > /dev/null 2>&1

echo "Shutdown complete."