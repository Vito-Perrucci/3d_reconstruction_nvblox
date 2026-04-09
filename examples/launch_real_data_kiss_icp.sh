#!/bin/bash

# --- Path Configuration ---
# Run this script from the workspace root
PKG_NAME="tizio_bot_pkg"
# The specific bag folder for real data tests
BAG_PATH="src/$PKG_NAME/rosbags/side_obstacles_bidirectional_cage_tour"
# Note: Check if your folder name is 'robags' or 'rosbags' and align here

echo "================================================="
echo "   REAL DATA RECONSTRUCTION (KISS-ICP + NVBLOX) "
echo "================================================="

# 1. Environment Setup
echo "--> Sourcing ROS 2 and Workspace..."
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "ERROR: 'install/setup.bash' not found. Please run 'colcon build' first."
    exit 1
fi

# 2. Cleanup
echo "--> Cleaning up existing ROS nodes..."
pkill -9 -f "ros2|rviz2" > /dev/null 2>&1
sleep 2

# 3. Launch the Real Robot Processing Pipeline
# This starts RViz, NVBlox, and KISS-ICP configured for bag data
echo "--> Launching NVBlox with KISS-ICP (Real Data Mode)..."
ros2 launch tizio_bot launch_real_robot_kiss.launch.py \
    data_source:=bag \
    voxel_size:=0.05 &

# Save the Process ID (PID)
LAUNCH_PID=$!

# 4. Wait for Initialization
echo "--> Waiting for RViz and NVBlox to initialize..."
sleep 10 

# 5. Check and Play Rosbag (filtered by topics)
if [ -d "$BAG_PATH" ] || [ -f "$BAG_PATH/metadata.yaml" ]; then
    echo "--> Starting Rosbag playback (Topic: /rslidar_points)..."
    # We use the specific topic filter as requested
    ros2 bag play "$BAG_PATH" --topics /rslidar_points
else
    echo "ERROR: Rosbag not found at $BAG_PATH"
    kill $LAUNCH_PID
    exit 1
fi

# 6. Final Cleanup
echo "================================================="
echo "Playback finished. Press ENTER to close everything."
read
kill $LAUNCH_PID
pkill -9 -f "ros2|rviz2|kiss_icp" > /dev/null 2>&1

echo "Shutdown complete."