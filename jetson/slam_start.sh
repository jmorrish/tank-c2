#!/bin/bash
# slam_start.sh — Start ydlidar driver, SLAM Toolbox, and slam_bridge
# Run this on the Jetson before starting the C++ app.
set -e

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

DETECT=~/YOLOv8-TensorRT/csrc/jetson/detect
X3_YAML=~/ros2_ws/src/ydlidar_ros2_driver/params/X3.yaml

# 1. YDLIDAR ROS2 driver (takes exclusive ownership of /dev/ttyUSB0)
# Use ros2 run directly — the package launch file uses old ROS2 API (node_executable etc.)
# which is incompatible with Humble (renamed to executable/name/namespace).
echo "[slam_start] Starting ydlidar_ros2_driver with X3.yaml..."
ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node \
  --ros-args --params-file "$X3_YAML" &
LIDAR_PID=$!

# Give driver a moment to claim the port
sleep 2

# 2. slam_bridge (manages SLAM Toolbox internally, serves TCP 9997)
# slam_bridge publishes base_link → laser_frame TF internally before starting SLAM Toolbox
echo "[slam_start] Starting slam_bridge.py..."
python3 $DETECT/slam_bridge.py &
BRIDGE_PID=$!

echo ""
echo "[slam_start] All components running."
echo "  ydlidar driver PID : $LIDAR_PID"
echo "  slam_bridge PID    : $BRIDGE_PID"
echo ""
echo "  Save map:  ros2 run nav2_map_server map_saver_cli -f ~/map"
echo "  Stop all:  kill $LIDAR_PID $BRIDGE_PID"
echo ""

# Wait for either process to exit and propagate Ctrl+C
trap "kill $LIDAR_PID $BRIDGE_PID 2>/dev/null" EXIT INT TERM
wait
