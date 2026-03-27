#!/bin/bash
# start_tank.sh — One command starts everything. One Ctrl+C stops everything.
#
# Starts: ydlidar driver, slam_bridge, robot_localization EKF, Nav2 planner,
#         mjpeg_bridge, and the main C++ app.
#
# Usage:
#   bash start_tank.sh [--no-nav2] [--no-ekf]
#
set -e

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

DETECT=~/YOLOv8-TensorRT/csrc/jetson/detect
ENGINE=~/YOLOv8-TensorRT/yolov8n.engine
X3_YAML=~/ros2_ws/src/ydlidar_ros2_driver/params/X3.yaml
PIDS=()

# Kill stale processes from previous runs
echo "[start_tank] Cleaning up stale processes..."
pkill -f "mjpeg_bridge.py" 2>/dev/null || true
pkill -f "slam_bridge.py" 2>/dev/null || true
pkill -f "ydlidar_ros2_driver_node" 2>/dev/null || true
pkill -f "planner_server" 2>/dev/null || true
pkill -f "lifecycle_manager_planner" 2>/dev/null || true
pkill -f "ekf_node" 2>/dev/null || true
pkill -f "navsat_transform_node" 2>/dev/null || true
pkill -f "yolov8.*engine" 2>/dev/null || true
sleep 1

# Parse flags
USE_NAV2=true
USE_EKF=true
for arg in "$@"; do
    case "$arg" in
        --no-nav2) USE_NAV2=false ;;
        --no-ekf)  USE_EKF=false ;;
    esac
done

cleanup() {
    echo ""
    echo "[start_tank] Stopping all processes..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    echo "[start_tank] Done."
}
trap cleanup EXIT INT TERM

STEP=1
TOTAL=6
[ "$USE_EKF" = false ] && TOTAL=$((TOTAL - 1))
[ "$USE_NAV2" = false ] && TOTAL=$((TOTAL - 1))

# 1. Ydlidar driver
echo "[start_tank] $STEP/$TOTAL Ydlidar driver..."
ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node \
    --ros-args --params-file "$X3_YAML" &
PIDS+=($!)
sleep 2
STEP=$((STEP + 1))

# 2. SLAM bridge (manages SLAM Toolbox lifecycle internally, TCP :9997)
echo "[start_tank] $STEP/$TOTAL SLAM bridge..."
python3 $DETECT/slam_bridge.py &
PIDS+=($!)
sleep 4   # wait for SLAM Toolbox to initialise TF + map
STEP=$((STEP + 1))

# 3. Robot localization EKF (fuses IMU + encoders + GPS)
if [ "$USE_EKF" = true ]; then
    echo "[start_tank] $STEP/$TOTAL Robot localization (EKF + navsat)..."
    ros2 run robot_localization ekf_node \
        --ros-args --params-file $DETECT/ekf_params.yaml &
    PIDS+=($!)
    ros2 run robot_localization navsat_transform_node \
        --ros-args --params-file $DETECT/ekf_params.yaml &
    PIDS+=($!)
    STEP=$((STEP + 1))
fi

# 4. Nav2 planner + costmap
if [ "$USE_NAV2" = true ]; then
    echo "[start_tank] $STEP/$TOTAL Nav2 path planner..."
    ros2 launch $DETECT/nav2_planner_launch.py &
    PIDS+=($!)
    sleep 2
    STEP=$((STEP + 1))
fi

# 5. MJPEG bridge (stream relay on port 8080)
echo "[start_tank] $STEP/$TOTAL MJPEG bridge..."
python3 $DETECT/mjpeg_bridge.py &
PIDS+=($!)
STEP=$((STEP + 1))

# 6. Main app (detection + tracking + comms)
echo "[start_tank] $STEP/$TOTAL Main app..."
$DETECT/build/yolov8 $ENGINE --auto-continue --headless --config $DETECT/config.json &
PIDS+=($!)

echo ""
echo "═══════════════════════════════════════════"
echo "  Tank C2 running (${#PIDS[@]} processes)"
[ "$USE_EKF" = false ]  && echo "  EKF:  disabled (--no-ekf)"
[ "$USE_NAV2" = false ] && echo "  Nav2: disabled (--no-nav2)"
echo "  Press Ctrl+C to stop everything"
echo "═══════════════════════════════════════════"
echo ""

wait
