#!/bin/bash
# =============================================================================
# Tank C2 — Fresh Jetson Setup Script
# =============================================================================
# Installs ALL dependencies on a fresh Jetson Orin (JetPack 5.x pre-flashed).
# Run this ON the Jetson as the target user (e.g. james).
#
# Prerequisites:
#   - JetPack 5.x flashed (provides CUDA, cuDNN, TensorRT)
#   - Internet access (apt, git clones, pip)
#   - User account exists (the script runs as this user, uses sudo for apt)
#
# Usage:
#   bash setup_jetson.sh [--skip-opencv] [--skip-ros2]
#
# Estimated time: ~3-4 hours (OpenCV build dominates)
# =============================================================================
set -e

SKIP_OPENCV=false
SKIP_ROS2=false
for arg in "$@"; do
    case "$arg" in
        --skip-opencv) SKIP_OPENCV=true ;;
        --skip-ros2)   SKIP_ROS2=true ;;
    esac
done

HOME_DIR="$HOME"
YOLO_DIR="$HOME_DIR/YOLOv8-TensorRT"
DETECT_DIR="$YOLO_DIR/csrc/jetson/detect"
ROS2_WS="$HOME_DIR/ros2_ws"
OPENCV_VERSION="4.10.0"

STEP=1
total_steps() {
    local t=10
    [ "$SKIP_OPENCV" = true ] && t=$((t - 1))
    [ "$SKIP_ROS2" = true ] && t=$((t - 2))
    echo $t
}
TOTAL=$(total_steps)

log() { echo ""; echo "═══════════════════════════════════════════════════════════════"; echo "  [$STEP/$TOTAL] $1"; echo "═══════════════════════════════════════════════════════════════"; STEP=$((STEP + 1)); }

# =============================================================================
# 1. System packages (apt)
# =============================================================================
log "Installing system packages..."

sudo apt update
sudo apt install -y \
    build-essential cmake cmake-curses-gui pkg-config git wget curl unzip \
    libboost-filesystem-dev libboost-system-dev \
    libzmq3-dev \
    libeigen3-dev \
    nlohmann-json3-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libavcodec-dev libavformat-dev libswscale-dev \
    libtbb-dev libjpeg-dev libpng-dev libtiff-dev \
    libv4l-dev v4l-utils \
    libgtk-3-dev libcanberra-gtk3-module \
    python3-dev python3-pip python3-numpy \
    libhdf5-dev \
    liblapack-dev libopenblas-dev \
    gfortran \
    tmux htop

# Python packages
pip3 install --user pyzmq paramiko

echo "  System packages installed."

# =============================================================================
# 2. OpenCV 4 with CUDA (built from source)
# =============================================================================
if [ "$SKIP_OPENCV" = false ]; then
    log "Building OpenCV $OPENCV_VERSION with CUDA (this takes 2-3 hours)..."

    OPENCV_BUILD_DIR="/tmp/opencv_build"
    mkdir -p "$OPENCV_BUILD_DIR"
    cd "$OPENCV_BUILD_DIR"

    # Download OpenCV + contrib
    if [ ! -d "opencv-$OPENCV_VERSION" ]; then
        wget -q "https://github.com/opencv/opencv/archive/$OPENCV_VERSION.zip" -O opencv.zip
        wget -q "https://github.com/opencv/opencv_contrib/archive/$OPENCV_VERSION.zip" -O opencv_contrib.zip
        unzip -q opencv.zip
        unzip -q opencv_contrib.zip
    fi

    mkdir -p "opencv-$OPENCV_VERSION/build"
    cd "opencv-$OPENCV_VERSION/build"

    # Detect CUDA compute capability
    # Orin NX/Super = 8.7, AGX Orin = 8.7, Xavier NX = 7.2
    CUDA_ARCH=""
    if command -v deviceQuery &> /dev/null; then
        CUDA_ARCH=$(deviceQuery | grep "CUDA Capability" | head -1 | grep -oP '\d+\.\d+' || true)
    fi
    if [ -z "$CUDA_ARCH" ]; then
        # Default to Orin family
        CUDA_ARCH="8.7"
        echo "  Could not detect CUDA arch, defaulting to $CUDA_ARCH (Orin)"
    fi
    echo "  Building for CUDA compute capability: $CUDA_ARCH"

    cmake \
        -D CMAKE_BUILD_TYPE=Release \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D OPENCV_EXTRA_MODULES_PATH="$OPENCV_BUILD_DIR/opencv_contrib-$OPENCV_VERSION/modules" \
        -D WITH_CUDA=ON \
        -D CUDA_ARCH_BIN="$CUDA_ARCH" \
        -D CUDA_ARCH_PTX="" \
        -D ENABLE_FAST_MATH=ON \
        -D CUDA_FAST_MATH=ON \
        -D WITH_CUBLAS=ON \
        -D WITH_CUDNN=ON \
        -D OPENCV_DNN_CUDA=ON \
        -D WITH_GSTREAMER=ON \
        -D WITH_V4L=ON \
        -D WITH_LIBV4L=ON \
        -D WITH_OPENGL=ON \
        -D WITH_TBB=ON \
        -D BUILD_opencv_python3=ON \
        -D BUILD_TESTS=OFF \
        -D BUILD_PERF_TESTS=OFF \
        -D BUILD_EXAMPLES=OFF \
        -D OPENCV_GENERATE_PKGCONFIG=ON \
        ..

    # Use all cores minus 1 to avoid OOM on 8GB boards
    NPROC=$(($(nproc) - 1))
    [ "$NPROC" -lt 1 ] && NPROC=1
    make -j"$NPROC"
    sudo make install
    sudo ldconfig

    echo "  OpenCV $OPENCV_VERSION with CUDA installed to /usr/local"
    cd "$HOME_DIR"
    # Clean up build files (several GB)
    rm -rf "$OPENCV_BUILD_DIR"
else
    log "Skipping OpenCV build (--skip-opencv)"
fi

# =============================================================================
# 3. ROS2 Humble
# =============================================================================
if [ "$SKIP_ROS2" = false ]; then
    log "Installing ROS2 Humble..."

    # Check if already installed
    if [ -f /opt/ros/humble/setup.bash ]; then
        echo "  ROS2 Humble already installed, skipping base install"
    else
        sudo apt install -y software-properties-common
        sudo add-apt-repository -y universe
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
            http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
            | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
        sudo apt update
        sudo apt install -y ros-humble-ros-base ros-humble-rmw-cyclonedds-cpp
    fi

    # Nav2 + SLAM + robot_localization packages
    log "Installing ROS2 Nav2, SLAM Toolbox, robot_localization..."
    sudo apt install -y \
        ros-humble-nav2-planner \
        ros-humble-nav2-costmap-2d \
        ros-humble-nav2-lifecycle-manager \
        ros-humble-nav2-msgs \
        ros-humble-nav2-smac-planner \
        ros-humble-slam-toolbox \
        ros-humble-robot-localization \
        ros-humble-tf2-ros \
        ros-humble-tf2-tools \
        python3-colcon-common-extensions

    # ROS2 Python deps
    pip3 install --user transforms3d

    # Source ROS2 in bashrc if not already there
    if ! grep -q "source /opt/ros/humble/setup.bash" "$HOME_DIR/.bashrc"; then
        echo "" >> "$HOME_DIR/.bashrc"
        echo "# ROS2 Humble" >> "$HOME_DIR/.bashrc"
        echo "source /opt/ros/humble/setup.bash" >> "$HOME_DIR/.bashrc"
        echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> "$HOME_DIR/.bashrc"
    fi

    source /opt/ros/humble/setup.bash

    # ── Build ydlidar_ros2_driver from source ──
    log "Building ydlidar_ros2_driver..."
    mkdir -p "$ROS2_WS/src"
    cd "$ROS2_WS/src"
    if [ ! -d "ydlidar_ros2_driver" ]; then
        git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git
    fi
    # Also need YDLidar-SDK
    if [ ! -d "YDLidar-SDK" ]; then
        git clone https://github.com/YDLIDAR/YDLidar-SDK.git
        cd YDLidar-SDK
        mkdir -p build && cd build
        cmake .. && make -j$(nproc)
        sudo make install
        cd "$ROS2_WS/src"
    fi
    cd "$ROS2_WS"
    source /opt/ros/humble/setup.bash
    colcon build --packages-select ydlidar_ros2_driver

    # Source workspace in bashrc
    if ! grep -q "source $ROS2_WS/install/setup.bash" "$HOME_DIR/.bashrc"; then
        echo "source $ROS2_WS/install/setup.bash" >> "$HOME_DIR/.bashrc"
    fi

    echo "  ROS2 Humble + Nav2 + SLAM + ydlidar installed."
else
    log "Skipping ROS2 install (--skip-ros2)"
fi

# =============================================================================
# 5. YOLOv8-TensorRT + BoTSORT-cpp
# =============================================================================
log "Setting up YOLOv8-TensorRT + BoTSORT-cpp..."

cd "$HOME_DIR"
if [ ! -d "$YOLO_DIR" ]; then
    git clone https://github.com/triple-Mu/YOLOv8-TensorRT.git
fi

cd "$YOLO_DIR"
if [ ! -d "BoTSORT-cpp" ]; then
    git clone https://github.com/viplix3/BoTSORT-cpp.git
fi

# Ensure BoTSORT assets exist (ReID ONNX model)
REID_MODEL="$YOLO_DIR/BoTSORT-cpp/assets/mobilenetv2_x1_4_msmt17.onnx"
if [ ! -f "$REID_MODEL" ]; then
    echo "  WARNING: BoTSORT ReID model not found at $REID_MODEL"
    echo "  You may need to download it manually or copy from existing Jetson."
    echo "  The app will still build but person re-ID won't work without it."
fi

# Create detect directory structure
mkdir -p "$DETECT_DIR/include"
mkdir -p "$DETECT_DIR/build"

echo "  YOLOv8-TensorRT directory structure ready."

# =============================================================================
# 6. Create runtime directories
# =============================================================================
log "Creating runtime directories..."

mkdir -p "$HOME_DIR/tank_missions"
mkdir -p "$HOME_DIR/tank_targets"
mkdir -p "$HOME_DIR/stereo_calib"

echo "  Created: tank_missions, tank_targets, stereo_calib"

# =============================================================================
# 7. Generate YOLOv8 TensorRT engine
# =============================================================================
log "YOLOv8 TensorRT engine setup..."

ENGINE_PATH="$YOLO_DIR/yolov8n.engine"
ONNX_PATH="$YOLO_DIR/yolov8n.onnx"

if [ -f "$ENGINE_PATH" ]; then
    echo "  Engine already exists at $ENGINE_PATH"
    echo "  NOTE: If this was copied from a different Jetson variant, delete it and re-run."
else
    # Download ONNX model if not present
    if [ ! -f "$ONNX_PATH" ]; then
        echo "  Downloading YOLOv8n ONNX model..."
        pip3 install --user ultralytics
        python3 -c "from ultralytics import YOLO; model = YOLO('yolov8n.pt'); model.export(format='onnx', opset=11, simplify=True)"
        mv yolov8n.onnx "$ONNX_PATH" 2>/dev/null || true
    fi

    if [ -f "$ONNX_PATH" ]; then
        echo "  Converting ONNX to TensorRT engine (this takes 5-15 minutes)..."
        /usr/src/tensorrt/bin/trtexec \
            --onnx="$ONNX_PATH" \
            --saveEngine="$ENGINE_PATH" \
            --fp16 \
            --workspace=4096
        echo "  Engine saved to $ENGINE_PATH"
    else
        echo "  WARNING: Could not find or download ONNX model."
        echo "  You can manually copy yolov8n.engine from an existing Jetson."
    fi
fi

# =============================================================================
# 8. Udev rules for camera symlinks
# =============================================================================
log "Setting up udev rules..."

RULES_SRC="$DETECT_DIR/99-tank-cameras.rules"
if [ -f "$RULES_SRC" ]; then
    sudo cp "$RULES_SRC" /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    echo "  Udev rules installed. Run 'sudo udevadm trigger' after plugging in cameras."
    echo "  NOTE: You may need to update busnum/devpath values for your USB topology."
    echo "  Run: udevadm info /dev/video0 | grep -E 'ATTRS\\{(busnum|devpath)\\}'"
else
    echo "  No udev rules found yet — will be installed after deploy_fresh.py runs."
fi

# =============================================================================
# 9. Serial port permissions (for lidar)
# =============================================================================
log "Setting up serial port permissions..."

sudo usermod -aG dialout "$USER"
echo "  Added $USER to dialout group (for /dev/ttyUSB0 lidar access)"
echo "  NOTE: Log out and back in for group change to take effect."

# =============================================================================
# 10. Verification
# =============================================================================
log "Verifying installation..."

echo ""
echo "Checking dependencies:"

# CUDA
if nvcc --version &> /dev/null; then
    echo "  [OK] CUDA: $(nvcc --version | grep release | awk '{print $6}')"
else
    echo "  [!!] CUDA: nvcc not found"
fi

# TensorRT
if [ -f /usr/lib/aarch64-linux-gnu/libnvinfer.so ]; then
    echo "  [OK] TensorRT: $(dpkg -l | grep libnvinfer-dev | awk '{print $3}' | head -1)"
else
    echo "  [!!] TensorRT: not found"
fi

# OpenCV
if python3 -c "import cv2; print(cv2.getBuildInformation())" 2>/dev/null | grep -q "CUDA:.*YES"; then
    echo "  [OK] OpenCV: $(python3 -c 'import cv2; print(cv2.__version__)') (CUDA enabled)"
else
    echo "  [!!] OpenCV: CUDA not detected (may need rebuild)"
fi

# cmake
echo "  [OK] cmake: $(cmake --version | head -1)"

# Boost
dpkg -l | grep -q libboost-filesystem-dev && echo "  [OK] Boost filesystem" || echo "  [!!] Boost filesystem"

# ZeroMQ
pkg-config --exists libzmq && echo "  [OK] ZeroMQ: $(pkg-config --modversion libzmq)" || echo "  [!!] ZeroMQ"

# Eigen3
[ -d /usr/include/eigen3 ] && echo "  [OK] Eigen3" || echo "  [!!] Eigen3"

# nlohmann-json
[ -f /usr/include/nlohmann/json.hpp ] && echo "  [OK] nlohmann-json" || echo "  [!!] nlohmann-json"

# ROS2
if [ "$SKIP_ROS2" = false ] && [ -f /opt/ros/humble/setup.bash ]; then
    echo "  [OK] ROS2 Humble"
    source /opt/ros/humble/setup.bash
    ros2 pkg list 2>/dev/null | grep -q nav2_planner && echo "  [OK] Nav2 planner" || echo "  [!!] Nav2 planner"
    ros2 pkg list 2>/dev/null | grep -q slam_toolbox && echo "  [OK] SLAM Toolbox" || echo "  [!!] SLAM Toolbox"
    [ -d "$ROS2_WS/install/ydlidar_ros2_driver" ] && echo "  [OK] ydlidar_ros2_driver" || echo "  [!!] ydlidar_ros2_driver"
else
    echo "  [--] ROS2: skipped"
fi

# YOLOv8 engine
[ -f "$ENGINE_PATH" ] && echo "  [OK] YOLOv8 engine: $ENGINE_PATH" || echo "  [!!] YOLOv8 engine: not found"

# BoTSORT
[ -d "$YOLO_DIR/BoTSORT-cpp/botsort/src" ] && echo "  [OK] BoTSORT-cpp" || echo "  [!!] BoTSORT-cpp"

# ReID model
[ -f "$REID_MODEL" ] && echo "  [OK] ReID ONNX model" || echo "  [!!] ReID ONNX model: not found"

# Directory structure
[ -d "$DETECT_DIR" ] && echo "  [OK] detect/ directory" || echo "  [!!] detect/ directory"
[ -d "$HOME_DIR/tank_missions" ] && echo "  [OK] tank_missions/" || echo "  [!!] tank_missions/"
[ -d "$HOME_DIR/tank_targets" ] && echo "  [OK] tank_targets/" || echo "  [!!] tank_targets/"

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo "  SETUP COMPLETE"
echo ""
echo "  Next steps:"
echo "    1. Run deploy_fresh.py from your Windows machine to upload"
echo "       source code and build the C++ app"
echo "    2. Copy stereo_params_cuda.xml to ~/stereo_calib/"
echo "       (if using the same stereo camera)"
echo "    3. Update 99-tank-cameras.rules for your USB topology"
echo "    4. Test with: bash start_tank.sh --no-ekf"
echo "═══════════════════════════════════════════════════════════════"
