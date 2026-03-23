cmake_minimum_required(VERSION 3.16)
project(yolov8 LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_ARCHITECTURES 60 61 62 70 72 75 86)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g")
set(CMAKE_BUILD_TYPE Debug)

# --- CUDA ---
find_package(CUDA REQUIRED)
message(STATUS "CUDA Libs: ${CUDA_LIBRARIES}")
message(STATUS "CUDA Headers: ${CUDA_INCLUDE_DIRS}")

# --- OpenCV ---
set(OpenCV_DIR /usr/local/lib/cmake/opencv4)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc dnn videoio highgui features2d calib3d video videostab cudaimgproc cudastereo cudawarping)
message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV libs: ${OpenCV_LIBRARIES}")

# --- TensorRT ---
find_library(TENSORRT_LIB nvinfer HINTS /usr/lib/aarch64-linux-gnu)
find_library(TENSORRT_PLUGIN_LIB nvinfer_plugin HINTS /usr/lib/aarch64-linux-gnu)
find_library(NVONNXPARSER_LIB nvonnxparser HINTS /usr/lib/aarch64-linux-gnu)
set(TensorRT_LIBRARIES ${TENSORRT_LIB} ${TENSORRT_PLUGIN_LIB} ${NVONNXPARSER_LIB})
set(TensorRT_INCLUDE_DIRS /usr/include/aarch64-linux-gnu)
message(STATUS "TensorRT Libs: ${TensorRT_LIBRARIES}")
message(STATUS "TensorRT Headers: ${TensorRT_INCLUDE_DIRS}")

# --- Threads ---
find_package(Threads REQUIRED)

# --- Boost (filesystem) ---
find_package(Boost REQUIRED COMPONENTS filesystem)
message(STATUS "Boost Libs: ${Boost_LIBRARIES}")
message(STATUS "Boost Headers: ${Boost_INCLUDE_DIRS}")

# --- ZeroMQ (for frame publishing) --- use pkg-config since no CMake config is installed
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZeroMQ REQUIRED libzmq)
message(STATUS "ZeroMQ Libs: ${ZeroMQ_LIBRARIES}")
message(STATUS "ZeroMQ Headers: ${ZeroMQ_INCLUDE_DIRS}")

# --- Include dirs ---
list(APPEND INCLUDE_DIRS
    ${CUDA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${TensorRT_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${ZeroMQ_INCLUDE_DIRS}  # NEW: For ZeroMQ headers
    /home/james/YOLOv8-TensorRT/BoTSORT-cpp/botsort/include
    include
    /usr/include/eigen3
    /usr/include/nlohmann  # NEW: For nlohmann/json.hpp (header-only; adjust path if different)
)

# --- BoTSORT sources ---
file(GLOB_RECURSE BOT_SORT_SOURCES
    /home/james/YOLOv8-TensorRT/BoTSORT-cpp/botsort/src/*.cpp
    /home/james/YOLOv8-TensorRT/BoTSORT-cpp/botsort/src/TRT_InferenceEngine/*.cpp
)

include_directories(${INCLUDE_DIRS})

# --- Our project sources ---
add_executable(${PROJECT_NAME}
    main.cpp
    object_detection.cpp
    movement.cpp
    comms.cpp
    stereo_depth.cpp
    runtime_config.cpp
    logger.cpp
    ${BOT_SORT_SOURCES}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    nvinfer
    nvinfer_plugin
    nvonnxparser
    ${OpenCV_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${Boost_LIBRARIES}
    ${ZeroMQ_LIBRARIES}  # NEW: Link ZeroMQ
    Threads::Threads
    atomic
)

message(STATUS "Included directories: ${INCLUDE_DIRS}")
message(STATUS "Linked libraries: ${OpenCV_LIBRARIES} ${CUDA_LIBRARIES} ${Boost_LIBRARIES} ${ZeroMQ_LIBRARIES} Threads::Threads")
message(STATUS "BoTSORT source files: ${BOT_SORT_SOURCES}")