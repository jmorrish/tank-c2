# Tank C2 — Claude Context

## Stack
- **Jetson Orin Super** (192.168.1.167) — C++ app (`yolov8`), YOLOv8 detection, person tracking, comms hub
- **VPS** (87.106.188.134) — Node.js `server.js`, website host, WebSocket relay
- **Control Teensy 4.1** (192.168.1.177:23) — Wheels + PTU
- **Sensor Teensy 4.1** (192.168.1.178:23) — BNO085 IMU + BN880 GPS + TOF + wheel encoders

## Credentials
- VPS SSH: `root` / `CP2Xy3TDzxHGVpcV`
- Jetson SSH: `james` / `!123Wo55a`
- Website password: `T4nkC2!2026`

## Deployment
**Always use Python paramiko SFTP** — sshpass not on Windows, scp password auth blocked on Jetson.
- Jetson C++ change: `sftp.put()` files → `cmake --build build` in `/home/james/YOLOv8-TensorRT/csrc/jetson/detect/`
- VPS JS change: `sftp.put()` files → `pm2 reload /var/www/tank-c2/ecosystem.config.js`
- Cannot use git on Jetson — deploy folder is inside upstream YOLOv8-TensorRT repo

## Key Paths
| What | Path |
|------|------|
| Local dev | `c:\python\tank-c2\` |
| Jetson C++ source | `/home/james/YOLOv8-TensorRT/csrc/jetson/detect/` |
| Jetson binary | `.../detect/build/yolov8` |
| Jetson missions dir | `/home/james/tank_missions/` |
| VPS Node.js app | `/var/www/tank-c2/` (pm2: `tank-c2`) |
| Person gallery | `/home/james/tank_targets/<id>/` |
| Stereo calibration | `/home/james/stereo_calib/stereo_params_cuda.xml` (contains map1x/y, map2x/y, P1, T) |
| Sensor Teensy sketch | `C:\Users\admin\Documents\Arduino\SENSOR_TEENSY\SENSOR_TEENSY.ino` |
| Control Teensy sketch | `C:\Users\admin\Documents\Arduino\PTU_Wheel_IMU_ETH_NoStutt\PTU_Wheel_IMU_ETH_NoStutt.ino` |

## Jetson Deploy Directory (`/home/james/YOLOv8-TensorRT/csrc/jetson/detect/`)
**C++ source files:**
| File | Purpose |
|------|---------|
| `main.cpp` | Entry point, arg parsing, subsystem startup |
| `comms.cpp` / `include/comms.h` | All TCP comms: control/sensor Teensys, VPS IPC, SLAM bridge, stereo depth |
| `object_detection.cpp` | YOLOv8 TensorRT inference, camera capture, BoTSORT tracking, person re-ID |
| `stereo_depth.cpp` | CUDA StereoBM stereo depth, ZMQ PUB ports 5557 (ROS2) and 5558 (mjpeg_bridge) |
| `movement.cpp` | PTU PID control, wheel follow-distance control |
| `runtime_config.cpp` | Runtime config load/save (`config.json` in working dir) |
| `logger.cpp` | Logging helpers |
| `include/config.h` | All compile-time defaults (IPs, ports, thresholds, paths) |

**Python / shell scripts:**
| File | Purpose |
|------|---------|
| `mjpeg_bridge.py` | MJPEG fan-out relay, port 8080. Subscribes to ZMQ 5558 for stereo disparity frames |
| `slam_bridge.py` | ROS2 → TCP bridge, port 9997. Owns SLAM Toolbox lifecycle |
| `slam_start.sh` | Starts ydlidar ROS2 driver + slam_bridge.py |
| `web_interface.py` | (legacy / unused — Node.js VPS is the active web interface) |
| `readdis.py` | Dev/debug script for reading distance sensor |
| `slam_toolbox_params_odom.yaml` | SLAM Toolbox config with odometry enabled |
| `slam_toolbox_params_no_odom.yaml` | SLAM Toolbox config without odometry |

**Old builds (do not deploy to):**
- `build_old/` and `build_old2/` — archived binaries, kept for rollback reference

## Sensor Teensy Hardware (Teensy 4.1)
| Peripheral | Serial | Pins | Baud | Notes |
|-----------|--------|------|------|-------|
| BNO085 IMU | Serial3 | TX=14, RX=15 | 115200 | PS0=GND, PS1=3V3 (SH2-UART). Adafruit_BNO08x lib, single instance only |
| BN880 GPS | Serial6 | TX=24, RX=25 | 115200 | NMEA, $GNGGA + $GNRMC forwarded |
| TOF sensor | Serial8 | TX=35, RX=34 | 921600 | 16-byte binary frame, 0x57 header |
| Left encoder | — | A=33, B=34 | — | Interrupt on A, direction from B |
| Right encoder | — | A=36, B=35 | — | Interrupt on A, direction from B |
| Ethernet | built-in | — | — | Static IP 192.168.1.178, port 23 |

⚠ TOF RX=34 and Left ENC B=34 share the same pin — move TOF to Serial7 (RX=28) if both needed simultaneously.

## Sensor Teensy Protocol
**Outbound (Teensy → Jetson):**
- `SENSOR_READY` — on new client connect
- `YPR <yaw> <pitch> <roll>` — body IMU degrees, 50 Hz default
- `$GNGGA,...` / `$GNRMC,...` — validated raw NMEA
- `ENC <left_sps> <right_sps>` — encoder steps/second, 10 Hz
- `TOF <dist_m>` — TOF distance in metres

**Inbound (Jetson → Teensy):**
- `IMU_ON` / `IMU_OFF` — toggle YPR streaming
- `IMU_RATE <hz>` — set YPR rate 1–200 Hz

## Jetson App Protocol
- Control Teensy TCP 192.168.1.177:23 — wheels + PTU (sends `LS<n>RS<n>`, `VP<pan>T<tilt>`, `P<pan>T<tilt>`; receives `ENC <l> <r>`, `PTU_YPR <y> <p> <r>`)
- Sensor Teensy TCP 192.168.1.178:23 — IMU/GPS/TOF/encoders (sends `IMU_ON`, `IMU_RATE <hz>`; receives `YPR`, `ENC`, `TOF`, NMEA)
- VPS IPC TCP port 9999 — status JSON + commands
- SLAM bridge TCP port 9997 — scan + pose data
- MJPEG stream port 8080 (mjpeg_bridge.py)
- ZMQ PUB port 5557 — stereo rectified frames for ROS2/RTAB-Map
- ZMQ PUB port 5558 — colourised disparity JPEG for mjpeg_bridge

## Run Command
```
./yolov8 /home/james/YOLOv8-TensorRT/yolov8n.engine --auto-continue --headless
```
Optional: `--ptu-ip 192.168.1.177:23` `--sensor-ip 192.168.1.178:23` (these are the compiled defaults)

## Implemented Features
- Full comms stack: Jetson → VPS → browser WebSocket
- Mobile-friendly dark web UI: map, mission planner, live telemetry, radar
- MJPEG stream fan-out relay
- Mission system: save/push/execute/abort/resume/skip waypoint, GPS navigation
- Person re-ID: BoTSORT, cosine similarity (threshold 0.75), persistent gallery
- Web gallery: thumbnails, live updates, one-click follow, inline rename
- Session auth: /login, 7-day cookie, WS auth via 30s one-time token
- YDLIDAR X3: 360° radar + obstacle avoidance via SLAM bridge
- SLAM: ROS2 ydlidar_ros2_driver → SLAM Toolbox → slam_bridge.py TCP 9997
- Odometry auto-plug-and-play: slam_bridge.py activates odom when encoders fresh

## Pending / Not Yet Wired
- Wheel encoders — sketch ready, hardware not yet wired
- PTU IMU — control Teensy sketch has it on Serial5; `controlRxLoop` parses `PTU_YPR` but this was reverted from deployed binary pending FPS fix
- IMU/GPS → slam_bridge sensor fusion (robot_localization EKF planned)
- RTAB-Map 3D mapping via ZMQ port 5557 (stereo_depth.cpp publishes rectified pairs)

## Known Issues / Gotchas
- **Stereo camera modes**: Camera is opened at 2560×720 MJPEG. Full frame → stereo depth, left half (1280×720) → detection. **2560×720 MJPEG only supports 30fps** — requesting any other fps causes V4L2 to fall back to raw YUV at 3fps. Always use `CAP_PROP_FPS, 30` for this resolution. Calibration: `/home/james/stereo_calib/stereo_params_cuda.xml`.
- **Rebuilding = recompiles everything**: `cmake --build build` recompiles all changed source files, not just comms.cpp. If object_detection.cpp has in-progress changes they will be compiled in too.
- **PTU_YPR parsing**: Added to local `comms.cpp` `controlRxLoop` but currently reverted from Jetson until FPS issue is resolved.

## Key Technical Notes
- BNO085: Adafruit_BNO08x supports **one UART instance only** (global HAL state). Use one per Teensy.
- BNO085 init: `Serial.begin(115200)` → `Serial.setTimeout(0)` → `begin_UART()` → `enableReport(SH2_ROTATION_VECTOR, 20)`. Check `wasReset()` every loop and re-enable reports.
- NativeEthernet must init **before** UART on Teensy 4.1 (DMA channel conflict).
- YDLIDAR X3 working ROS2 params: `X3.yaml` (baud=115200, single_channel=true, fixed_resolution=true). Not `ydlidar.yaml`.
- slam_bridge.py tunable robot params: `WHEEL_DIAMETER_M`, `TRACK_WIDTH_M`, `ENC_CPR`
- `#undef PI` before `#define PI` before Adafruit BNO08x include (conflicts with math.h)
