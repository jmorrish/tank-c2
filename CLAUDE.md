# Tank C2 — Claude Context

## Stack
- **Jetson Orin Super** (192.168.1.167) — C++ app (`yolov8`), YOLOv8 detection, person tracking, comms hub
- **VPS** (87.106.188.134) — Node.js `server.js`, website host, WebSocket relay
- **Control Teensy 4.1** (192.168.1.177:23) — Wheels + PTU
- **Sensor Teensy 4.1** (192.168.1.178:23) — BNO085 IMU + BN880 GPS + TOF + wheel encoders
- **FLIR Tau Thermal Camera** (192.168.0.71, RTSP) — Thermal imaging via `rtsp://192.168.0.71/z3-2.sdp`, grabbed by `cam2_rtsp` thread in object_detection.cpp

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
| Deploy script | `c:\python\tank-c2\deploy_phase4.py` (paramiko SFTP + cmake build) |

## Jetson Deploy Directory (`/home/james/YOLOv8-TensorRT/csrc/jetson/detect/`)
**C++ source files:**
| File | Purpose |
|------|---------|
| `main.cpp` | Entry point, arg parsing, subsystem startup, behavior registration |
| `comms.cpp` / `include/comms.h` | All TCP comms: control/sensor Teensys, VPS IPC, SLAM bridge, stereo depth. Forwards IMU/GPS to slam_bridge for ROS2 publishing |
| `object_detection.cpp` | YOLOv8 TensorRT inference, camera capture, BoTSORT tracking, person re-ID |
| `stereo_depth.cpp` | CUDA StereoBM stereo depth, ZMQ PUB ports 5557 (ROS2) and 5558 (mjpeg_bridge) |
| `behavior_coordinator.cpp` | Behavior state machine: transitions, tick loop at ~500Hz, actuator zeroing |
| `follow_behavior.cpp` | FollowBehavior: PID-based person tracking (PTU + wheels) extracted from old movement.cpp |
| `mission_behavior.cpp` | MissionBehavior: wraps MissionRunner, owns thread, auto-transitions to STOPPED on complete |
| `mission.cpp` / `include/mission.h` | MissionRunner: GPS waypoint navigation with sub-waypoint path planning and replanning |
| `nav2_planner.cpp` / `include/nav2_planner.h` | Nav2Planner: requests obstacle-aware paths via slam_bridge TCP, falls back to direct GPS |
| `sensor_store.cpp` / `include/sensor_store.h` | SensorStore with generic key-value slots for extensible sensor data |
| `slam_link.cpp` / `include/slam_link.h` | Bidirectional TCP to slam_bridge.py (port 9997), sends/receives JSON lines |
| `teensy_link.cpp` / `include/teensy_link.h` | Generic TCP link for Teensy connections (control + sensor) |
| `runtime_config.cpp` | Runtime config load/save (`config.json` in working dir) |
| `logger.cpp` | Logging helpers |
| `include/config.h` | All compile-time defaults (IPs, ports, thresholds, paths) |
| `include/behavior.h` | Behavior base class + StoppedBehavior, ManualBehavior inline implementations |
| `include/behavior_coordinator.h` | BehaviorCoordinator class declaration |
| `include/control_mode.h` | ControlMode enum: STOPPED, MANUAL, FOLLOW, MISSION |
| `include/follow_behavior.h` | FollowBehavior class declaration |
| `include/mission_behavior.h` | MissionBehavior class declaration |
| `include/path_planner.h` | PathPlanner abstract interface + DirectGPSPlanner (straight-line fallback) |
| `include/obstacle_utils.h` | Shared obstacle avoidance steering utility (suggestAvoidanceSteer) |
| `include/helpers.h` | Free helper functions (steady_now_ns, etc.) |

**Python / shell scripts:**
| File | Purpose |
|------|---------|
| `start_tank.sh` | **Unified launcher** — starts all 6 subsystems, one Ctrl+C stops everything. Supports `--no-nav2` and `--no-ekf` flags |
| `mjpeg_bridge.py` | MJPEG fan-out relay, port 8080. Subscribes to ZMQ 5558 for stereo disparity frames |
| `slam_bridge.py` | ROS2 ↔ TCP bridge (port 9997). Owns SLAM Toolbox lifecycle, publishes /imu/data + /gps/fix, Nav2 ComputePathToPose action client, dynamic odom→base_link TF, GPS↔map coordinate conversion |
| `web_interface.py` | (legacy / unused — Node.js VPS is the active web interface) |
| `readdis.py` | Dev/debug script for reading distance sensor |

**Config / launch files:**
| File | Purpose |
|------|---------|
| `nav2_params.yaml` | Nav2 planner server config: SmacPlanner2D + global costmap (static + obstacle + inflation layers) |
| `ekf_params.yaml` | robot_localization EKF config: fuses /odom + /imu/data. navsat_transform for GPS↔odom conversion |
| `nav2_planner_launch.py` | ROS2 launch file for planner_server + lifecycle_manager (planner creates its own costmap internally) |
| `slam_toolbox_params_odom.yaml` | SLAM Toolbox config with odometry enabled |
| `slam_toolbox_params_no_odom.yaml` | SLAM Toolbox config without odometry |

**Old builds (do not deploy to):**
- `build_old/` and `build_old2/` — archived binaries, kept for rollback reference

## Sensor Teensy Hardware (Teensy 4.1)
| Peripheral | Serial | Pins | Baud | Notes |
|-----------|--------|------|------|-------|
| BNO085 IMU | Serial3 | TX=14, RX=15 | 115200 | PS0=GND, PS1=3V3 (SH2-UART). Adafruit_BNO08x lib, single instance only |
| BN880 GPS | Serial6 | TX=24, RX=25 | 115200 | NMEA, $GNGGA + $GNRMC forwarded |
| TOF sensor | Serial7 | TX=29, RX=28 | 921600 | 16-byte binary frame, 0x57 header |
| Left encoder | — | A=33, B=34 | — | Interrupt on A, direction from B |
| Right encoder | — | A=36, B=35 | — | Interrupt on A, direction from B |
| Ethernet | built-in | — | — | Static IP 192.168.1.178, port 23 |

TOF moved from Serial8 to Serial7 to resolve pin conflict with encoder B pins (34/35).

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
- SLAM bridge TCP port 9997 — bidirectional JSON-line protocol (see below)
- MJPEG stream port 8080 (mjpeg_bridge.py)
- ZMQ PUB port 5557 — stereo rectified frames for ROS2/RTAB-Map
- ZMQ PUB port 5558 — colourised disparity JPEG for mjpeg_bridge

### SLAM Bridge TCP Protocol (port 9997)
**C++ → slam_bridge (forwarded sensor data):**
- `{"type":"imu_data","yaw":..,"pitch":..,"roll":..}` — BNO085 IMU, published as `/imu/data`
- `{"type":"gps_data","lat":..,"lon":..,"alt":..,"quality":..,"sats":..}` — GPS fix, published as `/gps/fix` (1Hz)
- `{"type":"gps_rmc","speed_knots":..,"course_deg":..}` — GPS speed/course for heading calibration
- `{"type":"plan_path","from_lat":..,"from_lon":..,"to_lat":..,"to_lon":..}` — request Nav2 path

**slam_bridge → C++:**
- `{"type":"scan","ranges":[...],"angle_min":..,"angle_max":..}` — lidar scan data
- `{"type":"slam_pose","x":..,"y":..,"theta":..}` — SLAM Toolbox pose in map frame
- `{"type":"planned_path","ok":true,"waypoints":[[lat,lon],...]}` — Nav2 planned path result
- `{"type":"planned_path","ok":false,"error":"..."}` — Nav2 planning failure

## Run Command
**Preferred — unified launcher (starts everything):**
```
bash start_tank.sh              # full stack: lidar + SLAM + EKF + Nav2 + MJPEG + C++ app
bash start_tank.sh --no-ekf     # skip EKF + navsat (use until wheel encoders are wired)
bash start_tank.sh --no-nav2    # skip Nav2 planner
```

**Direct C++ app only (requires ROS2 stack already running):**
```
./yolov8 /home/james/YOLOv8-TensorRT/yolov8n.engine --auto-continue --headless
```
Optional: `--ptu-ip 192.168.1.177:23` `--sensor-ip 192.168.1.178:23` (these are the compiled defaults)

## Implemented Features
- Full comms stack: Jetson → VPS → browser WebSocket
- Mobile-friendly dark web UI: map, mission planner, live telemetry, radar
- MJPEG stream fan-out relay
- Mission system: save/push/execute/abort/resume/skip waypoint, GPS navigation with obstacle-aware path planning
- Person re-ID: BoTSORT, cosine similarity (threshold 0.75), persistent gallery
- Web gallery: thumbnails, live updates, one-click follow, inline rename
- Session auth: /login, 7-day cookie, WS auth via 30s one-time token
- YDLIDAR X3: 360° radar + obstacle avoidance via SLAM bridge
- SLAM: ROS2 ydlidar_ros2_driver → SLAM Toolbox → slam_bridge.py TCP 9997
- Odometry auto-plug-and-play: slam_bridge.py activates odom when encoders fresh
- **Behavior architecture**: BehaviorCoordinator state machine with FollowBehavior, MissionBehavior, StoppedBehavior, ManualBehavior. Replaces old movement.cpp
- **Generic sensor slots**: SensorStore.setGeneric(key, value) — auto-serialized to status JSON
- **Nav2 path planning**: SmacPlanner2D on SLAM costmap. slam_bridge.py hosts ComputePathToPose action client. C++ requests paths via TCP, gets GPS sub-waypoints back
- **IMU/GPS → ROS2**: comms.cpp forwards sensor data to slam_bridge, which publishes /imu/data and /gps/fix
- **Dynamic odom→base_link TF**: Published by slam_bridge (or EKF when enabled) instead of static identity
- **GPS↔map conversion**: Tangent-plane approximation with heading offset rotation in slam_bridge.py
- **GPS heading auto-calibration**: EMA filter comparing GPS course to IMU yaw when moving
- **robot_localization EKF**: Fuses wheel odometry + IMU (config ready, use `--no-ekf` until encoders wired)
- **Unified launcher**: `start_tank.sh` starts all 6 subsystems, one Ctrl+C stops everything

## Pending / Not Yet Wired
- Wheel encoders — sketch ready, hardware not yet wired. **Use `--no-ekf` flag until wired** (EKF has no velocity source without them)
- PTU IMU — control Teensy sketch has it on Serial5; `controlRxLoop` parses `PTU_YPR` but this was reverted from deployed binary pending FPS fix
- RTAB-Map 3D mapping via ZMQ port 5557 (stereo_depth.cpp publishes rectified pairs)

## Architecture: Behavior System
Behaviors are registered with BehaviorCoordinator and activated via `requestTransition(ControlMode)`. Transitions are deferred to the coordinator's loop thread via atomic exchange — no mutex needed. Every transition: `onExit()` → zero all actuators → `onEnter()`.

**Adding a new behavior:**
1. Add enum to `include/control_mode.h`
2. Create `new_behavior.cpp` + header implementing `Behavior` interface
3. Register in `main.cpp`: `coordinator.addBehavior(ControlMode::NEW, &newBehavior);`
4. Add web command in `comms.cpp`: `coordinator_->requestTransition(ControlMode::NEW);`

**Adding a new sensor:**
1. In comms.cpp sensor callback: `sensors_.setGeneric("name", value);`
2. Auto-appears in status JSON under `sensors.name`

## Architecture: Nav2 Path Planning
Nav2 runs as a **planning service** alongside SLAM Toolbox. The C++ app keeps wheel control — Nav2 only answers "what's the best route?"

**Data flow:** C++ sends `plan_path` JSON → slam_bridge.py → Nav2 `ComputePathToPose` action → planned path → subsampled to GPS waypoints every ~1m → returned as `planned_path` JSON → C++ navigates sub-waypoints sequentially.

**Replanning triggers:** Path age > 10s, `Nav2Planner::isPathValid()` returns false (obstacle in path, robot deviation > 3m, path stale > 30s).

**Fallback:** If Nav2 is unavailable, `Nav2Planner::planPath()` falls back to direct GPS (same as `DirectGPSPlanner`).

**TF tree:** `map → odom → base_link → laser_frame`. SLAM Toolbox publishes `map → odom`, slam_bridge (or EKF) publishes `odom → base_link`, static TF for `base_link → laser_frame`.

## ROS2 Dependencies (Jetson)
```bash
sudo apt install ros-humble-nav2-planner ros-humble-nav2-costmap-2d \
  ros-humble-nav2-lifecycle-manager ros-humble-nav2-msgs \
  ros-humble-robot-localization
```
Plus existing: `ros-humble-slam-toolbox`, `ydlidar_ros2_driver` (built from source in `~/ros2_ws`)

## Feature Roadmap
**Safety-critical (before autonomous outdoor driving):**
- [ ] **Watchdog failsafe** — Control Teensy auto-stops motors after 500ms no heartbeat from Jetson. Prevents runaway if C++ app crashes
- [ ] **Geofence** — Virtual boundary (circle/polygon) checked in `navigateTo()`, abort mission if breached. Draw on mission map UI
- [ ] **Connection-loss behavior** — Configurable policy (continue / pause / return home) when comms drop. Layered: 4G lost but LoRa up → pause; both lost → RTH

**Comms upgrade (replace WiFi for field use):**
- [ ] **4G/5G modem** — Primary internet link via USB dongle or HAT + SIM. Drop-in WiFi replacement, existing VPS relay architecture works unchanged
- [ ] **LoRa backup channel** — Low-bandwidth safety net (300bps–50kbps, 2-15km range). Emergency stop, GPS beacon, heartbeat, RTH trigger. Works where cell signal doesn't
- [ ] **LoRa base station** — ESP32 or Teensy with LoRa module + small display as handheld controller showing GPS position + battery + emergency stop button
- [ ] **LoRa integration on Jetson side** — LoRa module via SPI on Sensor Teensy, or dedicated ESP32 radio bridge independent of main system

**High-value operational features:**
- [ ] **Return to home** — Save start GPS on launch, trigger RTH on low battery, connection loss, or operator command
- [ ] **Battery monitoring** — Voltage divider → Sensor Teensy ADC pin, low-voltage alerts + auto-RTH threshold
- [ ] **Mission progress readout** — Broadcast distance to waypoint, bearing error, ETA as `mission_progress` event to web UI
- [x] **Jetson system stats** — SystemMonitor thread reads sysfs every 2s, pushes CPU/GPU temp, CPU/GPU usage, RAM via setGeneric(). SYSTEM card on web dashboard
- [ ] **Telemetry logging** — GPS track, sensor readings, mode transitions, obstacle events to `/home/james/tank_logs/`
- [ ] **Video recording** — Save detection frames during active missions for post-mission replay
- [ ] **Run log viewer** — Serve `/tmp/robot_run.csv` via web UI for post-mission review
- [ ] **OTA config update** — Web UI panel for live PID gains, speed limits, obstacle threshold tuning without SSH

**Behaviour / detection features:**
- [ ] **Multi-person queue** — When active follow target lost for >Ns, auto-switch to next known person in gallery
- [ ] **Multi-target awareness** — Patrol + alert mode monitoring all detected people, notifying operator
- [ ] **Person heatmap** — Log gallery person sightings with GPS, overlay on mission map as heatmap
- [ ] **Night/low-light mode** — Switch to FLIR thermal cam2 for detection when visible-light confidence drops. IR illuminator as secondary option
- [ ] **Voice/text alert via Claude** — On known person detection during mission, push notification to browser via Haiku API call

**Additional sensors (hardware acquired):**
- [ ] **BME680 (temp/humidity/pressure/gas)** — Adafruit STEMMA QT, I2C. Environmental monitoring + air quality index. Wire to Sensor Teensy I2C
- [ ] **MR60BHA2 60GHz mmWave** — Vitals check on downed persons. UART 115200. Stop-and-scan only (won't work while moving — vibration swamps micro-Doppler). Stop near YOLO-detected person, hold still 15-20s, read breathing rate + heart rate. Works through smoke/dust/fog. Range: ~1.5m breathing, ~0.5m heartbeat. Single target, 60° FoV. Does NOT penetrate concrete/rubble (need UWB for that)
- [ ] **MQ-2 gas sensor** — LPG, propane, hydrogen detection. Analog output → Sensor Teensy ADC
- [ ] **MQ-7 gas sensor** — Carbon monoxide detection. Analog output → Sensor Teensy ADC
- [ ] **Voice localization mic array** — 4x INMP441 I2S MEMS mics at chassis corners (30-40 cm spacing). Bandpass 200-2 kHz, GCC-PHAT TDOA for bearing estimation. Wire to dedicated Teensy or Sensor Teensy I2S. Process with ODAS on Jetson or compute TDOA on Teensy. Output azimuth via setGeneric("voice_azimuth_deg"). Wide spacing optimised for outdoor use in voice band (200 Hz - 2 kHz). Effective range: 2-4m outdoors with motors stopped. NI cDAQ-9171/9222 not viable (no ARM SDK)

**Hardware / mechanical:**
- [ ] **PTU homing + position feedback** — Limit switch homing sequence on startup, display pan/tilt angle in UI
- [ ] **RTAB-Map 3D mapping** — Full 3D SLAM using stereo depth (ZMQ port 5557 already publishes rectified pairs)
- [ ] **Nav2 controller (DWB/MPPI)** — Replace proportional steering with velocity-space trajectory optimisation. Separate project

**Far future:**
- [ ] **Multi-tank fleet** — Inter-vehicle comms, fleet coordination. Current TCP architecture extends naturally to this

## Known Issues / Gotchas
- **Stereo camera modes**: Camera is opened at 2560×720 MJPEG. Full frame → stereo depth, left half (1280×720) → detection. **2560×720 MJPEG only supports 30fps** — requesting any other fps causes V4L2 to fall back to raw YUV at 3fps. Always use `CAP_PROP_FPS, 30` for this resolution. Calibration: `/home/james/stereo_calib/stereo_params_cuda.xml`.
- **Rebuilding = recompiles everything**: `cmake --build build` recompiles all changed source files, not just comms.cpp. If object_detection.cpp has in-progress changes they will be compiled in too.
- **PTU_YPR parsing**: Added to local `comms.cpp` `controlRxLoop` but currently reverted from Jetson until FPS issue is resolved.
- **EKF without encoders**: Don't run robot_localization EKF until wheel encoders are physically wired — EKF has no velocity/position source without them. Use `start_tank.sh --no-ekf`.
- **GPS cold start drift**: navsat_transform_node logs datum recalculation warnings for 2-5 minutes after GPS cold start as altitude/position converge. This is normal — it stabilizes.
- **Nav2 costmap node**: Do NOT launch a standalone `nav2_costmap_2d` node — `planner_server` creates its own costmap internally. The lifecycle manager only needs to manage `['planner_server']`.
- **slam_bridge.py init order**: `_dyn_tf_broadcaster` must be created before any call to `_publish_odom_tf()` in `__init__`.

## Key Technical Notes
- BNO085: Adafruit_BNO08x supports **one UART instance only** (global HAL state). Use one per Teensy.
- BNO085 init: `Serial.begin(115200)` → `Serial.setTimeout(0)` → `begin_UART()` → `enableReport(SH2_ROTATION_VECTOR, 20)`. Check `wasReset()` every loop and re-enable reports.
- NativeEthernet must init **before** UART on Teensy 4.1 (DMA channel conflict).
- YDLIDAR X3 working ROS2 params: `X3.yaml` (baud=115200, single_channel=true, fixed_resolution=true). Not `ydlidar.yaml`.
- slam_bridge.py tunable robot params: `WHEEL_DIAMETER_M`, `TRACK_WIDTH_M`, `ENC_CPR`
- `#undef PI` before `#define PI` before Adafruit BNO08x include (conflicts with math.h)
