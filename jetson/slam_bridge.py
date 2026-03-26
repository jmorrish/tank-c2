#!/usr/bin/env python3
"""
slam_bridge.py — SLAM ↔ C++ app bridge.

Topology:
  ydlidar_ros2_driver → /scan → SLAM Toolbox → /pose
                            ↓
                      slam_bridge.py  (TCP server :9997)
                            ↕
                        C++ app (client)

Messages bridge → C++ app:
  {"type":"scan_full","obs":bool,"fwd":float,"pts":[[deg,m],...]}
  {"type":"slam_pose","x":float,"y":float,"theta":float}
  {"type":"slam_map","png":"<base64>","res":float,"ox":float,"oy":float,"w":int,"h":int}

Messages C++ app → bridge:
  {"type":"odom_data","left":int,"right":int,"age_ms":float}
    → published as nav_msgs/Odometry on /odom when age_ms < 200
  {"type":"imu_data","yaw":float,"pitch":float,"roll":float}
    → published as sensor_msgs/Imu on /imu/data
  {"type":"gps_data","lat":float,"lon":float,"alt":float,"quality":int,"sats":int}
    → published as sensor_msgs/NavSatFix on /gps/fix
  {"type":"gps_rmc","speed_knots":float,"course_deg":float}
    → used for GPS heading auto-calibration
  {"type":"plan_path","from_lat":float,"from_lon":float,"to_lat":float,"to_lon":float}
    → requests Nav2 ComputePathToPose, returns planned_path response

Odometry state machine:
  ODOM_INACTIVE → ODOM_ACTIVE when 5 consecutive fresh encoder reads
  ODOM_ACTIVE → ODOM_INACTIVE when 3 consecutive stale reads
  On each transition: save map, kill SLAM Toolbox, restart with matching params yaml

Usage:
  python3 slam_bridge.py
  (best started from slam_start.sh after ydlidar driver and ROS2 env are sourced)
"""

# ── Tunable robot parameters ────────────────────────────────────────────────
WHEEL_DIAMETER_M = 0.10    # metres
TRACK_WIDTH_M    = 0.30    # metres (centre-to-centre)
ENC_CPR          = 1000    # encoder counts per revolution

# ── SLAM Toolbox process management ─────────────────────────────────────────
SLAM_PARAMS_NO_ODOM = '/home/james/YOLOv8-TensorRT/csrc/jetson/detect/slam_toolbox_params_no_odom.yaml'
SLAM_PARAMS_ODOM    = '/home/james/YOLOv8-TensorRT/csrc/jetson/detect/slam_toolbox_params_odom.yaml'
SLAM_MAP_SAVE_PATH  = '/home/james/slam_map_autosave'
SLAM_LAUNCH_CMD     = 'ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false slam_params_file:={params}'

# Consecutive read counts required for state transitions
FRESH_RUNS_TO_ACTIVATE   = 5   # ≥0.5 s of fresh data → enable odometry
STALE_RUNS_TO_DEACTIVATE = 3   # 3 stale reads in a row → disable odometry

# ── Map rendering ────────────────────────────────────────────────────────────
MAP_SEND_INTERVAL_S = 1.0   # how often to send the occupancy grid (seconds)
MAP_MAX_CELLS       = 512   # cap longest axis at this many cells before downsampling

# ── Imports ──────────────────────────────────────────────────────────────────
import rclpy
import socket
import threading
import json
import math
import time
import subprocess
import os
import base64
import io

import numpy as np
from PIL import Image

from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

try:
    from nav2_msgs.action import ComputePathToPose
    from rclpy.action import ActionClient
    HAS_NAV2 = True
except ImportError:
    HAS_NAV2 = False

try:
    import tf_transformations
    HAS_TF = True
except ImportError:
    HAS_TF = False


class SlamBridge(Node):
    def __init__(self):
        super().__init__('slam_bridge')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publish required static TFs so SLAM Toolbox's message filter can resolve the
        # full chain: odom → base_link → laser_frame
        self._tf_broadcaster = StaticTransformBroadcaster(self)
        # Stamp must be 0 (epoch) for static transforms — makes them valid at all times.
        # Using now() causes lookupTransform at a specific scan timestamp to fail.
        def make_static_tf(parent, child, z=0.0):
            t = TransformStamped()
            t.header.stamp.sec     = 0
            t.header.stamp.nanosec = 0
            t.header.frame_id      = parent
            t.child_frame_id       = child
            t.transform.translation.z = z
            t.transform.rotation.w    = 1.0
            return t

        # Only base_link → laser_frame is truly static.
        # odom → base_link is now published dynamically (by EKF or by _publish_odom_tf).
        self._tf_broadcaster.sendTransform([
            make_static_tf('base_link', 'laser_frame', z=0.02),
        ])

        # Dynamic TF broadcaster for odom → base_link (used when EKF is not running)
        self._dyn_tf_broadcaster = TransformBroadcaster(self)

        # Publish an initial identity odom → base_link so TF tree is complete
        # before EKF starts or before first encoder data arrives.
        self._publish_odom_tf(0.0, 0.0, 0.0, self.get_clock().now().to_msg())

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub  = self.create_publisher(Imu, '/imu/data', 10)
        self.gps_pub  = self.create_publisher(NavSatFix, '/gps/fix', 10)

        # Nav2 path planning action client
        self._nav2_client = None
        if HAS_NAV2:
            self._nav2_client = ActionClient(
                self, ComputePathToPose, 'compute_path_to_pose')
            self.get_logger().info('Nav2 ComputePathToPose action client created')
        else:
            self.get_logger().warn('nav2_msgs not installed — path planning disabled')

        # GPS reference point for GPS↔map conversion (first valid fix)
        self._gps_ref = None       # (lat, lon) or None
        self._gps_ref_map = None   # (map_x, map_y) at reference time

        # Heading calibration state
        self._heading_offset = 0.0
        self._heading_calibrated = False
        self._last_imu_yaw_rad = 0.0

        self.create_subscription(LaserScan, '/scan', self.on_scan, qos)
        self.create_subscription(
            PoseWithCovarianceStamped, '/pose', self.on_pose, 10)
        # /map is published TRANSIENT_LOCAL (latched) by SLAM Toolbox — must match
        from rclpy.qos import DurabilityPolicy
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)

        # Connected C++ app socket
        self._client      = None
        self._client_lock = threading.Lock()
        self._last_map_time = 0.0
        self._last_gps_pub_time = 0.0   # throttle GPS publishing to 1 Hz

        # Odometry state machine
        self._odom_state       = 'INACTIVE'   # 'INACTIVE' | 'ACTIVE'
        self._fresh_run        = 0
        self._stale_run        = 0
        self._enc_prev_left    = None
        self._enc_prev_right   = None
        self._odom_x           = 0.0
        self._odom_y           = 0.0
        self._odom_theta       = 0.0
        self._odom_lock        = threading.Lock()

        # SLAM Toolbox subprocess
        self._slam_proc        = None
        self._slam_proc_lock   = threading.Lock()

        # Start TCP server in background thread
        threading.Thread(target=self._serve, daemon=True).start()

        # Start SLAM Toolbox via a timer so it fires AFTER rclpy.spin() has started.
        # sendTransform() above only queues the TF internally — it isn't actually published
        # until the event loop runs. Waiting 3s inside a timer callback guarantees the TF
        # is in the graph before SLAM Toolbox's message filter initialises.
        self._start_timer = self.create_timer(3.0, self._on_start_timer)

        self.get_logger().info('slam_bridge ready')

    def _on_start_timer(self):
        """One-shot timer: fires after spin starts so TF is propagated before SLAM init."""
        self._start_timer.cancel()
        self.get_logger().info('Static TF propagated — starting SLAM Toolbox now')
        self._start_slam(use_odom=False)

    # ── TCP server ────────────────────────────────────────────────────────────

    def _serve(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('127.0.0.1', 9997))
        srv.listen(1)
        self.get_logger().info('SLAM bridge listening on TCP 9997')
        while True:
            try:
                conn, addr = srv.accept()
                self.get_logger().info(f'C++ app connected from {addr}')
                with self._client_lock:
                    if self._client:
                        try: self._client.close()
                        except Exception: pass
                    self._client = conn
                threading.Thread(target=self._handle_client,
                                 args=(conn,), daemon=True).start()
            except Exception as e:
                self.get_logger().error(f'serve() error: {e}')
                time.sleep(1)

    def _handle_client(self, conn):
        buf = ''
        try:
            while True:
                data = conn.recv(4096)
                if not data:
                    break
                buf += data.decode('utf-8', errors='replace')
                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    line = line.strip()
                    if line:
                        try:
                            self._on_cpp_message(json.loads(line))
                        except Exception:
                            pass
        except Exception:
            pass
        finally:
            with self._client_lock:
                if self._client is conn:
                    self._client = None
            try: conn.close()
            except Exception: pass
            self.get_logger().info('C++ app disconnected')
            # Reset odom state on disconnect
            self._fresh_run = 0
            self._stale_run = 0

    def _send_to_cpp(self, obj: dict):
        msg = json.dumps(obj) + '\n'
        with self._client_lock:
            if self._client:
                try:
                    self._client.sendall(msg.encode())
                except Exception:
                    pass

    # ── ROS2 subscribers ──────────────────────────────────────────────────────

    def on_scan(self, scan_msg: LaserScan):
        """Convert /scan to scan_full JSON and push to C++ app."""
        pts = []
        for i, r in enumerate(scan_msg.ranges):
            if scan_msg.range_min < r < scan_msg.range_max:
                angle_deg = math.degrees(
                    scan_msg.angle_min + i * scan_msg.angle_increment)
                # Normalise to [0, 360)
                angle_deg = angle_deg % 360.0
                pts.append([round(angle_deg, 1), round(float(r), 2)])

        fwd_arc = 30.0
        fwd_dists = [d for a, d in pts
                     if d > 0 and (a <= fwd_arc or a >= 360.0 - fwd_arc)]
        fwd = round(min(fwd_dists), 2) if fwd_dists else -1.0
        obs = 0.0 < fwd < 0.5

        self._send_to_cpp({'type': 'scan_full', 'obs': obs,
                           'fwd': fwd, 'pts': pts})

    def on_pose(self, pose_msg: PoseWithCovarianceStamped):
        """Convert /pose to slam_pose JSON and push to C++ app."""
        p = pose_msg.pose.pose
        q = p.orientation
        if HAS_TF:
            _, _, theta = tf_transformations.euler_from_quaternion(
                [q.x, q.y, q.z, q.w])
        else:
            # Simplified yaw extraction from quaternion
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            theta = math.atan2(siny_cosp, cosy_cosp)

        self._send_to_cpp({
            'type':  'slam_pose',
            'x':     round(p.position.x, 3),
            'y':     round(p.position.y, 3),
            'theta': round(theta, 4),
        })

    def on_map(self, map_msg: OccupancyGrid):
        """Convert /map occupancy grid to PNG and push to C++ app (throttled)."""
        now = time.time()
        if now - self._last_map_time < MAP_SEND_INTERVAL_S:
            return
        self._last_map_time = now

        w = map_msg.info.width
        h = map_msg.info.height
        if w == 0 or h == 0:
            return

        # Convert flat int8 array to 2-D numpy, flip Y (ROS row-0 = bottom)
        data = np.array(map_msg.data, dtype=np.int8).reshape((h, w))
        data = np.flipud(data)

        # Downsample if largest dimension exceeds MAP_MAX_CELLS
        scale_factor = 1
        if max(w, h) > MAP_MAX_CELLS:
            scale_factor = math.ceil(max(w, h) / MAP_MAX_CELLS)
            data = data[::scale_factor, ::scale_factor]
            h2, w2 = data.shape
        else:
            h2, w2 = h, w

        # Map cell values → RGB (dark UI theme)
        #   -1 unknown → dark blue-grey
        #    0 free    → very dark (near background)
        #  1-100 occ   → cyan scaled by probability
        img = np.zeros((h2, w2, 3), dtype=np.uint8)
        unknown  = (data == -1)
        free     = (data == 0)
        occ_mask = (data > 0)

        img[unknown] = (35, 50, 65)
        img[free]    = (12, 26, 40)
        if occ_mask.any():
            prob = data[occ_mask].astype(np.float32) / 100.0
            img[occ_mask, 0] = 0
            img[occ_mask, 1] = (prob * 229).astype(np.uint8)
            img[occ_mask, 2] = (prob * 255).astype(np.uint8)

        # Encode as PNG
        pil_img = Image.fromarray(img, 'RGB')
        buf = io.BytesIO()
        pil_img.save(buf, format='PNG', optimize=True)
        png_b64 = base64.b64encode(buf.getvalue()).decode('ascii')

        self._send_to_cpp({
            'type': 'slam_map',
            'png':  png_b64,
            'res':  round(map_msg.info.resolution * scale_factor, 4),
            'ox':   round(map_msg.info.origin.position.x, 3),
            'oy':   round(map_msg.info.origin.position.y, 3),
            'w':    w2,
            'h':    h2,
        })

    # ── C++ app → bridge ──────────────────────────────────────────────────────

    def _on_cpp_message(self, msg: dict):
        msg_type = msg.get('type')
        if msg_type == 'odom_data':
            self._process_odom(msg)
        elif msg_type == 'imu_data':
            self._publish_imu(msg)
        elif msg_type == 'gps_data':
            self._publish_gps(msg)
        elif msg_type == 'gps_rmc':
            self._process_heading(msg)
        elif msg_type == 'plan_path':
            threading.Thread(target=self._handle_plan_path,
                             args=(msg,), daemon=True).start()

    def _process_odom(self, msg: dict):
        left   = int(msg['left'])
        right  = int(msg['right'])
        age_ms = float(msg['age_ms'])

        if age_ms > 200.0:
            # Stale data
            self._fresh_run = 0
            self._stale_run += 1
            if self._stale_run >= STALE_RUNS_TO_DEACTIVATE and self._odom_state == 'ACTIVE':
                self.get_logger().warn(
                    'Odometry lost (stale encoders) — restarting SLAM without odometry')
                self._odom_state = 'INACTIVE'
                self._restart_slam(use_odom=False)
            return

        # Fresh data
        self._stale_run = 0
        self._fresh_run += 1
        if self._fresh_run >= FRESH_RUNS_TO_ACTIVATE and self._odom_state == 'INACTIVE':
            self.get_logger().info(
                'Odometry active — restarting SLAM with wheel odometry')
            self._odom_state = 'ACTIVE'
            self._restart_slam(use_odom=True)

        # Publish /odom
        self._integrate_and_publish(left, right)

    def _integrate_and_publish(self, left: int, right: int):
        wheel_circ = math.pi * WHEEL_DIAMETER_M

        with self._odom_lock:
            if self._enc_prev_left is None:
                self._enc_prev_left  = left
                self._enc_prev_right = right
                return

            dl = (left  - self._enc_prev_left)  / ENC_CPR * wheel_circ
            dr = (right - self._enc_prev_right) / ENC_CPR * wheel_circ
            self._enc_prev_left  = left
            self._enc_prev_right = right

            dc    = (dl + dr) / 2.0
            dtheta = (dr - dl) / TRACK_WIDTH_M

            self._odom_theta += dtheta
            self._odom_x     += dc * math.cos(self._odom_theta)
            self._odom_y     += dc * math.sin(self._odom_theta)

            x, y, theta = self._odom_x, self._odom_y, self._odom_theta

        odom = Odometry()
        now  = self.get_clock().now().to_msg()
        odom.header.stamp    = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y

        # Quaternion from yaw
        odom.pose.pose.orientation.z = math.sin(theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(theta / 2.0)

        self.odom_pub.publish(odom)

        # Publish dynamic odom → base_link TF (fallback when EKF is not running)
        self._publish_odom_tf(x, y, theta, now)

    def _publish_odom_tf(self, x, y, theta, stamp):
        """Publish dynamic odom → base_link transform."""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = math.sin(theta / 2.0)
        t.transform.rotation.w = math.cos(theta / 2.0)
        self._dyn_tf_broadcaster.sendTransform(t)

    # ── SLAM Toolbox lifecycle ─────────────────────────────────────────────────

    def _start_slam(self, use_odom: bool):
        params = SLAM_PARAMS_ODOM if use_odom else SLAM_PARAMS_NO_ODOM
        cmd    = SLAM_LAUNCH_CMD.format(params=params)
        self.get_logger().info(f'Starting SLAM Toolbox: {cmd}')
        log = open('/tmp/slam_toolbox.log', 'w')
        with self._slam_proc_lock:
            self._slam_proc = subprocess.Popen(
                cmd, shell=True,
                stdout=log, stderr=log)

    def _stop_slam(self):
        with self._slam_proc_lock:
            if self._slam_proc and self._slam_proc.poll() is None:
                self._slam_proc.terminate()
                try:
                    self._slam_proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self._slam_proc.kill()
                self._slam_proc = None

    def _save_map(self):
        self.get_logger().info(f'Saving SLAM map to {SLAM_MAP_SAVE_PATH}')
        try:
            subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-f', SLAM_MAP_SAVE_PATH],
                timeout=10, check=False)
        except Exception as e:
            self.get_logger().warn(f'Map save failed: {e}')

    def _restart_slam(self, use_odom: bool):
        """Save map, kill SLAM, restart with new params."""
        self._save_map()
        self._stop_slam()
        time.sleep(1)   # let ROS2 graph settle
        self._start_slam(use_odom=use_odom)
        # Reset dead-reckoning state
        with self._odom_lock:
            self._enc_prev_left  = None
            self._enc_prev_right = None
            self._odom_x = self._odom_y = self._odom_theta = 0.0


    # ── IMU publishing ─────────────────────────────────────────────────────────

    def _publish_imu(self, msg: dict):
        """Publish BNO085 YPR as sensor_msgs/Imu on /imu/data."""
        yaw_deg   = msg.get('yaw', 0.0)
        pitch_deg = msg.get('pitch', 0.0)
        roll_deg  = msg.get('roll', 0.0)

        yaw   = math.radians(yaw_deg)
        pitch = math.radians(pitch_deg)
        roll  = math.radians(roll_deg)

        # Store for heading calibration
        self._last_imu_yaw_rad = yaw

        # Convert Euler (ZYX) to quaternion
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'base_link'
        imu.orientation.x = sr * cp * cy - cr * sp * sy
        imu.orientation.y = cr * sp * cy + sr * cp * sy
        imu.orientation.z = cr * cp * sy - sr * sp * cy
        imu.orientation.w = cr * cp * cy + sr * sp * sy
        # BNO085 orientation accuracy ~0.01 rad (~0.5°)
        imu.orientation_covariance[0] = 0.01
        imu.orientation_covariance[4] = 0.01
        imu.orientation_covariance[8] = 0.01
        self.imu_pub.publish(imu)

    # ── GPS publishing ────────────────────────────────────────────────────────

    def _publish_gps(self, msg: dict):
        """Publish GPS fix as sensor_msgs/NavSatFix on /gps/fix (throttled to 1 Hz)."""
        now = time.time()
        if now - self._last_gps_pub_time < 1.0:
            return
        self._last_gps_pub_time = now

        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'gps_link'
        fix.latitude  = msg.get('lat', 0.0)
        fix.longitude = msg.get('lon', 0.0)
        fix.altitude  = msg.get('alt', 0.0)

        quality = msg.get('quality', 0)
        fix.status.status = (NavSatStatus.STATUS_FIX
                             if quality > 0 else NavSatStatus.STATUS_NO_FIX)
        fix.status.service = NavSatStatus.SERVICE_GPS

        # Position covariance — ~2.5m CEP for typical GPS
        fix.position_covariance[0] = 2.5   # lat variance
        fix.position_covariance[4] = 2.5   # lon variance
        fix.position_covariance[8] = 5.0   # alt variance
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.gps_pub.publish(fix)

        # Set GPS reference on first valid fix (for GPS↔map conversion)
        if self._gps_ref is None and quality > 0:
            self._gps_ref = (fix.latitude, fix.longitude)
            # Capture current SLAM pose as map reference
            # (will be (0,0) if SLAM hasn't produced a pose yet — that's OK)
            with self._odom_lock:
                self._gps_ref_map = (self._odom_x, self._odom_y)
            self.get_logger().info(
                f'GPS reference set: ({fix.latitude:.7f}, {fix.longitude:.7f})')

    # ── Heading calibration ───────────────────────────────────────────────────

    def _process_heading(self, msg: dict):
        """Auto-calibrate GPS↔IMU heading offset when moving with GPS fix."""
        speed_knots = msg.get('speed_knots', 0.0)
        course_deg  = msg.get('course_deg', 0.0)

        # Only calibrate when actually moving (>0.5 knots ≈ 0.26 m/s)
        if speed_knots < 0.5:
            return

        # GPS course: 0=north, 90=east (CW from north)
        # Convert to math convention: 0=east, CCW positive
        gps_heading_rad = math.radians(90.0 - course_deg)

        offset = gps_heading_rad - self._last_imu_yaw_rad
        # Normalise to [-pi, pi]
        offset = math.atan2(math.sin(offset), math.cos(offset))

        # Exponential moving average
        alpha = 0.05
        if not self._heading_calibrated:
            self._heading_offset = offset
            self._heading_calibrated = True
            self.get_logger().info(
                f'Heading calibration initial: offset={math.degrees(offset):.1f}°')
        else:
            self._heading_offset = (self._heading_offset * (1 - alpha)
                                    + offset * alpha)

    # ── GPS ↔ map coordinate conversion ───────────────────────────────────────

    def _gps_to_map(self, lat, lon):
        """Convert GPS lat/lon to map frame (x, y) using tangent plane approx."""
        if self._gps_ref is None:
            return None, None
        ref_lat, ref_lon = self._gps_ref
        ref_x, ref_y = self._gps_ref_map or (0.0, 0.0)

        # Local tangent plane (metres per degree at reference latitude)
        m_per_deg_lat = 111132.92
        m_per_deg_lon = 111132.92 * math.cos(math.radians(ref_lat))

        dx = (lon - ref_lon) * m_per_deg_lon
        dy = (lat - ref_lat) * m_per_deg_lat

        # Apply heading offset to rotate GPS frame into map frame
        cos_h = math.cos(self._heading_offset)
        sin_h = math.sin(self._heading_offset)
        map_x = ref_x + dx * cos_h - dy * sin_h
        map_y = ref_y + dx * sin_h + dy * cos_h

        return map_x, map_y

    def _map_to_gps(self, map_x, map_y):
        """Convert map frame (x, y) back to GPS lat/lon."""
        if self._gps_ref is None:
            return None, None
        ref_lat, ref_lon = self._gps_ref
        ref_x, ref_y = self._gps_ref_map or (0.0, 0.0)

        m_per_deg_lat = 111132.92
        m_per_deg_lon = 111132.92 * math.cos(math.radians(ref_lat))

        # Undo heading rotation
        dx = map_x - ref_x
        dy = map_y - ref_y
        cos_h = math.cos(-self._heading_offset)
        sin_h = math.sin(-self._heading_offset)
        dx_gps = dx * cos_h - dy * sin_h
        dy_gps = dx * sin_h + dy * cos_h

        lat = ref_lat + dy_gps / m_per_deg_lat
        lon = ref_lon + dx_gps / m_per_deg_lon
        return lat, lon

    # ── Nav2 path planning ────────────────────────────────────────────────────

    def _handle_plan_path(self, msg: dict):
        """Handle plan_path request from C++ — calls Nav2 ComputePathToPose."""
        if not HAS_NAV2 or self._nav2_client is None:
            self._send_to_cpp({
                'type': 'planned_path', 'ok': False,
                'error': 'Nav2 not available (nav2_msgs not installed)'})
            return

        if self._gps_ref is None:
            self._send_to_cpp({
                'type': 'planned_path', 'ok': False,
                'error': 'No GPS reference yet'})
            return

        if not self._nav2_client.wait_for_server(timeout_sec=2.0):
            self._send_to_cpp({
                'type': 'planned_path', 'ok': False,
                'error': 'Nav2 planner server not ready'})
            return

        # Convert GPS goal to map frame
        goal_x, goal_y = self._gps_to_map(msg['to_lat'], msg['to_lon'])
        if goal_x is None:
            self._send_to_cpp({
                'type': 'planned_path', 'ok': False,
                'error': 'GPS→map conversion failed'})
            return

        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = 'map'
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x = goal_x
        goal.goal.pose.position.y = goal_y
        goal.goal.pose.orientation.w = 1.0  # don't care about goal heading
        goal.use_start = False  # use current robot pose from TF

        try:
            future = self._nav2_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            goal_handle = future.result()

            if not goal_handle or not goal_handle.accepted:
                self._send_to_cpp({
                    'type': 'planned_path', 'ok': False,
                    'error': 'Goal rejected by Nav2'})
                return

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
            result = result_future.result()

            if result and result.result.path.poses:
                # Convert map-frame path back to GPS waypoints
                # Subsample: one waypoint every ~1m to avoid flooding C++
                waypoints = []
                prev = None
                for pose in result.result.path.poses:
                    x = pose.pose.position.x
                    y = pose.pose.position.y
                    if prev:
                        dist = math.hypot(x - prev[0], y - prev[1])
                        if dist < 1.0:
                            continue
                    lat, lon = self._map_to_gps(x, y)
                    if lat is not None:
                        waypoints.append([round(lat, 7), round(lon, 7)])
                    prev = (x, y)

                # Always include final point
                last = result.result.path.poses[-1].pose.position
                lat, lon = self._map_to_gps(last.x, last.y)
                if lat is not None:
                    final = [round(lat, 7), round(lon, 7)]
                    if not waypoints or waypoints[-1] != final:
                        waypoints.append(final)

                self.get_logger().info(
                    f'Nav2 planned path: {len(waypoints)} waypoints')
                self._send_to_cpp({
                    'type': 'planned_path', 'ok': True,
                    'waypoints': waypoints})
            else:
                self._send_to_cpp({
                    'type': 'planned_path', 'ok': False,
                    'error': 'No valid path found'})

        except Exception as e:
            self.get_logger().error(f'Nav2 planning error: {e}')
            self._send_to_cpp({
                'type': 'planned_path', 'ok': False,
                'error': str(e)})


def main():
    rclpy.init()
    node = SlamBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_slam()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
