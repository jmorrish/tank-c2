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
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

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

        self._tf_broadcaster.sendTransform([
            make_static_tf('base_link', 'laser_frame', z=0.02),
            make_static_tf('odom',      'base_link'),   # identity; needed for TF chain
        ])

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
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
        if msg.get('type') == 'odom_data':
            self._process_odom(msg)

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
