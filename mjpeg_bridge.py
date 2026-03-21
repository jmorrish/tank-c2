#!/usr/bin/env python3
"""
Tank C2 - MJPEG Bridge
Subscribes to the ZMQ frame publisher (tcp://127.0.0.1:5555) and
serves the latest frame as an MJPEG stream on http://0.0.0.0:8080/stream

Fully self-recovering:
  - ZMQ receiver reconnects automatically if the C++ app restarts
  - HTTP server restarts if it crashes
  - Run via systemd for OS-level restart on Jetson reboot

Run on the Jetson:
  python3 mjpeg_bridge.py
"""
import zmq
import time
import threading
import socket
import logging
import os
import json
from http.server import BaseHTTPRequestHandler, HTTPServer, ThreadingHTTPServer

logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')
log = logging.getLogger('mjpeg')

ZMQ_ADDR       = "tcp://127.0.0.1:5555"
HTTP_PORT      = 8080
TARGETS_DIR    = "/home/james/tank_targets"
TARGET_FPS     = 25
STALE_TIMEOUT  = 5.0   # seconds without a frame before we consider ZMQ dead

INTERVAL  = 1.0 / TARGET_FPS
BOUNDARY  = b"--tankframe"

# Shared state
latest      = None
latest_time = 0.0
lock        = threading.Lock()


# ── ZMQ receiver — reconnects automatically when C++ app restarts ────────────

def zmq_receiver():
    """Runs forever. Reconnects to ZMQ whenever the publisher disappears."""
    global latest, latest_time
    ctx = None
    sock = None

    while True:
        try:
            if ctx is None:
                ctx = zmq.Context()
            if sock is None:
                log.info(f"Connecting to ZMQ at {ZMQ_ADDR}")
                sock = ctx.socket(zmq.SUB)
                sock.setsockopt(zmq.SUBSCRIBE, b"")
                sock.setsockopt(zmq.RCVTIMEO, 2000)   # 2 s receive timeout
                sock.setsockopt(zmq.CONFLATE, 1)       # keep only latest frame
                sock.setsockopt(zmq.LINGER, 0)
                sock.connect(ZMQ_ADDR)

            data = sock.recv()
            with lock:
                latest      = data
                latest_time = time.monotonic()

        except zmq.Again:
            # No frame within timeout — publisher may be gone, but ZMQ will
            # reconnect automatically when it comes back. Just keep looping.
            pass

        except zmq.ZMQError as e:
            log.warning(f"ZMQ error: {e} — resetting socket in 2s")
            try:
                sock.close()
            except Exception:
                pass
            sock = None
            time.sleep(2.0)

        except Exception as e:
            log.warning(f"Unexpected error in receiver: {e}")
            time.sleep(1.0)


threading.Thread(target=zmq_receiver, daemon=True, name="zmq-recv").start()


# ── MJPEG HTTP handler ────────────────────────────────────────────────────────

class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        # ── GET /targets → JSON list of known persons ─────────────────────────
        if self.path in ('/targets', '/targets/'):
            entries = []
            if os.path.isdir(TARGETS_DIR):
                for name in sorted(os.listdir(TARGETS_DIR)):
                    d = os.path.join(TARGETS_DIR, name)
                    if os.path.isdir(d) and name.isdigit():
                        label = f"Person {name}"
                        nf = os.path.join(d, 'name.txt')
                        if os.path.isfile(nf):
                            try:
                                label = open(nf).read().strip() or label
                            except Exception:
                                pass
                        entries.append({"id": int(name), "name": label,
                                        "thumb_url": f"/targets/{name}/thumb"})
            body = json.dumps(entries).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(body)
            return

        # ── GET /targets/<id>/thumb → JPEG thumbnail ──────────────────────────
        parts = self.path.rstrip('/').split('/')
        if (len(parts) == 4 and parts[1] == 'targets' and
                parts[3] == 'thumb' and parts[2].isdigit()):
            thumb = os.path.join(TARGETS_DIR, parts[2], 'thumb.jpg')
            if os.path.isfile(thumb):
                with open(thumb, 'rb') as f:
                    data = f.read()
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_response(404)
                self.end_headers()
            return

        if self.path not in ('/stream', '/stream/'):
            self.send_response(404)
            self.end_headers()
            return

        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=tankframe')
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

        try:
            while True:
                t0 = time.monotonic()

                with lock:
                    frame = latest
                    age   = t0 - latest_time if latest_time else STALE_TIMEOUT + 1

                if frame and age < STALE_TIMEOUT:
                    self.wfile.write(
                        BOUNDARY + b"\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        b"Content-Length: " + str(len(frame)).encode() + b"\r\n\r\n" +
                        frame + b"\r\n"
                    )
                    self.wfile.flush()

                elapsed = time.monotonic() - t0
                sleep   = INTERVAL - elapsed
                if sleep > 0:
                    time.sleep(sleep)

        except (BrokenPipeError, ConnectionResetError):
            pass   # client disconnected — normal
        except Exception:
            pass

    def do_POST(self):
        # ── POST /targets/<id>/name  body: {"name":"..."} ─────────────────────
        parts = self.path.rstrip('/').split('/')
        if (len(parts) == 4 and parts[1] == 'targets' and
                parts[3] == 'name' and parts[2].isdigit()):
            length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(length)
            try:
                data = json.loads(body)
                label = str(data.get('name', '')).strip()[:64]
                d = os.path.join(TARGETS_DIR, parts[2])
                if os.path.isdir(d) and label:
                    with open(os.path.join(d, 'name.txt'), 'w') as f:
                        f.write(label)
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.wfile.write(json.dumps({"ok": True}).encode())
                    return
            except Exception:
                pass
        self.send_response(400)
        self.end_headers()

    def log_message(self, fmt, *args):
        pass   # suppress per-request access logs


# ── Main — HTTP server with restart loop ─────────────────────────────────────

def run_server():
    while True:
        try:
            log.info(f"Starting MJPEG server on http://0.0.0.0:{HTTP_PORT}/stream")
            # SO_REUSEADDR so we can restart quickly without 'address in use'
            ThreadingHTTPServer.allow_reuse_address = True
            srv = ThreadingHTTPServer(('0.0.0.0', HTTP_PORT), Handler)
            srv.serve_forever()
        except OSError as e:
            log.error(f"Server error: {e} — retrying in 5s")
            time.sleep(5.0)
        except Exception as e:
            log.error(f"Unexpected server crash: {e} — retrying in 3s")
            time.sleep(3.0)


if __name__ == "__main__":
    run_server()
