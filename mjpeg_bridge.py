#!/usr/bin/env python3
"""
Tank C2 - MJPEG Bridge
Subscribes to the ZMQ frame publisher (tcp://127.0.0.1:5555) and
serves the latest frame as an MJPEG stream on http://0.0.0.0:8080/stream

Run on the Jetson:
  python3 mjpeg_bridge.py

Or as a systemd service (see README).
"""
import zmq
import time
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

ZMQ_ADDR  = "tcp://127.0.0.1:5555"
HTTP_PORT = 8080
TARGET_FPS = 25

ctx  = zmq.Context()
sock = ctx.socket(zmq.SUB)
sock.connect(ZMQ_ADDR)
sock.setsockopt(zmq.SUBSCRIBE, b"")
sock.setsockopt(zmq.RCVTIMEO, 1000)
sock.setsockopt(zmq.CONFLATE, 1)   # Keep only latest frame — no build-up

latest = None
lock   = threading.Lock()

def receiver():
    global latest
    print(f"[zmq] Connecting to {ZMQ_ADDR}")
    while True:
        try:
            data = sock.recv()
            with lock:
                latest = data
        except zmq.Again:
            pass
        except Exception as e:
            print(f"[zmq] Error: {e}")
            time.sleep(0.5)

threading.Thread(target=receiver, daemon=True).start()

BOUNDARY = b"--tankframe"
INTERVAL = 1.0 / TARGET_FPS

class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path not in ('/stream', '/stream/'):
            self.send_response(404)
            self.end_headers()
            return

        self.send_response(200)
        self.send_header('Content-Type', f'multipart/x-mixed-replace; boundary=tankframe')
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

        try:
            while True:
                t0 = time.monotonic()
                with lock:
                    frame = latest
                if frame:
                    try:
                        self.wfile.write(
                            BOUNDARY + b"\r\n"
                            b"Content-Type: image/jpeg\r\n"
                            b"Content-Length: " + str(len(frame)).encode() + b"\r\n\r\n" +
                            frame + b"\r\n"
                        )
                        self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError):
                        break
                elapsed = time.monotonic() - t0
                sleep = INTERVAL - elapsed
                if sleep > 0:
                    time.sleep(sleep)
        except Exception:
            pass

    def log_message(self, fmt, *args):
        pass  # suppress per-request logs

if __name__ == "__main__":
    print(f"[mjpeg] Bridge starting — serving on http://0.0.0.0:{HTTP_PORT}/stream")
    HTTPServer(('0.0.0.0', HTTP_PORT), Handler).serve_forever()
