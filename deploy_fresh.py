#!/usr/bin/env python3
"""
Tank C2 — Fresh Jetson Deployment Script
=========================================
Runs from Windows. Uploads ALL source files to a Jetson, runs setup_jetson.sh
(if --setup flag), then builds the C++ app.

Two modes:
  1. Full setup (first time):  python deploy_fresh.py --setup --host <ip> --user <user> --password <pass>
  2. Code deploy only:         python deploy_fresh.py --host <ip> --user <user> --password <pass>

The --setup flag runs setup_jetson.sh on the Jetson first (installs all deps,
builds OpenCV, ROS2, etc). This takes 3-4 hours on first run.

Without --setup, it just uploads source files and builds (like deploy_phase4.py
but uploads ALL files including ones that were missing from the regular deploy list).
"""
import paramiko
import argparse
import os
import sys
import stat
import time

# ── Configuration ──────────────────────────────────────────────────────────────

LOCAL_JETSON_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "jetson")
LOCAL_ROOT_DIR   = os.path.dirname(os.path.abspath(__file__))

# Remote paths (relative to user home)
YOLO_REL        = "YOLOv8-TensorRT"
DETECT_REL      = f"{YOLO_REL}/csrc/jetson/detect"

# ALL files to deploy from jetson/ directory
JETSON_FILES = [
    # ── C++ source ─────────────────────────────────────────────────────────
    "main.cpp",
    "object_detection.cpp",
    "behavior_coordinator.cpp",
    "follow_behavior.cpp",
    "mission_behavior.cpp",
    "nav2_planner.cpp",
    "comms.cpp",
    "teensy_link.cpp",
    "slam_link.cpp",
    "mission.cpp",
    "stereo_depth.cpp",
    "sensor_store.cpp",
    "system_monitor.cpp",
    "runtime_config.cpp",
    "logger.cpp",

    # ── C++ headers ────────────────────────────────────────────────────────
    "include/comms.h",
    "include/mission.h",
    "include/config.h",
    "include/behavior.h",
    "include/behavior_coordinator.h",
    "include/control_mode.h",
    "include/follow_behavior.h",
    "include/mission_behavior.h",
    "include/slam_link.h",
    "include/teensy_link.h",
    "include/nav2_planner.h",
    "include/path_planner.h",
    "include/obstacle_utils.h",
    "include/sensor_store.h",
    "include/helpers.h",
    "include/stereo_depth.h",
    "include/system_monitor.h",
    "include/runtime_config.h",
    "include/object_detection.h",
    "include/movement.h",
    "include/logger.h",
    "include/common.hpp",
    "include/filesystem.hpp",
    "include/yolov8.hpp",

    # ── Build ──────────────────────────────────────────────────────────────
    "CMakeLists.txt",

    # ── Configuration ──────────────────────────────────────────────────────
    "config.json",
    "nav2_params.yaml",
    "ekf_params.yaml",
    "slam_toolbox_params_odom.yaml",
    "slam_toolbox_params_no_odom.yaml",
    "99-tank-cameras.rules",

    # ── Python scripts ─────────────────────────────────────────────────────
    "slam_bridge.py",
    "nav2_planner_launch.py",

    # ── Shell scripts ──────────────────────────────────────────────────────
    "start_tank.sh",
    "slam_start.sh",
]

# Files from repo root that go to detect/ directory
ROOT_FILES = [
    "mjpeg_bridge.py",
]

# Files from repo root to upload alongside setup
SETUP_FILES = [
    "setup_jetson.sh",
]


def connect(host, user, password, port=22):
    """Connect via SSH and return (ssh, sftp) tuple."""
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    print(f"Connecting to {user}@{host}:{port}...")
    ssh.connect(host, port=port, username=user, password=password)
    sftp = ssh.open_sftp()
    return ssh, sftp


def ensure_remote_dir(sftp, path):
    """Recursively create remote directory if it doesn't exist."""
    dirs_to_create = []
    check = path
    while True:
        try:
            sftp.stat(check)
            break
        except FileNotFoundError:
            dirs_to_create.append(check)
            check = os.path.dirname(check)
            if not check or check == '/':
                break
    for d in reversed(dirs_to_create):
        try:
            sftp.mkdir(d)
        except IOError:
            pass  # race condition or already exists


def upload_files(sftp, home_dir, detect_dir):
    """Upload all project files to the Jetson."""
    ensure_remote_dir(sftp, f"{detect_dir}/include")

    uploaded = 0
    skipped = 0

    # Jetson directory files
    print(f"\nUploading {len(JETSON_FILES)} files from jetson/...")
    for f in JETSON_FILES:
        local = os.path.join(LOCAL_JETSON_DIR, f.replace("/", os.sep))
        remote = f"{detect_dir}/{f}"
        if not os.path.exists(local):
            print(f"  SKIP (not found locally): {f}")
            skipped += 1
            continue
        sftp.put(local, remote)
        print(f"  OK: {f}")
        uploaded += 1

    # Root files -> detect dir
    print(f"\nUploading {len(ROOT_FILES)} root files...")
    for f in ROOT_FILES:
        local = os.path.join(LOCAL_ROOT_DIR, f)
        remote = f"{detect_dir}/{f}"
        if not os.path.exists(local):
            print(f"  SKIP (not found locally): {f}")
            skipped += 1
            continue
        sftp.put(local, remote)
        print(f"  OK: {f}")
        uploaded += 1

    print(f"\n  Uploaded: {uploaded}  Skipped: {skipped}")
    return uploaded


def run_remote(ssh, cmd, timeout=600, stream=True):
    """Run a command on the remote host, optionally streaming output."""
    print(f"\n  Running: {cmd}")
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)

    if stream:
        # Stream output line by line
        while True:
            line = stdout.readline()
            if not line:
                break
            print(f"  | {line}", end="")
        err = stderr.read().decode()
        if err:
            print(f"  STDERR: {err[-2000:]}")
    else:
        out = stdout.read().decode()
        err = stderr.read().decode()
        if out:
            print(out[-3000:] if len(out) > 3000 else out)
        if err:
            print(f"  STDERR: {err[-2000:]}")

    exit_code = stdout.channel.recv_exit_status()
    return exit_code


def run_setup(ssh, sftp, home_dir, detect_dir, skip_opencv=False, skip_ros2=False):
    """Upload and run setup_jetson.sh on the remote host."""
    print("\n" + "=" * 60)
    print("  RUNNING FULL SETUP (this takes 3-4 hours first time)")
    print("=" * 60)

    # Upload setup script
    local_setup = os.path.join(LOCAL_ROOT_DIR, "setup_jetson.sh")
    remote_setup = f"{home_dir}/setup_jetson.sh"
    sftp.put(local_setup, remote_setup)
    sftp.chmod(remote_setup, stat.S_IRWXU | stat.S_IRGRP | stat.S_IXGRP)
    print("  Uploaded setup_jetson.sh")

    # Build flags
    flags = []
    if skip_opencv:
        flags.append("--skip-opencv")
    if skip_ros2:
        flags.append("--skip-ros2")
    flag_str = " ".join(flags)

    exit_code = run_remote(ssh, f"bash {remote_setup} {flag_str}", timeout=14400, stream=True)
    if exit_code != 0:
        print(f"\n  [ERROR] setup_jetson.sh exited with code {exit_code}")
        print("  Check the output above for errors. You can re-run with --skip-opencv")
        print("  or --skip-ros2 to skip already-completed steps.")
        return False

    print("\n  Setup completed successfully!")
    return True


def build_app(ssh, detect_dir):
    """Run cmake + make on the Jetson."""
    print("\n" + "=" * 60)
    print("  BUILDING C++ APP")
    print("=" * 60)

    build_dir = f"{detect_dir}/build"

    # Ensure build dir exists
    run_remote(ssh, f"mkdir -p {build_dir}", stream=False)

    # cmake configure
    print("\n  Running cmake...")
    exit_code = run_remote(ssh, f"cd {build_dir} && cmake .. 2>&1", timeout=120)
    if exit_code != 0:
        print("  [ERROR] cmake configure failed!")
        return False

    # cmake build
    print("\n  Building (this takes 2-5 minutes)...")
    exit_code = run_remote(ssh, f"cd {build_dir} && cmake --build . 2>&1", timeout=600)
    if exit_code != 0:
        print("  [ERROR] Build failed!")
        return False

    print("\n  Build succeeded!")
    return True


def install_udev(ssh, detect_dir):
    """Install camera udev rules."""
    rules = f"{detect_dir}/99-tank-cameras.rules"
    print("\n  Installing udev rules...")
    run_remote(ssh,
        f"sudo cp {rules} /etc/udev/rules.d/ && sudo udevadm control --reload-rules 2>&1",
        stream=False)
    print("  Udev rules installed (update busnum/devpath for your USB topology)")


def make_executable(ssh, detect_dir):
    """Make shell scripts executable."""
    run_remote(ssh, f"chmod +x {detect_dir}/start_tank.sh {detect_dir}/slam_start.sh 2>/dev/null", stream=False)


def copy_from_existing(ssh, sftp, home_dir, source_ip, source_user, source_pass):
    """Copy engine, calibration, and ReID model from an existing Jetson."""
    print("\n" + "=" * 60)
    print(f"  COPYING FILES FROM EXISTING JETSON ({source_ip})")
    print("=" * 60)

    files_to_copy = [
        (f"/home/{source_user}/YOLOv8-TensorRT/yolov8n.engine",
         f"{home_dir}/{YOLO_REL}/yolov8n.engine",
         "YOLOv8 TensorRT engine"),
        (f"/home/{source_user}/stereo_calib/stereo_params_cuda.xml",
         f"{home_dir}/stereo_calib/stereo_params_cuda.xml",
         "Stereo calibration"),
        (f"/home/{source_user}/YOLOv8-TensorRT/BoTSORT-cpp/assets/mobilenetv2_x1_4_msmt17.onnx",
         f"{home_dir}/{YOLO_REL}/BoTSORT-cpp/assets/mobilenetv2_x1_4_msmt17.onnx",
         "BoTSORT ReID model"),
    ]

    # Use scp from new Jetson to old Jetson
    for src, dst, desc in files_to_copy:
        print(f"\n  Copying {desc}...")
        exit_code = run_remote(ssh,
            f'sshpass -p "{source_pass}" scp -o StrictHostKeyChecking=no '
            f'{source_user}@{source_ip}:{src} {dst} 2>&1',
            timeout=120, stream=False)
        if exit_code != 0:
            print(f"  WARNING: Failed to copy {desc} — you may need to copy manually")
        else:
            print(f"  OK: {desc}")


def main():
    parser = argparse.ArgumentParser(
        description="Tank C2 — Deploy to fresh or existing Jetson",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Full fresh setup (first time — installs everything, 3-4 hours):
  python deploy_fresh.py --setup --host 192.168.1.200 --user james --password mypass

  # Full setup but OpenCV already installed:
  python deploy_fresh.py --setup --skip-opencv --host 192.168.1.200 --user james --password mypass

  # Code deploy + build only (deps already installed):
  python deploy_fresh.py --host 192.168.1.200 --user james --password mypass

  # Deploy and also copy engine/calibration from existing Jetson:
  python deploy_fresh.py --host 192.168.1.200 --user james --password mypass \\
    --copy-from 192.168.1.167 --copy-user james --copy-pass '!123Wo55a'
        """)

    parser.add_argument("--host", required=True, help="Jetson IP address")
    parser.add_argument("--user", required=True, help="Jetson SSH username")
    parser.add_argument("--password", required=True, help="Jetson SSH password")
    parser.add_argument("--port", type=int, default=22, help="SSH port (default: 22)")

    parser.add_argument("--setup", action="store_true",
                        help="Run full setup (install all deps, build OpenCV, ROS2, etc)")
    parser.add_argument("--skip-opencv", action="store_true",
                        help="Skip OpenCV build during setup (if already installed)")
    parser.add_argument("--skip-ros2", action="store_true",
                        help="Skip ROS2 install during setup (if already installed)")

    parser.add_argument("--copy-from", metavar="IP",
                        help="Copy engine/calibration from existing Jetson at this IP")
    parser.add_argument("--copy-user", default="james",
                        help="SSH user for existing Jetson (default: james)")
    parser.add_argument("--copy-pass", default="",
                        help="SSH password for existing Jetson")

    parser.add_argument("--no-build", action="store_true",
                        help="Upload files but don't build")

    args = parser.parse_args()

    ssh, sftp = connect(args.host, args.user, args.password, args.port)
    home_dir = f"/home/{args.user}"
    detect_dir = f"{home_dir}/{DETECT_REL}"

    try:
        # ── Step 1: Full setup (optional) ──────────────────────────────────
        if args.setup:
            # Need detect dir to exist before uploading source
            ensure_remote_dir(sftp, f"{detect_dir}/include")
            ensure_remote_dir(sftp, f"{detect_dir}/build")

            if not run_setup(ssh, sftp, home_dir, detect_dir,
                           skip_opencv=args.skip_opencv, skip_ros2=args.skip_ros2):
                print("\nSetup failed. Fix errors and re-run.")
                return 1

        # ── Step 2: Upload all source files ────────────────────────────────
        print("\n" + "=" * 60)
        print("  UPLOADING SOURCE FILES")
        print("=" * 60)
        ensure_remote_dir(sftp, f"{detect_dir}/include")
        ensure_remote_dir(sftp, f"{detect_dir}/build")
        uploaded = upload_files(sftp, home_dir, detect_dir)
        if uploaded == 0:
            print("No files uploaded!")
            return 1

        # Make scripts executable
        make_executable(ssh, detect_dir)

        # ── Step 3: Install udev rules ─────────────────────────────────────
        install_udev(ssh, detect_dir)

        # ── Step 4: Copy from existing Jetson (optional) ───────────────────
        if args.copy_from:
            # Install sshpass if needed
            run_remote(ssh, "sudo apt install -y sshpass 2>&1", stream=False)
            copy_from_existing(ssh, sftp, home_dir,
                             args.copy_from, args.copy_user, args.copy_pass)

        # ── Step 5: Build ──────────────────────────────────────────────────
        if not args.no_build:
            if not build_app(ssh, detect_dir):
                print("\nBuild failed. Check errors above.")
                return 1

        # ── Done ───────────────────────────────────────────────────────────
        print("\n" + "=" * 60)
        print("  DEPLOYMENT COMPLETE")
        print("=" * 60)
        print(f"""
  Jetson: {args.user}@{args.host}
  Binary: {detect_dir}/build/yolov8
  Config: {detect_dir}/config.json

  To start:
    ssh {args.user}@{args.host}
    bash {detect_dir}/start_tank.sh --no-ekf

  Things to verify:
    - Stereo calibration: ~/stereo_calib/stereo_params_cuda.xml
    - YOLOv8 engine:      ~/{YOLO_REL}/yolov8n.engine
    - ReID model:         ~/{YOLO_REL}/BoTSORT-cpp/assets/mobilenetv2_x1_4_msmt17.onnx
    - Camera udev rules:  Update busnum/devpath in 99-tank-cameras.rules
    - Config:             Edit {detect_dir}/config.json for new IPs if needed
""")
        return 0

    finally:
        sftp.close()
        ssh.close()


if __name__ == "__main__":
    sys.exit(main())
