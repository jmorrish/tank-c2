#!/usr/bin/env python3
"""Deploy Phase 4 (Nav2 path planning) files to Jetson and build."""
import paramiko
import os

JETSON_IP   = "192.168.1.167"
JETSON_USER = "james"
JETSON_PASS = "!123Wo55a"
REMOTE_DIR  = "/home/james/YOLOv8-TensorRT/csrc/jetson/detect"
LOCAL_DIR   = r"c:\python\tank-c2\jetson"

# Files to deploy — ALL phases (1-4)
FILES = [
    # Phase 1: Generic sensor slots
    "sensor_store.cpp",
    "include/sensor_store.h",
    "include/helpers.h",
    # Phase 2: Behavior architecture
    "behavior_coordinator.cpp",
    "follow_behavior.cpp",
    "include/behavior.h",
    "include/behavior_coordinator.h",
    "include/follow_behavior.h",
    "include/control_mode.h",
    # Phase 3: MissionBehavior
    "mission_behavior.cpp",
    "include/mission_behavior.h",
    # Phase 4: Nav2 path planning
    "slam_link.cpp",
    "comms.cpp",
    "mission.cpp",
    "main.cpp",
    "nav2_planner.cpp",
    "CMakeLists.txt",
    "include/comms.h",
    "include/mission.h",
    "include/slam_link.h",
    "include/path_planner.h",
    "include/nav2_planner.h",
    "include/obstacle_utils.h",
    # Config/script files
    "nav2_params.yaml",
    "ekf_params.yaml",
    "nav2_planner_launch.py",
    "start_tank.sh",
    # Python
    "slam_bridge.py",
]

def main():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    print(f"Connecting to {JETSON_IP}...")
    ssh.connect(JETSON_IP, username=JETSON_USER, password=JETSON_PASS)
    sftp = ssh.open_sftp()

    # Ensure include dir exists
    try:
        sftp.stat(f"{REMOTE_DIR}/include")
    except FileNotFoundError:
        sftp.mkdir(f"{REMOTE_DIR}/include")

    print(f"Deploying {len(FILES)} files...")
    for f in FILES:
        local  = os.path.join(LOCAL_DIR, f.replace("/", os.sep))
        remote = f"{REMOTE_DIR}/{f}"
        if not os.path.exists(local):
            print(f"  SKIP (not found): {f}")
            continue
        sftp.put(local, remote)
        print(f"  OK: {f}")

    # Make start_tank.sh executable
    ssh.exec_command(f"chmod +x {REMOTE_DIR}/start_tank.sh")
    print("\nMade start_tank.sh executable")

    # Build
    print("\nBuilding...")
    build_dir = f"{REMOTE_DIR}/build"
    stdin, stdout, stderr = ssh.exec_command(
        f"cd {build_dir} && cmake .. 2>&1 && cmake --build . 2>&1",
        timeout=300
    )
    output = stdout.read().decode()
    errors = stderr.read().decode()
    exit_code = stdout.channel.recv_exit_status()

    print(output[-3000:] if len(output) > 3000 else output)
    if errors:
        print("STDERR:", errors[-2000:] if len(errors) > 2000 else errors)
    print(f"\nBuild exit code: {exit_code}")

    sftp.close()
    ssh.close()

if __name__ == "__main__":
    main()
