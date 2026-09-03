#!/usr/bin/env python3
"""
Força todos os enemies a voar na mesma altitude Z do drone principal no Gazebo.
O principal voa naturalmente. Os enemies são teleportados para o Z do principal.
"""
import subprocess
import threading
import time
import re
import signal
import sys
import json
import os

WORLD = "experimento_6"
REFERENCE = "x500_lidar_3d_1"

script_dir = os.path.dirname(os.path.abspath(__file__))
enemies_json = os.path.join(script_dir, "collision_test_enemies.json")
with open(enemies_json) as f:
    num_enemies = len(json.load(f))

ENEMIES = [f"x500_lidar_3d_{3 + i}" for i in range(num_enemies)]
ALL_MODELS = [REFERENCE] + ENEMIES

poses = {}
lock = threading.Lock()
running = True

def signal_handler(sig, frame):
    global running
    running = False
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

def pose_listener():
    proc = subprocess.Popen(
        ["gz", "topic", "-e", "-t", f"/world/{WORLD}/dynamic_pose/info"],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True
    )
    current_name = None
    current_pos = {}

    for line in proc.stdout:
        if not running:
            break
        line = line.strip()

        if line.startswith("name:"):
            match = re.search(r'name:\s*"(.+)"', line)
            if match:
                current_name = match.group(1)
                current_pos = {}

        if current_name and current_name in ALL_MODELS:
            for axis in ["x:", "y:", "z:"]:
                if line.startswith(axis):
                    try:
                        val = float(line.split(":")[1].strip())
                        current_pos[axis[0]] = val
                    except:
                        pass

            if len(current_pos) == 3:
                with lock:
                    poses[current_name] = dict(current_pos)
                current_name = None
                current_pos = {}

def teleport_loop(model):
    while running:
        with lock:
            ref = poses.get(REFERENCE)
            enemy = poses.get(model)

        if ref and enemy and ref['z'] > 1.0:
            if abs(enemy['z'] - ref['z']) > 0.02:
                try:
                    subprocess.run(
                        ["gz", "service", "-s", f"/world/{WORLD}/set_pose",
                         "--reqtype", "gz.msgs.Pose",
                         "--reptype", "gz.msgs.Boolean",
                         "--timeout", "50",
                         "--req",
                         f'name: "{model}", position: {{x: {enemy["x"]}, y: {enemy["y"]}, z: {ref["z"]}}}'],
                        capture_output=True, timeout=1
                    )
                except:
                    pass

        time.sleep(0.03)

print(f"Altitude sync: {ENEMIES} -> follows {REFERENCE}")

t = threading.Thread(target=pose_listener, daemon=True)
t.start()

print("Aguardando modelos...")
while running:
    with lock:
        found = [m for m in ALL_MODELS if m in poses]
    if len(found) == len(ALL_MODELS):
        break
    time.sleep(1)
    print(f"  {len(found)}/{len(ALL_MODELS)} modelos encontrados")

print(f"Todos os modelos OK. Sincronizando {len(ENEMIES)} enemies com {REFERENCE}...")

threads = []
for enemy in ENEMIES:
    th = threading.Thread(target=teleport_loop, args=(enemy,), daemon=True)
    th.start()
    threads.append(th)

while running:
    with lock:
        ref = poses.get(REFERENCE)
    if ref:
        zs = []
        with lock:
            for m in ALL_MODELS:
                if m in poses:
                    zs.append(f"{m}: {poses[m]['z']:.2f}")
        print(f"  {' | '.join(zs)}")
    time.sleep(5)
