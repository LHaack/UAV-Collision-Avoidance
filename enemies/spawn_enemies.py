#!/usr/bin/env python3
import asyncio
import json
import math
import os
import subprocess
import sys
import time
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityNedYaw, PositionNedYaw)

PX4_PATH = "../../PX4-Autopilot-ColAvoid"
WORLD_NAME = "experimento_6"
MODEL_NAME = "gz_omnicopter"
TARGET_HEIGHT = -4.0
SPEED_TAKEOFF = 2.0
SPEED_CRUISE = 1.0

class SimpleBarrier:
    def __init__(self, number):
        self.number = number
        self.counter = 0
        self.event = asyncio.Event()

    async def wait(self):
        self.counter += 1
        if self.counter >= self.number:
            self.event.set()
        await self.event.wait()

async def run_drone(instance_id, spawn_pose, target_pose, barrier):
    sys_addr = f"udpin://0.0.0.0:{14540 + instance_id}"
    print(f"[Enemey {instance_id}] Connecting to {sys_addr}...")
    
    drone = System(port=50050 + instance_id)
    await drone.connect(system_address=sys_addr)

    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"[Enemy {instance_id}] Connected!")
            break

    print(f"[Enemy {instance_id}] Waiting for Global Position Estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print(f"[Enemy {instance_id}] Global Position OK!")
            break

    print(f"[Enemy {instance_id}] Setting up safety parameters...")
    try:
        await drone.param.set_param_int("COM_RCL_EXCEPT", 4)
        await drone.param.set_param_int("NAV_DLL_ACT", 0)
        await drone.param.set_param_int("NAV_RCL_ACT", 0)
        await drone.param.set_param_int("COM_ARM_MAG_ANG", -1)
        await drone.param.set_param_int("COM_ARM_WO_GPS", 1)
        await drone.param.set_param_int("EKF2_MAG_TYPE", 0)
        await drone.param.set_param_int("EKF2_GPS_CHECK", 0)
        await drone.param.set_param_float("COM_OF_LOSS_T", 5.0)
        print(f"[Enemy {instance_id}] Safety parameters set.")
    except Exception as e:
        print(f"[Enemy {instance_id}] Error setting params: {e}")

    print(f"[Enemy {instance_id}] Arming...")
    for attempt in range(5):
        try:
            await drone.action.arm()
            print(f"[Enemy {instance_id}] Armed!")
            break
        except Exception as e:
            print(f"[Enemy {instance_id}] Arming failed (attempt {attempt+1}): {e}")
            await asyncio.sleep(2)
    else:
        print(f"[Enemy {instance_id}] Failed to arm after 5 attempts. Exiting control.")
        return
    
    GLOBAL_HOME_LAT = 47.397971057728974
    GLOBAL_HOME_LON = 8.546163739800146
    GLOBAL_HOME_ALT = 488.049
    print(f"[Enemy {instance_id}] Setting global home ({GLOBAL_HOME_LAT}, {GLOBAL_HOME_LON}, {GLOBAL_HOME_ALT}m AMSL)...")
    try:
        await drone.shell.send(f"commander set_home {GLOBAL_HOME_LAT} {GLOBAL_HOME_LON} {GLOBAL_HOME_ALT}")
        print(f"[Enemy {instance_id}] Global home set.")
    except Exception as e:
        print(f"[Enemy {instance_id}] Failed to set home via shell: {e}")

    print(f"[Enemy {instance_id}] Starting Offboard...")
    initial_setpoint = VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
    await drone.offboard.set_velocity_ned(initial_setpoint)

    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"[Enemy {instance_id}] Offboard failed: {error}")
        return

    print(f"[Enemy {instance_id}] Taking off to {-TARGET_HEIGHT}m...")
    async for position in drone.telemetry.position_velocity_ned():
        current_alt = -position.position.down_m
        print(f"[Enemy {instance_id}] Current Alt: {current_alt:.2f}")

        if current_alt >= -TARGET_HEIGHT:
             print(f"[Enemy {instance_id}] Altitude reached: {current_alt:.2f}m!")
             break

        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -SPEED_TAKEOFF, 0.0))

    target_n, target_e, _ = target_pose

    # Guardar posição inicial para oscilar
    async for pos in drone.telemetry.position_velocity_ned():
        start_n = pos.position.north_m
        start_e = pos.position.east_m
        break

    waypoints = [(target_n, target_e), (start_n, start_e)]
    wp_idx = 0

    print(f"[Enemy {instance_id}] Oscillating between ({start_n:.1f},{start_e:.1f}) and ({target_n:.1f},{target_e:.1f})")

    error_z_integral = 0.0

    async for position_vel in drone.telemetry.position_velocity_ned():
        current_n = position_vel.position.north_m
        current_e = position_vel.position.east_m
        current_d = position_vel.position.down_m

        dest_n, dest_e = waypoints[wp_idx]
        delta_n = dest_n - current_n
        delta_e = dest_e - current_e
        dist = math.sqrt(delta_n**2 + delta_e**2)

        if dist < 1.0:
            wp_idx = (wp_idx + 1) % len(waypoints)
            dest_n, dest_e = waypoints[wp_idx]
            delta_n = dest_n - current_n
            delta_e = dest_e - current_e
            dist = math.sqrt(delta_n**2 + delta_e**2)
            print(f"[Enemy {instance_id}] Switching to waypoint {wp_idx}: ({dest_n:.1f},{dest_e:.1f})")

        if dist > 0.1:
            norm_n = delta_n / dist
            norm_e = delta_e / dist
            vel_n = norm_n * SPEED_CRUISE
            vel_e = norm_e * SPEED_CRUISE
        else:
            vel_n = 0.0
            vel_e = 0.0

        error_z = TARGET_HEIGHT - current_d
        error_z_integral += error_z * 0.05
        error_z_integral = max(-1.0, min(1.0, error_z_integral))
        vel_d = error_z * 3.0 + error_z_integral * 2.0

        await drone.offboard.set_velocity_ned(VelocityNedYaw(vel_n, vel_e, vel_d, 0.0))

def spawn_px4(instance_id, x, y, z):
    print(f"Spawning Instance {instance_id} at {x},{y},{z}...")
    pose_str = f"{x},{y},{z}"

    script_dir = os.path.dirname(os.path.abspath(__file__))
    px4_dir = os.path.abspath(os.path.join(script_dir, PX4_PATH))
    
    if not os.path.exists(px4_dir):
        print(f"ERROR: PX4 directory not found at {px4_dir}")
        return

    log_file = f"/home/arthurgroll/Documents/estudos/IC/bolsa-ia-drones/GazeboDRL/logs/px4_{instance_id}_enemy.log"
    bin_path = os.path.join(px4_dir, "build", "px4_sitl_default", "bin", "px4")

    env = os.environ.copy()
    env["PX4_SYS_AUTOSTART"] = "8011"
    env["PX4_GZ_WORLD"] = WORLD_NAME
    env["PX4_GZ_MODEL_POSE"] = pose_str
    env["PX4_SIM_MODEL"] = MODEL_NAME

    print(f"  PX4 binary: {bin_path}")
    print(f"  Working dir: {px4_dir}/build/px4_sitl_default")
    print(f"  Log: {log_file}")

    try:
        with open(log_file, "w") as lf:
            subprocess.Popen(
                [bin_path, "-i", str(instance_id)],
                cwd=os.path.join(px4_dir, "build", "px4_sitl_default"),
                env=env,
                stdout=lf,
                stderr=lf
            )
    except Exception as e:
        print(f"  ERROR spawning PX4: {e}")

    time.sleep(10)

async def main():
    no_spawn = "--no-spawn" in sys.argv
    filtered_args = [a for a in sys.argv[1:] if not a.startswith("--")]
    settings_path = filtered_args[0] if filtered_args else os.path.join(os.path.dirname(__file__), "settings.json")
    with open(settings_path, "r") as f:
        enemies = json.load(f)

    tasks = []

    start_instance = 3
    for a in sys.argv[1:]:
        if a.startswith("--start-instance="):
            start_instance = int(a.split("=")[1])
    barrier = SimpleBarrier(len(enemies))

    for i, enemy in enumerate(enemies):
        instance_id = start_instance + i
        spawn_pose = enemy["spawn_pose"]
        target_pose = enemy["target_pose"]

        if not no_spawn:
            spawn_px4(instance_id, spawn_pose[0], spawn_pose[1], spawn_pose[2])
        tasks.append(asyncio.create_task(run_drone(instance_id, spawn_pose, target_pose, barrier)))

    await asyncio.gather(*tasks)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Stopping enemies...")
