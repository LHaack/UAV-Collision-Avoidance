#!/usr/bin/env python3
import subprocess
import sys
import time


PX4_PATH = "/home/arthurgroll/Documents/estudos/IC/bolsa-ia-drones/PX4-Autopilot-ColAvoid"
WORLD_NAME = "experimento_6"


def main(args=None):
    instance = int(sys.argv[1])
    spawn_position = sys.argv[2] if len(sys.argv) > 2 else "0.0,0.0"

    if instance == 0:
        subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", 
                        "MicroXRCEAgent udp4 -p 8888" + "; exec bash"])
    else:
        cd_command = f"cd {PX4_PATH} && "
        log_dir = "/home/arthurgroll/Documents/estudos/IC/bolsa-ia-drones/GazeboDRL/logs"
        log_file = f"{log_dir}/px4_{instance}_sitl.log"
        simulation_command = f"PX4_SYS_AUTOSTART=4013 PX4_GZ_WORLD={WORLD_NAME} PX4_GZ_MODEL_POSE={spawn_position} PX4_SIM_MODEL=gz_x500_lidar_3d ./build/px4_sitl_default/bin/px4 -i {instance} | tee -a {log_file}"

        time.sleep(5*instance)
        subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", 
                        cd_command + simulation_command + "; exec bash"])

if __name__ == '__main__':
    main()