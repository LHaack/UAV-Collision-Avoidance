
from geometry_msgs.msg import Twist, Vector3
import math
import numpy as np
from px4_msgs.msg import ObstacleDistance, OffboardControlMode, TrajectorySetpoint, VehicleAttitude, VehicleCommand, VehicleOdometry, VehicleStatus, VehicleLandDetected
from sensor_msgs.msg import PointCloud2
import struct
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Bool
import sys


DISTANCE_COLLISION = 50
DISTANCE_TOOCLOSE = 100
DISTANCE_SAFE = 150
MAX_SPEED_LIMIT = 500

DISTANCE_VARSPACE = DISTANCE_SAFE - DISTANCE_TOOCLOSE
SPEED_LIMIT_MULTIPLIER = 100.0

HALF_PI = 1.570796325

DANGER_ZONE_RADIUS = 2

MIN_SPEED = 0.0
MAX_SPEED = 1.0
MAX_YAW_SPEED = 1.5

MISSION_TOLERANCE = 0.5
ANGLE_TOLERANCE = 5

TARGET_Z = 5



class Tee(object):
    def __init__(self, *files):
        self.files = files
    def write(self, obj):
        for f in self.files:
            f.write(obj)
            f.flush()
    def flush(self):
        for f in self.files:
            f.flush()

class OffboardControl(Node):

    def __init__(self, namespace, uav_number, mission_mode, mission_steps, spawn_configuration, lidar_topic=None):
        super().__init__('minimal_publisher')
        
        import os
        log_dir = "/home/arthurgroll/Documents/estudos/IC/bolsa-ia-drones/GazeboDRL/logs"
        os.makedirs(log_dir, exist_ok=True)
        log_file_path = os.path.join(log_dir, f"{namespace}.log")
        self.log_file = open(log_file_path, "a")
        self.get_logger().info(f"Logging to {log_file_path}")

        self.mission_target_z = 4.0
        self.altitude_error_integral = 0.0

        # --- Toggle: nosso VFH+ 3D (default) vs VFH+ vanilla de livro (baseline) ---
        # Ligado pela flag --px4-standard do run_collision_test.sh (exporta USE_VFH=1).
        # Quando ligado, o desvio usa avoid_obstacles_vfh (VFH+ de livro) em vez do
        # nosso avoid_obstacles_3d. Sem a flag (default), nada muda: roda o nosso.
        self.use_vfh = (os.environ.get('USE_VFH', '0') == '1')
        self.get_logger().info(
            f"Collision avoidance: {'VFH+ vanilla (baseline, --px4-standard)' if self.use_vfh else 'custom (VFH+ 3D)'}")

        self.namespace = namespace
        self.uav_id = int(namespace.split('_')[-1]) + 1
        self.instance_id = self.uav_id - 1
        self.uav_number = uav_number

        self.mission_mode = mission_mode
        self.mission_steps = mission_steps
        self.log_debug(f"INIT: Drone {self.instance_id} Mission Steps: {self.mission_steps}")
        self.mission_index = 0
        self.next_step = ""
        self.finish_mission_position = {'N': 0.0, 'E': 0.0, 'D': 0.0}

        if isinstance(spawn_configuration, str) and spawn_configuration.endswith('.json'):
            import json
            try:
                with open(spawn_configuration, 'r') as f:
                    config = json.load(f)
                    uav_idx = int(namespace.split('_')[-1]) - 1 # Use 0-based index for JSON
                    if 0 <= uav_idx < len(config):
                        json_mission = config[uav_idx].get('mission', [])
                        if json_mission:
                            self.mission_steps = json_mission
                            self.get_logger().info(f"Loaded {len(self.mission_steps)} steps from JSON")
            except Exception as e:
                self.get_logger().error(f"Failed to load mission from JSON: {e}")
        self.last_position = {
            'N': 0.0,
            'E': 0.0,
            'D': -self.mission_target_z,
            'yaw': 0.0 
        }

        self.spawn_configuration = spawn_configuration
        self.lidar_topic_arg = lidar_topic
        self.spawn_position = get_spawn_position(self.spawn_configuration, int(self.namespace[-1]) - 1, self.uav_number)

        self.arrived = True
        self.landed = False
        self.landing_mode = False
        self.collision_detected = False
        self.lidar_cb_count = 0

        self.cmdloop_control = 0
        
        self.debug_log_path = "/home/arthurgroll/Documents/estudos/IC/bolsa-ia-drones/GazeboDRL/logs/custom_debug.log"
        with open(self.debug_log_path, "a") as f:
             f.write(f"\n\n--- NEW SESSION: {namespace} ---\n")
        
        self.init_node_logic()
    
    def log_debug(self, msg):
        try:
            with open(self.debug_log_path, "a") as f:
                timestamp = Clock().now().nanoseconds / 1e9
                f.write(f"[{timestamp:.3f}] {msg}\n")
        except Exception as e:
            print(f"LOG ERROR: {e}")
            
        print(msg)

    def init_node_logic(self):
        pass

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self.log_debug("INIT: QoS Profile Created (CP1)")
        
        self.obstacle_distance = ObstacleDistance()
        self.obstacle_distance.increment = 5.0
        self.obstacle_distance.min_distance = 10
        self.obstacle_distance.max_distance = 2000
        self.obstacle_distance.angle_offset = 0.0
        self.obstacle_distance.distances = [65535] * 72

        self.avoidance_decision_memory = None
        self.lidar_points_3d = None
        self.dodge_state = "CLEAR"
        self.dodge_side_h = None
        self.dodge_side_v = None
        self._dodge_perp = None
        self._dodge_gd = None
        self._stuck_counter = 0
        self._last_pos_n = 0.0
        self._last_pos_e = 0.0
        self._bug_side = None   # TangentBug: lado comprometido ('L'/'R') enquanto contorna
        self._leave_counter = 0  # ciclos consecutivos com alvo livre (debounce p/ largar a parede)
        self._prev_min_obs_dist = None  # distancia ao obstaculo mais proximo no ciclo anterior
        self._closing_ema = 0.0  # taxa de aproximacao suavizada (m/s) p/ detectar obstaculo movel
        self._moving_count = 0   # frames consecutivos com obstaculo movel se aproximando
        self.last_avoidance_time = 0.0

        self.status_subscription = self.create_subscription(
            VehicleStatus,
            f'/{self.namespace}/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile
        )

        self.offboard_velocity_subscription = self.create_subscription(
            Twist,
            f'/{self.namespace}/offboard_velocity_cmd',
            self.offboard_velocity_callback,
            qos_profile
        )

        self.attitude_subscription = self.create_subscription(
            VehicleAttitude,
            f'/{self.namespace}/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile
        )
        self.log_debug("INIT: Basic Subs Created (CP2)")

        self.uav_positions = {}
        self.position_subscriptions = []

        for i in range(1, self.uav_number+1):
            uav_namespace = f'px4_{i}'
            topic_name = f'/{uav_namespace}/fmu/out/vehicle_odometry'

            self.uav_positions[uav_namespace] = None

            subscription = self.create_subscription(
                VehicleOdometry,
                topic_name,
                lambda msg, ns=uav_namespace: self.odometry_callback(msg, ns),
                qos_profile
            )

            self.position_subscriptions.append(subscription)

        target_lidar_topic = self.lidar_topic_arg if self.lidar_topic_arg else f'/x500_lidar_3d_{self.instance_id}/lidar_3d/points'
        
        self.lidar_subscription = self.create_subscription(
            PointCloud2,
            target_lidar_topic,
            self.lidar_3d_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info(f"Subscribed to {target_lidar_topic} (PointCloud2)")
        self.log_debug("INIT: Lidar Sub Created (CP3)")

        self.my_bool_subscription = self.create_subscription(
            Bool,
            f'/{self.namespace}/arm_message',
            self.arm_message_callback,
            qos_profile
        )

        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, f'/{self.namespace}/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_velocity = self.create_publisher(Twist, f'/{self.namespace}/fmu/in/setpoint_velocity/cmd_vel_unstamped', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, f'/{self.namespace}/fmu/in/trajectory_setpoint', qos_profile)
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, f'/{self.namespace}/fmu/in/vehicle_command', 10)
        self.arm_publisher = self.create_publisher(Bool, f'/{self.namespace}/arm_message', qos_profile)

        arm_timer_period = .1
        self.arm_timer_ = self.create_timer(arm_timer_period, self.arm_timer_callback)
        self.log_debug("INIT: Arm Timer Created (CP4)")

        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED
        self.velocity = Vector3()
        self.yaw = 0.0
        self.true_yaw = 0.0
        self.offboard_mode = False
        self.flight_check = False
        self.my_control = 0
        self.arm_message = False
        self.failsafe = False

        self.states = {
            "IDLE": self.state_init,
            "ARMING": self.state_arming,
            "TAKEOFF": self.state_takeoff,
            "LOITER": self.state_loiter,
            "OFFBOARD": self.state_offboard,
            "FINISHED": self.state_finished
        }

        self.current_state = "IDLE"
        self.last_state = self.current_state

        self.log_debug("INIT: Initialization Complete.")

    def state_finished(self):
        pass

    def arm_timer_callback(self):
        if self.my_control % 20 == 0:
            self.log_debug(f"ARM_TIMER: State={self.current_state} FlightCheck={self.flight_check}")

        match self.current_state:
            case "IDLE":
                if(self.flight_check and self.arm_message == True):
                    self.current_state = "ARMING"
                    self.arm_message = False
                    self.get_logger().info(f"Arming")
                elif self.my_control % 20 == 0:
                    self.get_logger().info(f"IDLE: FlightCheck={self.flight_check}, ArmMsg={self.arm_message}")

            case "ARMING":
                if(not(self.flight_check)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Arming, Flight Check Failed")
                elif(self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.my_control > 10):
                    self.current_state = "OFFBOARD"
                    self.get_logger().info(f"Arming, Switching to Offboard (Direct Takeoff)")
                    self.set_global_home()
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
                    self.offboard_mode = True

                self.arm()

            case "FINISHED":
                pass

            case "OFFBOARD":
                if(not(self.flight_check) or self.arm_state == VehicleStatus.ARMING_STATE_DISARMED or self.failsafe == True):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Offboard, Flight Check Failed")

                self.state_offboard()

        if (self.last_state != self.current_state):
            self.last_state = self.current_state
            self.get_logger().info(self.current_state)

        self.my_control += 1


    def state_init(self):
        self.my_control = 0


    def state_arming(self):
        self.my_control = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")


    def state_takeoff(self):
        self.my_control = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7=5.0)
        self.get_logger().info("Takeoff command send")


    def state_loiter(self):
        self.my_control = 0
        self.get_logger().info("Loiter Status")


    def state_offboard(self):
        self.my_control = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        self.offboard_mode = True

    def arm_message_callback(self, msg):
        if msg.data:
            self.arm_message = True

    def set_global_home(self):
        GLOBAL_HOME_LAT = 47.397971057728974
        GLOBAL_HOME_LON = 8.546163739800146
        GLOBAL_HOME_ALT = 488.049
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_HOME
        msg.param1 = 0.0
        msg.param5 = GLOBAL_HOME_LAT
        msg.param6 = GLOBAL_HOME_LON
        msg.param7 = GLOBAL_HOME_ALT
        msg.target_system = self.uav_id
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.publisher_vehicle_command.publish(msg)
        self.get_logger().info(f"Home set to global: {GLOBAL_HOME_LAT}, {GLOBAL_HOME_LON}, {GLOBAL_HOME_ALT}m AMSL")

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")

    def take_off(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7=5.0)
        self.get_logger().info("Takeoff command send")

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7
        msg.command = command
        msg.target_system = self.uav_id
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.publisher_vehicle_command.publish(msg)

    def vehicle_status_callback(self, msg):
        if (msg.nav_state != self.nav_state):
            self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")

        if (msg.arming_state != self.arm_state):
            self.get_logger().info(f"ARM STATUS: {msg.arming_state}")

        if (msg.failsafe != self.failsafe):
            self.get_logger().info(f"FAILSAFE: {msg.failsafe}")

        if (msg.pre_flight_checks_pass != self.flight_check):
            self.get_logger().info(f"Flight check: {msg.pre_flight_checks_pass}")

        if self.nav_state != msg.nav_state:
            self.get_logger().info(f"NAV_STATUS Changed: {self.nav_state} -> {msg.nav_state}")
            self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flight_check = msg.pre_flight_checks_pass

    def offboard_velocity_callback(self, msg):
        self.velocity.x = -msg.linear.y
        self.velocity.y = msg.linear.x
        self.velocity.z = -msg.linear.z
        self.yaw = msg.angular.z

    def attitude_callback(self, msg):
        orientation_q = msg.q
        self.attitude_q = orientation_q
        self.true_yaw = np.arctan2(2.0*(orientation_q[0]*orientation_q[3] + orientation_q[1]*orientation_q[2]),
                                  1.0 - 2.0*(orientation_q[2]*orientation_q[2] + orientation_q[3]*orientation_q[3]))
        
    def odometry_callback(self, msg, namespace):
        spawn = get_spawn_position(self.spawn_configuration, int(namespace[-1]) - 1, self.uav_number)
        self.uav_positions[namespace] = (msg.position[0]+spawn[0], msg.position[1]+spawn[1], msg.position[2])
        
        if namespace == self.namespace:
            self.odometry = msg
            if self.cmdloop_control % 50 == 0:
                self.log_debug(f"ODOM: Received OWN. Pos=[{msg.position[0]:.2f}, {msg.position[1]:.2f}, {msg.position[2]:.2f}]")
        else:
            pass

    def obstacle_distance_callback(self, msg):
        self.obstacle_distance = msg

    def lidar_3d_callback(self, msg):
        self.lidar_cb_count += 1
        if self.lidar_cb_count % 50 == 0:
            self.log_debug(f"LIDAR-ALIVE [Drone {self.instance_id}]: Msg Recv! Size={len(msg.data)}")
            
        # Parse PointCloud2 (simplified)ata from the 3D Lidar sensor.
        # Extracts X,Y,Z points and populates ObstacleDistance (72 sectors).
        # Filters ground points based on Z-height relative to sensor.
        # (malformed docstring removed)
        if not hasattr(self, 'odometry') or self.odometry is None:
            if self.cmdloop_control % 100 == 0:
                self.get_logger().warn("Lidar3D: Waiting for Odometry...")
            return
            
        # ObstacleDistance configuration
        increment_deg = 5.0  # 5 degrees per sector
        num_sectors = 72     # 360 / 5 = 72 sectors
        # Initialize with max distance (Numpy Array for Speed)
        distances = np.full(num_sectors, 65535, dtype=np.uint16)
        
        # Get point cloud attributes
        width = msg.width
        height = msg.height
        point_step = msg.point_step
        data = msg.data
        is_bigendian = msg.is_bigendian
        
        if self.cmdloop_control % 50 == 0:
            self.log_debug(f"Lidar3D [Drone {self.instance_id}]: Received cloud {width}x{height} point_step={point_step}")

        # --- OPTIMIZED NUMPY PARSING ---
        import ctypes
        
        # 1. Convert byte data to numpy array of float32
        # Note: This assumes x,y,z are float32 and little endian (standard in ROS 2 on x86)
        # If big endian, we might need byteswap.
        
        # We need to handle striding (point_step)
        # Structure view:
        # We form a structured array or just a view with stride.
        
        raw_data = np.frombuffer(data, dtype=np.uint8)
        
        # Validate size
        expected_bytes = width * height * point_step
        if len(raw_data) != expected_bytes:
             self.log_debug(f"Lidar3D: Error size mismatch. Exp {expected_bytes}, Got {len(raw_data)}")
             return

        # --- Extract Offsets from Fields ---
        x_off, y_off, z_off = 0, 4, 8 # Defaults
        try:
            for field in msg.fields:
                if field.name == 'x': x_off = field.offset
                elif field.name == 'y': y_off = field.offset
                elif field.name == 'z': z_off = field.offset
        except Exception:
            pass # Use defaults

        try:
            endian = '>' if is_bigendian else '<'
            dt_spec = {
                'names': ['x', 'y', 'z'],
                'formats': [f'{endian}f4', f'{endian}f4', f'{endian}f4'],
                'offsets': [x_off, y_off, z_off],
                'itemsize': point_step
            }
            point_dtype = np.dtype(dt_spec)
            cloud_arr = np.frombuffer(data, dtype=point_dtype)

            x_arr = cloud_arr['x']
            y_arr = cloud_arr['y']
            z_arr = cloud_arr['z']
            
            valid_mask = np.isfinite(x_arr) & np.isfinite(y_arr) & np.isfinite(z_arr)
            valid_mask &= (z_arr >= -3.0) & (z_arr <= 3.0)

            x_valid = x_arr[valid_mask]
            y_valid = y_arr[valid_mask]
            z_valid = z_arr[valid_mask]

            dist_3d = np.sqrt(x_valid**2 + y_valid**2 + z_valid**2)

            dist_center_sq = (x_valid + 0.12)**2 + y_valid**2
            body_mask = dist_center_sq >= (0.50**2)
            range_mask = (dist_3d >= 0.1) & (dist_3d <= 7.0)  # 7m: TangentBug ve longe p/ escolher o lado
            final_mask = body_mask & range_mask

            x_final = x_valid[final_mask]
            y_final = y_valid[final_mask]
            z_final = z_valid[final_mask]

            self.lidar_points_3d = np.column_stack((x_final, y_final, z_final)) if len(x_final) > 0 else None

            z_near = np.abs(z_final) < 0.5
            x_2d = x_final[z_near]
            y_2d = y_final[z_near]
            range_2d = np.sqrt(x_2d**2 + y_2d**2)
            range_mask_2d = (range_2d >= 0.1) & (range_2d <= 1.5)
            x_2d = x_2d[range_mask_2d]
            y_2d = y_2d[range_mask_2d]
            range_2d = range_2d[range_mask_2d]

            if len(x_2d) > 0:
                angles = np.arctan2(y_2d, x_2d)
                angles_deg = np.degrees(angles)
                angles_deg[angles_deg < 0] += 360.0

                sector_indices = (angles_deg / increment_deg).astype(int)
                np.clip(sector_indices, 0, num_sectors - 1, out=sector_indices)

                range_cm = (range_2d * 100).astype(int)
                np.clip(range_cm, 0, 65535, out=range_cm)

                np.minimum.at(distances, sector_indices, range_cm)
                        
        except Exception as e:
            self.log_debug(f"Lidar3D Numpy Error: {e}")
            return
            
        self.obstacle_distance.timestamp = int(Clock().now().nanoseconds / 1000)
        self.obstacle_distance.increment = increment_deg
        self.obstacle_distance.distances = distances.tolist()

        min_dist = np.min(distances)
        if min_dist < 65535 and (self.cmdloop_control % 50 == 0):
             min_idx = np.argmin(distances)
             min_angle = min_idx * increment_deg
             rad = math.radians(min_angle)
             d_m = float(min_dist) / 100.0
             obs_x = d_m * math.cos(rad)
             obs_y = d_m * math.sin(rad)
             
             min_x = np.min(x_final) if len(x_final) > 0 else 0.0
             max_x = np.max(x_final) if len(x_final) > 0 else 0.0
             min_y = np.min(y_final) if len(y_final) > 0 else 0.0
             max_y = np.max(y_final) if len(y_final) > 0 else 0.0
             
             self.log_debug(f"LIDAR_DEBUG [Drone {self.instance_id}]: Min={min_dist}cm @ {min_angle:.1f}deg (X={obs_x:.2f}, Y={obs_y:.2f}) | Bounds X[{min_x:.2f}, {max_x:.2f}] Y[{min_y:.2f}, {max_y:.2f}]")
             

             
        if not hasattr(self, 'lidar_log_count'): self.lidar_log_count = 0
        self.lidar_log_count += 1
        
        if self.lidar_log_count % 10 == 0: # Log every ~10 frames (approx 1 sec at 10Hz)
            sector_dump = []

            for i, d in enumerate(distances):
                 if d < 65535: # Only valid sectors
                     angle = i * increment_deg
                     sector_dump.append(f"[{angle:.0f}deg: {d}cm]")
            
            if sector_dump:
                self.log_debug(f"SECTOR MAP [Drone {self.instance_id}]: " + " | ".join(sector_dump))
            else:
                self.log_debug(f"SECTOR MAP [Drone {self.instance_id}]: CLEAR (No obstacles < 1.5m)")




        if self.cmdloop_control % 50 == 0:
             valid_indices = np.where(distances < 65535)[0]
             if len(valid_indices) > 0:
                 idx = valid_indices[0]
                 val = distances[idx]
                 self.log_debug(f"SAMPLEPT [Drone {self.instance_id}]: Sector {idx} has {val}cm")

    def cmdloop_callback(self):
        if self.cmdloop_control % 50 == 0:
             self.log_debug(f"CMDLOOP: Entered. Control={self.cmdloop_control} State={self.current_state} MissionMode={self.mission_mode}")

        if self.current_state == "IDLE" and self.mission_mode:
            arm_message = Bool()
            arm_message.data = True
            self.arm_publisher.publish(arm_message)

        if self.cmdloop_control % 50 == 0:
            pos_str = "N/A"
            dist_str = "N/A"
            if hasattr(self, 'odometry') and self.odometry is not None:
                curr_N = self.odometry.position[0]
                curr_E = self.odometry.position[1]
                curr_D = self.odometry.position[2]
                pos_str = f"[{curr_N:.2f}, {curr_E:.2f}, {curr_D:.2f}]"
                
                if self.finish_mission_position['N'] != 0.0 or self.finish_mission_position['E'] != 0.0:
                     tgt_N = self.finish_mission_position['N']
                     tgt_E = self.finish_mission_position['E']
                     dist = math.sqrt((tgt_N - curr_N)**2 + (tgt_E - curr_E)**2)
                     dist_str = f"{dist:.2f}m"
            
            tgt_str = f"[{self.finish_mission_position['N']:.2f}, {self.finish_mission_position['E']:.2f}]"
            
            nav_state_val = self.nav_state if hasattr(self, 'nav_state') else "UNK"
            self.log_debug(f"MONITOR: State={self.current_state} NavState={nav_state_val} Offboard={self.offboard_mode} Pos={pos_str} Tgt={tgt_str} Dist={dist_str}")
            if self.cmdloop_control % 20 == 0:
                self.get_logger().info("[CRITICAL] CODE VALIDATION: If you see this, the file IS updated.")
        if self.offboard_mode:
            self.cmdloop_control += 1

            if self.mission_mode and hasattr(self, 'finish_mission_position'):
                dist_to_final = math.sqrt(
                    (self.odometry.position[0] - self.finish_mission_position['N'])**2 + 
                    (self.odometry.position[1] - self.finish_mission_position['E'])**2
                )
                
                if dist_to_final < 1.0:
                    if self.cmdloop_control % 50 == 0:
                        self.log_debug(
                            f"🎯 CLOSE TO TARGET: dist={dist_to_final:.2f}m, "
                            f"state={self.current_state}, "
                            f"mission_idx={self.mission_index}, "
                            f"landing_mode={self.landing_mode}"
                        )
                
                drone_altitude = abs(self.odometry.position[2])
                if (self.current_state == "OFFBOARD" and
                    drone_altitude > 2.0 and
                    dist_to_final < 0.5 and
                    not self.landing_mode):
                    
                    self.log_debug(f"🚨 EMERGENCY LANDING TRIGGER: Dist={dist_to_final:.2f}m, Alt={drone_altitude:.2f}m - FORCING LAND!")
                    self.landing_mode = True
                
                if self.landing_mode:
                    if self.cmdloop_control % 10 == 0:
                        self.get_logger().info(f"🛬 LANDING MODE ACTIVE: Sending LAND command...")
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                    return

            # publish offboard control modes
            # Sempre VELOCIDADE (tanto custom quanto baseline). O CP do PX4, quando
            # plugado no offboard via patch de firmware, atua sobre o velocity setpoint.
            offboard_message = OffboardControlMode()
            offboard_message.timestamp = int(Clock().now().nanoseconds / 1000)
            offboard_message.position = False
            offboard_message.velocity = True
            offboard_message.acceleration = False
            self.publisher_offboard_mode.publish(offboard_message)

            if self.cmdloop_control % 100 == 0:
                has_finish = hasattr(self, 'finish_mission_position')
                finish_val = self.finish_mission_position if has_finish else "N/A"
                self.log_debug(
                    f"🔍 DIAGNOSTIC: mission_mode={self.mission_mode}, "
                    f"has_finish_pos={has_finish}, "
                    f"finish_pos={finish_val}"
                )

            if self.mission_mode and self.mission_index < len(self.mission_steps):
                step = self.mission_steps[self.mission_index]
                if step.startswith("go"):
                    coords = get_coordinates(step)
                    self.finish_mission_position['N'] = coords[0]
                    self.finish_mission_position['E'] = coords[1]
                    self.finish_mission_position['D'] = coords[2] if len(coords) > 2 else -4.0


            # 1. Velocidade ideal: linha reta ao destino (NED world frame)
            goal_vel_n, goal_vel_e, dist_to_target = self.compute_ideal_velocity_ned()

            # 2. Collision avoidance — baseline (VFH+ vanilla) vs custom (nosso VFH+ 3D).
            #    use_vfh (flag --px4-standard): roda o VFH+ de livro como baseline.
            #    senao: roda o nosso avoid_obstacles_3d, inalterado.
            avoidance_z = 0.0
            try:
                if not self.landing_mode:
                    if self.use_vfh:
                        goal_vel_n, goal_vel_e, avoidance_z = self.avoid_obstacles_vfh(goal_vel_n, goal_vel_e)
                    else:
                        goal_vel_n, goal_vel_e, avoidance_z = self.avoid_obstacles_3d(goal_vel_n, goal_vel_e)
            except Exception as e:
                print(f"ERROR in avoidance: {e}")
                import traceback
                traceback.print_exc()

            # 3. Altitude: PI controller + avoidance vertical
            error_z = -self.mission_target_z - self.odometry.position[2]
            self.altitude_error_integral += error_z * 0.02
            self.altitude_error_integral = max(-1.0, min(1.0, self.altitude_error_integral))

            if not hasattr(self, '_climbing'):
                self._climbing = False

            if avoidance_z < -0.1:
                self._climbing = True

            if self._climbing:
                vel_z = -MAX_SPEED
                self.altitude_error_integral = 0.0
                if avoidance_z >= 0 and dist_to_target > 0.5:
                    self._climbing = False
                    new_alt = -self.odometry.position[2]
                    self.mission_target_z = max(new_alt, self.mission_target_z)
                    vel_z = 0.0
            else:
                vel_z = error_z * 3.0 + self.altitude_error_integral * 2.0 + avoidance_z
                # Só pode descer se perto do target (< 3m)
                if vel_z > 0 and dist_to_target > 3.0:
                    vel_z = 0.0

            # Clamp
            speed = math.sqrt(goal_vel_n**2 + goal_vel_e**2)
            if speed > MAX_SPEED:
                goal_vel_n = (goal_vel_n / speed) * MAX_SPEED
                goal_vel_e = (goal_vel_e / speed) * MAX_SPEED

            if self.cmdloop_control % 20 == 0:
                print(f"VEL_PUB: [{goal_vel_n:.2f}, {goal_vel_e:.2f}, {vel_z:.2f}] dist={dist_to_target:.1f}")

            # 4. Publicar setpoint
            # Setpoint de VELOCIDADE nos dois modos.
            #  - custom:   goal_vel_n/e já passaram pelo avoid_obstacles_3d (desvio nosso).
            #  - baseline: goal_vel_n/e é linha reta ao destino (sem desvio nosso); quem
            #              corta/desvia é o CP do PX4 (quando plugado no offboard via patch).
            trajectory_message = TrajectorySetpoint()
            trajectory_message.timestamp = int(Clock().now().nanoseconds / 1000)
            trajectory_message.velocity[0] = goal_vel_n
            trajectory_message.velocity[1] = goal_vel_e
            trajectory_message.velocity[2] = vel_z
            trajectory_message.position[0] = float('nan')
            trajectory_message.position[1] = float('nan')
            trajectory_message.position[2] = float('nan')
            trajectory_message.acceleration[0] = float('nan')
            trajectory_message.acceleration[1] = float('nan')
            trajectory_message.acceleration[2] = float('nan')

            trajectory_message.yaw = float('nan')
            trajectory_message.yawspeed = 0.0

            self.publisher_trajectory.publish(trajectory_message)


    def go_to_destiny(self, step):
        coordinates = get_coordinates(step)

        # Update final target for homing
        self.finish_mission_position['N'] = coordinates[0]
        self.finish_mission_position['E'] = coordinates[1]
        self.finish_mission_position['D'] = coordinates[2]

        if self.cmdloop_control % 100 == 0:
            print(f"GO_TO: Target=({coordinates[0]}, {coordinates[1]})")

        error_N = coordinates[0] - self.odometry.position[0]
        error_E = coordinates[1] - self.odometry.position[1]
        error_D = coordinates[2] - self.odometry.position[2]
        
        if self.cmdloop_control % 100 == 0:
             dist = math.sqrt(error_N**2 + error_E**2)
             self.get_logger().info(f"NAV: Tgt[{coordinates[0]:.1f}, {coordinates[1]:.1f}] Curr[{self.odometry.position[0]:.1f}, {self.odometry.position[1]:.1f}] Dist: {dist:.1f}m")

        self.velocity.x = 0.0
        self.velocity.y = 0.0
        self.velocity.z = 0.0
        
        arrived_at_waypoint = False
        
        magnitude = math.sqrt(error_N**2 + error_E**2)

        if self.cmdloop_control % 50 == 0:
            self.get_logger().info(f"ARRIVAL_CHECK: error_N={abs(error_N):.2f}, error_E={abs(error_E):.2f}, TOLERANCE={MISSION_TOLERANCE}, magnitude={magnitude:.2f}")

        if abs(error_N) < MISSION_TOLERANCE and abs(error_E) < MISSION_TOLERANCE:
            arrived_at_waypoint = True
            self.get_logger().info(f"✓ Arrived at waypoint! error_N={abs(error_N):.2f}, error_E={abs(error_E):.2f}")
        else:
            
            max_speed_internal = MAX_SPEED * 100.0

            if magnitude < 0.05:
                desired_speed = 0.0
            elif magnitude < 3.0:
                normalized_dist = magnitude / 3.0
                speed_factor = pow(normalized_dist, 0.6)
                desired_speed = max_speed_internal * speed_factor

                if desired_speed < 20.0 and magnitude > 0.15:
                    desired_speed = 20.0
            else:
                desired_speed = max_speed_internal

            if desired_speed > max_speed_internal:
                desired_speed = max_speed_internal

            if magnitude > 0.001:
                error_N = (error_N / magnitude) * desired_speed
                error_E = (error_E / magnitude) * desired_speed
            else:
                error_N = 0.0
                error_E = 0.0

            cos_yaw = np.cos(self.true_yaw)
            sin_yaw = np.sin(self.true_yaw)

            velocity_body_x = error_N * cos_yaw + error_E * sin_yaw
            velocity_body_y = -error_N * sin_yaw + error_E * cos_yaw

            self.velocity.x = velocity_body_x * 0.01
            self.velocity.y = velocity_body_y * 0.01

        
        if magnitude < 0.5:
             if len(coordinates) > 3:
                 desired_yaw = coordinates[3]
             else:
                 desired_yaw = self.true_yaw
        else:
             desired_yaw = math.atan2(error_E, error_N)
        current_yaw = self.true_yaw
        
        while desired_yaw > math.pi: desired_yaw -= 2*math.pi
        while desired_yaw < -math.pi: desired_yaw += 2*math.pi
        
        yaw_error = desired_yaw - current_yaw
        while yaw_error > math.pi: yaw_error -= 2*math.pi
        while yaw_error < -math.pi: yaw_error += 2*math.pi
        
        self.yaw = yaw_error * 1.5

        is_turning = False
        if abs(yaw_error) > 0.5:
            self.velocity.x = 0.0
            self.velocity.y = 0.0
            is_turning = True
            
        if self.cmdloop_control % 20 == 0:
            self.get_logger().info(f"NAV: YawErr={math.degrees(yaw_error):.1f} deg | Hdg={math.degrees(current_yaw):.1f} | Tgt={math.degrees(desired_yaw):.1f} | Turning={is_turning}")

        if self.yaw > MAX_YAW_SPEED: self.yaw = MAX_YAW_SPEED
        if self.yaw < -MAX_YAW_SPEED: self.yaw = -MAX_YAW_SPEED

        return arrived_at_waypoint

    def turn_to_destiny(self, next_step):
        angle = int(next_step.split(":")[1])
        return True
                

    def compute_ideal_velocity_ned(self):
        """
        Calcula a velocidade ideal (linha reta ao destino) no world frame NED.
        Retorna (vel_n, vel_e, speed) ou (0,0,0) se não há destino.
        """
        if not hasattr(self, 'finish_mission_position'):
            return 0.0, 0.0, 0.0

        curr_n = self.odometry.position[0]
        curr_e = self.odometry.position[1]
        tgt_n = self.finish_mission_position['N']
        tgt_e = self.finish_mission_position['E']

        delta_n = tgt_n - curr_n
        delta_e = tgt_e - curr_e
        dist = math.sqrt(delta_n**2 + delta_e**2)

        if dist < MISSION_TOLERANCE:
            return 0.0, 0.0, dist

        if dist < 3.0:
            speed = MAX_SPEED * pow(dist / 3.0, 0.6)
            speed = max(speed, 0.2)
        else:
            speed = MAX_SPEED

        vel_n = (delta_n / dist) * speed
        vel_e = (delta_e / dist) * speed

        return vel_n, vel_e, dist

    def avoid_obstacles_vfh(self, goal_vel_n, goal_vel_e):
        """
        VFH+ VANILLA (de livro) — baseline de comparacao (flag --px4-standard).

        Implementacao fiel ao VFH+ classico (Ulrich & Borenstein, 1998), adaptado
        para um drone holonomico (sem masking cinematico). DE PROPOSITO nao tem as
        melhorias do nosso avoid_obstacles_3d (sem wall-sliding, sem repulsao de
        emergencia, sem 3D): serve para medir o que essas melhorias agregam.

        Passos:
          1. Histograma polar primario: cada ponto do LiDAR vota com magnitude
             m = a - b*d (mais perto = maior), espalhada por +-gamma setores
             (alargamento pelo raio do drone -> trata o drone como ponto).
          2. Histograma binario com histerese (tau_low/tau_high).
          3. Selecao de candidatos a partir das aberturas (vales) livres:
             aberturas largas geram candidatos perto das bordas e na direcao do
             alvo; estreitas geram o candidato central.
          4. Custo g(c) = mu1*d(c,alvo) + mu2*d(c,heading) + mu3*d(c,c_anterior);
             escolhe o candidato de menor custo.
          5. Velocidade na direcao escolhida, reduzida perto de obstaculo.

        Retorna (vel_n, vel_e, 0.0). So atua XY (Z fica no controle de altitude).
        """
        # --- Parametros VFH+ ---
        N = 72                      # setores de 5 graus
        SECTOR = 2.0 * math.pi / N
        DETECT_DIST = 5.0           # alcance considerado (m)
        ROBOT_RADIUS = 0.6          # raio do drone + margem de seguranca (m)
        A_CONST = DETECT_DIST       # m = A - B*d  (>=0); A garante m=0 em d=DETECT
        B_CONST = 1.0
        TAU_HIGH = 3.0              # limiar superior (setor passa a bloqueado)
        TAU_LOW = 1.5               # limiar inferior (setor volta a livre) - histerese
        S_MAX = 16                  # largura (setores) acima da qual a abertura e "larga"
        MU1, MU2, MU3 = 5.0, 2.0, 2.0   # pesos: alvo, heading atual, direcao anterior
        SAFE_DIST = 2.5             # abaixo disso reduz velocidade

        if self.lidar_points_3d is None or len(self.lidar_points_3d) == 0:
            return goal_vel_n, goal_vel_e, 0.0

        pts = self.lidar_points_3d
        xb, yb, zb = pts[:, 0], pts[:, 1], pts[:, 2]
        d2d = np.sqrt(xb**2 + yb**2)
        valid = (d2d >= 0.3) & (d2d <= DETECT_DIST) & (np.abs(zb) < 2.0)
        if not np.any(valid):
            return goal_vel_n, goal_vel_e, 0.0

        xv, yv, dv = xb[valid], yb[valid], d2d[valid]

        # body -> NED (mundo), igual ao avoid_obstacles_3d
        cos_y, sin_y = math.cos(self.true_yaw), math.sin(self.true_yaw)
        on = xv * cos_y - yv * sin_y
        oe = xv * sin_y + yv * cos_y
        beta = np.arctan2(oe, on)               # angulo do obstaculo no mundo (NED: N=0, E=+90)

        # direcao do alvo (mundo NED)
        curr_n, curr_e = self.odometry.position[0], self.odometry.position[1]
        gn = self.finish_mission_position['N'] - curr_n
        ge = self.finish_mission_position['E'] - curr_e
        goal_dist = math.hypot(gn, ge)
        if goal_dist < 0.1:
            return goal_vel_n, goal_vel_e, 0.0
        goal_angle = math.atan2(ge, gn)

        # --- 1. Histograma primario (com alargamento por gamma) ---
        # Vetorizado: para cada offset de setor [-max_sp, +max_sp], soma a magnitude
        # nos pontos cujo alargamento (spread) alcanca aquele offset. Evita loop por
        # ponto (rapido a 50 Hz mesmo com milhares de pontos).
        hist = np.zeros(N)
        mags = np.maximum(A_CONST - B_CONST * dv, 0.0)              # magnitude por ponto
        gamma = np.arcsin(np.minimum(ROBOT_RADIUS / dv, 1.0))      # meio-alargamento (rad)
        centers = ((beta % (2 * math.pi)) / SECTOR).astype(int) % N
        spreads = np.ceil(gamma / SECTOR).astype(int)
        max_sp = int(spreads.max()) if spreads.size else 0
        for off in range(-max_sp, max_sp + 1):
            sel = np.abs(off) <= spreads
            if np.any(sel):
                np.add.at(hist, (centers[sel] + off) % N, mags[sel])

        # --- 2. Histograma binario com histerese ---
        prev_bin = getattr(self, '_vfh_binary', np.zeros(N, dtype=bool))
        blocked = np.where(hist > TAU_HIGH, True, np.where(hist < TAU_LOW, False, prev_bin))
        self._vfh_binary = blocked

        free = ~blocked
        target_sector = int((goal_angle % (2 * math.pi)) / SECTOR) % N

        # Se nao ha nenhum setor livre -> para (VFH puro trava; e um ponto de comparacao)
        if not np.any(free):
            return 0.0, 0.0, 0.0
        # Se ha caminho livre direto ao alvo -> vai reto
        if free[target_sector]:
            chosen = target_sector
        else:
            # --- 3. Candidatos a partir das aberturas ---
            heading_sector = int((self.true_yaw % (2 * math.pi)) / SECTOR) % N
            prev_sector = getattr(self, '_vfh_prev_sector', heading_sector)
            candidates = []
            s = 0
            while s < N:
                if free[s]:
                    run = 0
                    while run < N and free[(s + run) % N]:
                        run += 1
                    kr = s                      # borda direita da abertura
                    kl = (s + run - 1) % N      # borda esquerda
                    if run > S_MAX:             # abertura larga: candidatos perto das bordas
                        candidates.append((kr + S_MAX // 2) % N)
                        candidates.append((kl - S_MAX // 2) % N)
                        if free[target_sector]:
                            candidates.append(target_sector)
                    else:                       # abertura estreita: candidato central
                        candidates.append((kr + run // 2) % N)
                    s += run
                else:
                    s += 1

            if not candidates:
                return 0.0, 0.0, 0.0

            def ang_diff(a, b):
                d = abs(a - b) % N
                return min(d, N - d)

            def cost(c):
                return (MU1 * ang_diff(c, target_sector)
                        + MU2 * ang_diff(c, heading_sector)
                        + MU3 * ang_diff(c, prev_sector))
            chosen = min(candidates, key=cost)

        self._vfh_prev_sector = chosen
        chosen_angle = chosen * SECTOR          # NED: 0=Norte, +90=Leste

        # --- 5. Velocidade: reduz perto de obstaculo na direcao escolhida ---
        cn, ce = math.cos(chosen_angle), math.sin(chosen_angle)
        proj = on * cn + oe * ce                # distancia projetada na direcao de voo
        ahead = proj[proj > 0]
        min_ahead = float(np.min(ahead)) if ahead.size > 0 else DETECT_DIST
        speed = MAX_SPEED * float(np.clip(min_ahead / SAFE_DIST, 0.2, 1.0))
        # desacelera ao chegar perto do alvo
        if goal_dist < 3.0:
            speed = min(speed, MAX_SPEED * max(goal_dist / 3.0, 0.15))

        return speed * cn, speed * ce, 0.0

    def _choose_bug_side(self, points, gd_n, gd_e, vision, goal_sector, N, sector_deg):
        """TangentBug: escolhe o lado (esquerda/direita do alvo) por onde o obstaculo
        TERMINA mais perto — ou seja, varre a partir da direcao do alvo para cada
        lado e acha a 1a 'saida' (vao livre real) dentro de 'vision' metros. O lado
        com a borda angularmente mais proxima tem mais espaco livre / menor desvio.
        Retorna 'L' ou 'R'."""
        # Ocupacao por setor ate 'vision' (binario: tem obstaculo dentro de vision?)
        occ = np.zeros(N, dtype=bool)
        if points is not None and len(points) > 0:
            x, y = points[:, 0], points[:, 1]
            cy, sy = math.cos(self.true_yaw), math.sin(self.true_yaw)
            on = x * cy - y * sy            # body -> NED
            oe = x * sy + y * cy
            d = np.sqrt(on**2 + oe**2)
            m = (d >= 0.3) & (d <= vision)
            if np.any(m):
                ang = np.degrees(np.arctan2(oe[m], on[m])) % 360.0
                idx = (ang / sector_deg).astype(int) % N
                occ[idx] = True
        GAP_MIN = 4   # vao precisa de >=4 setores livres (~20 graus) p/ valer como saida

        def edge_distance(direction):
            """Quantos setores ate o 1o vao livre (>=GAP_MIN) varrendo p/ 'direction'."""
            k = 1
            while k < N // 2:
                if not occ[(goal_sector + direction * k) % N]:
                    w = 0
                    while w < GAP_MIN and not occ[(goal_sector + direction * (k + w)) % N]:
                        w += 1
                    if w >= GAP_MIN:
                        return k
                k += 1
            return N  # nao achou saida desse lado dentro da visao

        right_e = edge_distance(+1)   # +k = sentido horario = direita do alvo
        left_e = edge_distance(-1)    # -k = esquerda do alvo
        return 'R' if right_e <= left_e else 'L'

    def _bug_follow_velocity(self, side, gd_n, gd_e, nearest_n, nearest_e, wall_dist, safe_dist):
        """TangentBug: segue a fronteira do obstaculo pelo lado COMPROMETIDO,
        mantendo um standoff. A tangente e escolhida pelo lado fixo (nao troca de
        lado a cada ciclo -> sem oscilar na borda). Apenas XY (nao sobe)."""
        wd = max(wall_dist, 0.01)
        nn = -nearest_n / wd            # normal apontando para LONGE do obstaculo
        ne = -nearest_e / wd
        t1n, t1e = -ne, nn              # as duas tangentes da parede
        t2n, t2e = ne, -nn
        if side == 'L':
            pn, pe = gd_e, -gd_n        # esquerda do alvo
        else:
            pn, pe = -gd_e, gd_n        # direita do alvo
        # tangente alinhada ao lado comprometido
        if t1n * pn + t1e * pe >= t2n * pn + t2e * pe:
            tn, te = t1n, t1e
        else:
            tn, te = t2n, t2e
        rep_w = max(0.0, min(1.0, (safe_dist - wd) / safe_dist))   # mais perto -> mais repulsao
        vn = tn * (1.0 - 0.5 * rep_w) + nn * (0.5 * rep_w)
        ve = te * (1.0 - 0.5 * rep_w) + ne * (0.5 * rep_w)
        mag = math.hypot(vn, ve)
        if mag > 1e-6:
            vn, ve = vn / mag, ve / mag
        return vn * MAX_SPEED, ve * MAX_SPEED

    # OLD functions removed for cleanliness
    # Available in git history

    def avoid_obstacles_3d(self, goal_vel_n, goal_vel_e):
        """
        VFH+ (Vector Field Histogram) Local Planner.

        1. Constrói histograma polar a partir dos pontos 3D do LiDAR
        2. Identifica vales (sequências de setores livres)
        3. Escolhe o vale cuja direção central está mais próxima do goal
        4. Calcula velocidade na direção do vale, com magnitude proporcional
           à distância do obstáculo mais próximo
        5. Repulsão de emergência quando obstáculo está a < EMERGENCY_DIST
        6. Wall-sliding: quando perto de parede, projeta velocidade do goal
           na tangente da parede para "escorregar" ao longo dela
        """
        # --- PARÂMETROS ---
        DETECT_DIST = 5.0       # alcance máximo do LiDAR considerado (m)
        BUG_VISION = 7.0        # visão p/ escolher o LADO no TangentBug (m) — vê longe
        EMERGENCY_DIST = 1.0    # distância de emergência — recuo forte (m)
        SAFE_DIST = 2.5         # distância segura — abaixo disso, reduz velocidade + desvia (m)
        N_SECTORS = 72          # setores de 5° cada
        SECTOR_DEG = 360.0 / N_SECTORS
        MIN_VALLEY_WIDTH = 5    # mínimo de setores livres para ser um vale válido (~25°)
        SPEED_NEAR_OBS = 0.6    # velocidade mínima quando perto de obstáculo (m/s)

        vel_z = 0.0  # default: sem ajuste vertical

        # --- VALIDAÇÃO ---
        if self.lidar_points_3d is None or len(self.lidar_points_3d) == 0:
            return goal_vel_n, goal_vel_e, 0.0

        points = self.lidar_points_3d
        x_b, y_b, z_b = points[:, 0], points[:, 1], points[:, 2]
        dist_2d = np.sqrt(x_b**2 + y_b**2)

        # Filtrar pontos válidos (range e altura relativa ao drone)
        valid = (dist_2d >= 0.3) & (dist_2d <= DETECT_DIST) & (np.abs(z_b) < 2.0)
        if not np.any(valid):
            return goal_vel_n, goal_vel_e, 0.0

        xv, yv = x_b[valid], y_b[valid]
        dv = dist_2d[valid]

        # --- BODY -> NED ---
        cos_yaw = np.cos(self.true_yaw)
        sin_yaw = np.sin(self.true_yaw)
        obs_n = xv * cos_yaw - yv * sin_yaw
        obs_e = xv * sin_yaw + yv * cos_yaw

        # --- GOAL RELATIVO ---
        curr_n = self.odometry.position[0]
        curr_e = self.odometry.position[1]
        tgt_n = self.finish_mission_position['N']
        tgt_e = self.finish_mission_position['E']
        goal_rel_n = tgt_n - curr_n
        goal_rel_e = tgt_e - curr_e
        goal_dist = math.sqrt(goal_rel_n**2 + goal_rel_e**2)

        if goal_dist < 0.1:
            return goal_vel_n, goal_vel_e, 0.0

        gd_n = goal_rel_n / goal_dist
        gd_e = goal_rel_e / goal_dist

        # --- Distância mínima global ---
        obs_dists_all = np.sqrt(obs_n**2 + obs_e**2)
        min_obs_dist = float(np.min(obs_dists_all))
        nearest_idx = int(np.argmin(obs_dists_all))
        nearest_n = float(obs_n[nearest_idx])
        nearest_e = float(obs_e[nearest_idx])

        # =============================================
        # OBSTÁCULO MÓVEL (inimigo) vindo na direção do drone -> SUBIR rápido
        # =============================================
        # Estima a taxa de aproximação do obstáculo mais próximo. Uma PAREDE estática
        # só "se aproxima" no máximo à velocidade do PRÓPRIO drone; se a aproximação
        # for MAIOR que isso, o obstáculo está se movendo em nossa direção (inimigo).
        # Nesse caso, subir é muito mais rápido que contornar lateralmente.
        MOVING_TRIGGER = 4.0     # distância p/ reagir a obstáculo móvel (m)
        MOVING_MARGIN = 0.8      # quanto a aproximação precisa exceder a vel. do drone (m/s)
        MOVING_FRAMES = 3        # frames consecutivos exigidos (~0.06s) p/ confirmar (anti-ruído)
        dt = 0.02                # loop a 50 Hz
        closing = 0.0
        if self._prev_min_obs_dist is not None:
            closing = (self._prev_min_obs_dist - min_obs_dist) / dt   # >0 = aproximando
            closing = max(-12.0, min(12.0, closing))                  # clamp anti-spike (troca de cluster)
        self._closing_ema = 0.6 * self._closing_ema + 0.4 * closing
        self._prev_min_obs_dist = min_obs_dist

        drone_speed = 0.0
        try:
            drone_speed = math.hypot(float(self.odometry.velocity[0]), float(self.odometry.velocity[1]))
        except Exception:
            drone_speed = 0.0

        # Obstáculo se aproximando MAIS RÁPIDO que a vel. do drone => não é parede.
        if min_obs_dist < MOVING_TRIGGER and self._closing_ema > drone_speed + MOVING_MARGIN:
            self._moving_count += 1
        else:
            self._moving_count = 0

        if self._moving_count >= MOVING_FRAMES:
            # Afasta-se levemente na horizontal e SOBE rápido (vel_z muito negativo
            # aciona o modo de subida no cmdloop_callback).
            rep_n = -nearest_n / max(min_obs_dist, 0.01)
            rep_e = -nearest_e / max(min_obs_dist, 0.01)
            rm = math.hypot(rep_n, rep_e)
            if rm > 1e-6:
                rep_n, rep_e = rep_n / rm, rep_e / rm
            if self.cmdloop_control % 25 == 0:
                self.log_debug(
                    f"MOVING-OBS [Drone {self.instance_id}]: closing={self._closing_ema:.1f}m/s "
                    f"drone_v={drone_speed:.1f} d={min_obs_dist:.2f}m -> SOBE"
                )
            return rep_n * MAX_SPEED * 0.4, rep_e * MAX_SPEED * 0.4, -MAX_SPEED

        # =============================================
        # EMERGÊNCIA: obstáculo muito perto (< 1.0m)
        # Repulsão direta + subir
        # =============================================
        if min_obs_dist < EMERGENCY_DIST:
            rep_strength = (EMERGENCY_DIST - min_obs_dist) / EMERGENCY_DIST
            rep_strength = min(rep_strength, 1.0)

            # Direção de repulsão (para longe do obstáculo mais próximo)
            rep_n = -nearest_n / max(min_obs_dist, 0.01)
            rep_e = -nearest_e / max(min_obs_dist, 0.01)
            rep_mag = math.sqrt(rep_n**2 + rep_e**2)
            if rep_mag > 0.01:
                rep_n /= rep_mag
                rep_e /= rep_mag

            # Perpendicular à repulsão (escorregar ao longo da parede). Se ja ha um
            # lado comprometido pelo TangentBug, escorrega POR ELE (mantem o contorno);
            # senao, a perpendicular mais alinhada ao goal.
            perp1_n, perp1_e = -rep_e, rep_n
            perp2_n, perp2_e = rep_e, -rep_n
            if self._bug_side == 'L':
                pref_n, pref_e = gd_e, -gd_n
            elif self._bug_side == 'R':
                pref_n, pref_e = -gd_e, gd_n
            else:
                pref_n, pref_e = gd_n, gd_e
            if perp1_n * pref_n + perp1_e * pref_e >= perp2_n * pref_n + perp2_e * pref_e:
                slide_n, slide_e = perp1_n, perp1_e
            else:
                slide_n, slide_e = perp2_n, perp2_e

            # Misturar: mais perto -> mais repulsão, mais longe -> mais sliding
            vel_n = rep_n * MAX_SPEED * rep_strength + slide_n * MAX_SPEED * (1.0 - rep_strength)
            vel_e = rep_e * MAX_SPEED * rep_strength + slide_e * MAX_SPEED * (1.0 - rep_strength)
            vel_z = 0.0  # NAO sobe: o desvio e lateral

            # Se a direção final também está bloqueada, recua direto (repulsão pura) —
            # ainda SEM subir.
            move_speed = math.sqrt(vel_n**2 + vel_e**2)
            if move_speed > 0.05:
                md_n = vel_n / move_speed
                md_e = vel_e / move_speed
                proj = obs_n * md_n + obs_e * md_e
                lat = np.abs(obs_n * (-md_e) + obs_e * md_n)
                in_path = (proj > 0) & (proj < 2.0) & (lat < 0.6) & (obs_dists_all < SAFE_DIST)
                if bool(np.any(in_path)):
                    vel_n = rep_n * MAX_SPEED
                    vel_e = rep_e * MAX_SPEED
                    vel_z = 0.0

            # Clamp
            speed = math.sqrt(vel_n**2 + vel_e**2)
            if speed > MAX_SPEED:
                vel_n = (vel_n / speed) * MAX_SPEED
                vel_e = (vel_e / speed) * MAX_SPEED

            if self.cmdloop_control % 50 == 0:
                self.log_debug(
                    f"VFH-EMERG [Drone {self.instance_id}]: min={min_obs_dist:.2f}m "
                    f"rep=({rep_n:.2f},{rep_e:.2f}) slide=({slide_n:.2f},{slide_e:.2f}) "
                    f"NED=({vel_n:.2f},{vel_e:.2f},{vel_z:.2f})"
                )

            return vel_n, vel_e, vel_z

        # =============================================
        # VFH+: Construir histograma polar (NED frame)
        # =============================================
        # Ângulos dos obstáculos em NED (0° = North, CW positivo)
        angles_ned = np.degrees(np.arctan2(obs_e, obs_n))  # atan2(east, north)
        angles_ned[angles_ned < 0] += 360.0

        # Histograma: distância mínima por setor
        sector_dist = np.full(N_SECTORS, DETECT_DIST + 1.0)
        sector_idx = (angles_ned / SECTOR_DEG).astype(int) % N_SECTORS
        for si, sd in zip(sector_idx, obs_dists_all):
            if sd < sector_dist[si]:
                sector_dist[si] = sd

        # Setores bloqueados: obstáculo a menos de SAFE_DIST
        blocked = sector_dist < SAFE_DIST

        # Ângulo do goal
        goal_angle = math.degrees(math.atan2(gd_e, gd_n))
        if goal_angle < 0:
            goal_angle += 360.0
        goal_sector = int(goal_angle / SECTOR_DEG) % N_SECTORS

        # =============================================
        # TANGENTBUG: comprometimento de lado FIRME + boundary following
        # =============================================
        # Dois cones em torno do alvo:
        #  - estreito (gatilho de desvio): caminho direto ao alvo bloqueado?
        #  - largo (criterio de saida): alvo livre num angulo amplo? (so larga a
        #    parede ai, e ainda com debounce, p/ NAO oscilar na borda)
        GOAL_CONE = 2       # ~+-10 graus
        WIDE_CONE = 7       # ~+-35 graus
        LEAVE_DEBOUNCE = 15  # ciclos (~0.3s) de alvo-livre-amplo antes de largar

        goal_blocked = any(blocked[(goal_sector + k) % N_SECTORS]
                           for k in range(-GOAL_CONE, GOAL_CONE + 1))
        goal_wide_clear = not any(blocked[(goal_sector + k) % N_SECTORS]
                                  for k in range(-WIDE_CONE, WIDE_CONE + 1))

        if self._bug_side is None:
            # --- NAO contornando ---
            if goal_blocked:
                # Comeca a contornar: escolhe o lado UMA vez (borda mais proxima,
                # visao 7m) e COMPROMETE. A partir daqui nao inverte mais.
                self._bug_side = self._choose_bug_side(
                    self.lidar_points_3d, gd_n, gd_e, BUG_VISION,
                    goal_sector, N_SECTORS, SECTOR_DEG)
                self._leave_counter = 0
                self.log_debug(f"TANGENTBUG [Drone {self.instance_id}]: COMECA contorno lado={self._bug_side}")
                vel_n, vel_e = self._bug_follow_velocity(
                    self._bug_side, gd_n, gd_e, nearest_n, nearest_e, min_obs_dist, SAFE_DIST)
            else:
                # Caminho livre: vai reto ao alvo.
                chosen_rad = math.radians(goal_angle)
                vel_n = math.cos(chosen_rad) * MAX_SPEED
                vel_e = math.sin(chosen_rad) * MAX_SPEED
        else:
            # --- COMPROMETIDO: segue SEMPRE o mesmo lado; so larga com debounce ---
            if goal_wide_clear:
                self._leave_counter += 1
            else:
                self._leave_counter = 0

            if self._leave_counter >= LEAVE_DEBOUNCE:
                # Contornou: alvo livre num cone amplo por tempo suficiente -> larga.
                self.log_debug(f"TANGENTBUG [Drone {self.instance_id}]: LARGA parede (alvo livre)")
                self._bug_side = None
                self._leave_counter = 0
                chosen_rad = math.radians(goal_angle)
                vel_n = math.cos(chosen_rad) * MAX_SPEED
                vel_e = math.sin(chosen_rad) * MAX_SPEED
            else:
                # Continua contornando pelo MESMO lado (sem inverter, sem subir).
                vel_n, vel_e = self._bug_follow_velocity(
                    self._bug_side, gd_n, gd_e, nearest_n, nearest_e, min_obs_dist, SAFE_DIST)
        # vel_z permanece 0.0 no contorno (lateral, nao sobe).

        # =============================================
        # MODULAÇÃO DE VELOCIDADE por proximidade
        # =============================================
        # Modula pela folga A FRENTE (na direcao do movimento), NAO pela parede
        # lateral. Assim, escorregando ao longo da parede (folga frontal grande) ele
        # mantem velocidade no desvio lateral; so freia se houver obstaculo de FRENTE.
        ms = math.hypot(vel_n, vel_e)
        if ms > 0.05:
            mdn, mde = vel_n / ms, vel_e / ms
            projm = obs_n * mdn + obs_e * mde
            latm = np.abs(obs_n * (-mde) + obs_e * mdn)
            fwd = projm[(projm > 0) & (latm < 1.0)]
            fwd_clear = float(np.min(fwd)) if fwd.size > 0 else DETECT_DIST
            if fwd_clear < SAFE_DIST:
                speed_factor = (fwd_clear - EMERGENCY_DIST) / (SAFE_DIST - EMERGENCY_DIST)
                speed_factor = max(SPEED_NEAR_OBS / MAX_SPEED, min(1.0, speed_factor))
                vel_n *= speed_factor
                vel_e *= speed_factor

        # =============================================
        # ANTI-STUCK: detecta se o drone está parado
        # =============================================
        moved = math.sqrt((curr_n - self._last_pos_n)**2 + (curr_e - self._last_pos_e)**2)
        if moved < 0.05:
            self._stuck_counter += 1
        else:
            self._stuck_counter = 0
            self._last_pos_n = curr_n
            self._last_pos_e = curr_e

        STUCK_THRESHOLD = 100  # ~2 segundos a 50Hz

        if self._stuck_counter > STUCK_THRESHOLD:
            # Preso: empurra LATERALMENTE (sem subir). Mantem o lado ja comprometido
            # pelo TangentBug; se nao houver, vai pro lado com menos obstaculos.
            if self._bug_side == 'L':
                vel_n, vel_e = gd_e * MAX_SPEED, -gd_n * MAX_SPEED
            elif self._bug_side == 'R':
                vel_n, vel_e = -gd_e * MAX_SPEED, gd_n * MAX_SPEED
            else:
                perp1_n, perp1_e = -gd_e, gd_n
                perp2_n, perp2_e = gd_e, -gd_n
                dot1 = obs_n * perp1_n + obs_e * perp1_e
                dot2 = obs_n * perp2_n + obs_e * perp2_e
                clear1 = float(np.min(obs_dists_all[dot1 > 0])) if np.any(dot1 > 0) else DETECT_DIST
                clear2 = float(np.min(obs_dists_all[dot2 > 0])) if np.any(dot2 > 0) else DETECT_DIST
                if clear1 >= clear2:
                    vel_n, vel_e = perp1_n * MAX_SPEED, perp1_e * MAX_SPEED
                else:
                    vel_n, vel_e = perp2_n * MAX_SPEED, perp2_e * MAX_SPEED
            # vel_z permanece 0: escape lateral, NAO sobe

        # =============================================
        # SEGURANÇA FINAL: nunca mover PARA DENTRO de um obstáculo próximo
        # =============================================
        # Remove apenas a COMPONENTE da velocidade que aponta para o obstáculo mais
        # próximo no caminho (a < SAFE_DIST, baixo offset lateral): o drone ESCORREGA
        # ao longo da parede em vez de clipar a lateral dela (caso classico: ao virar
        # pro alvo perto da parede grossa). É uma projeção geométrica determinística,
        # entao NAO reintroduz oscilacao.
        move_speed = math.sqrt(vel_n**2 + vel_e**2)
        if move_speed > 0.05:
            mdn, mde = vel_n / move_speed, vel_e / move_speed
            proj = obs_n * mdn + obs_e * mde             # quão à frente
            lat = np.abs(obs_n * (-mde) + obs_e * mdn)   # offset lateral
            ahead = (proj > 0) & (proj < SAFE_DIST) & (lat < 0.8)
            if np.any(ahead):
                di = np.where(ahead, obs_dists_all, 1e9)
                j = int(np.argmin(di))
                rn, re_ = float(obs_n[j]), float(obs_e[j])
                rd = math.hypot(rn, re_)
                if rd > 1e-6:
                    rn, re_ = rn / rd, re_ / rd
                    v_in = vel_n * rn + vel_e * re_       # componente indo PARA o obstáculo
                    if v_in > 0:
                        vel_n -= v_in * rn                # remove só a parte radial -> escorrega
                        vel_e -= v_in * re_

        # =============================================
        # CLAMP FINAL
        # =============================================
        speed = math.sqrt(vel_n**2 + vel_e**2)
        if speed > MAX_SPEED:
            vel_n = (vel_n / speed) * MAX_SPEED
            vel_e = (vel_e / speed) * MAX_SPEED

        # Não descer abaixo de 1.5m
        if -self.odometry.position[2] < 1.5 and vel_z > 0:
            vel_z = 0.0

        if self.cmdloop_control % 50 == 0:
            n_blocked = int(np.sum(blocked))
            self.log_debug(
                f"TANGENTBUG [Drone {self.instance_id}]: min={min_obs_dist:.2f}m "
                f"blocked={n_blocked}/{N_SECTORS} goal_ang={goal_angle:.0f} "
                f"side={self._bug_side} stuck={self._stuck_counter} "
                f"NED=({vel_n:.2f},{vel_e:.2f},{vel_z:.2f})"
            )

        return vel_n, vel_e, vel_z

    def avoid_obstacles_3d_OLD_TBUG(self, goal_vel_n, goal_vel_e):
        """OLD Tangent Bug.

        Estados:
        - MOTION_TO_GOAL: segue linha reta ao goal. Se obstáculo detectado na
          linha drone→goal, encontra os pontos de tangência (bordas do obstáculo)
          e escolhe o que reduz mais a distância ao goal. Transiciona para BOUNDARY_FOLLOW.

        - BOUNDARY_FOLLOW: contorna o obstáculo mantendo distância segura.
          A cada tick registra d_reach = min distância ao goal alcançável.
          Quando d_reach diminui (encontrou passagem), volta para MOTION_TO_GOAL.
        """
        DETECT_DIST = 5.0
        SAFE_DIST = 2.5
        N_RAYS = 72
        SECTOR_DEG = 360.0 / N_RAYS

        if self.lidar_points_3d is None or len(self.lidar_points_3d) == 0:
            self._tbug_state = "MOTION_TO_GOAL"
            return goal_vel_n, goal_vel_e, 0.0

        points = self.lidar_points_3d
        x_b, y_b, z_b = points[:, 0], points[:, 1], points[:, 2]
        dist_2d = np.sqrt(x_b**2 + y_b**2)

        valid = (dist_2d >= 0.3) & (dist_2d <= DETECT_DIST) & (np.abs(z_b) < 2.0)
        if not np.any(valid):
            self._tbug_state = "MOTION_TO_GOAL"
            return goal_vel_n, goal_vel_e, 0.0

        xv, yv = x_b[valid], y_b[valid]
        dv = dist_2d[valid]

        # Body → NED
        cos_yaw = np.cos(self.true_yaw)
        sin_yaw = np.sin(self.true_yaw)
        n_pts = xv * cos_yaw - yv * sin_yaw
        e_pts = xv * sin_yaw + yv * cos_yaw

        # Goal relativo
        curr_n = self.odometry.position[0]
        curr_e = self.odometry.position[1]
        tgt_n = self.finish_mission_position['N']
        tgt_e = self.finish_mission_position['E']
        goal_rel_n = tgt_n - curr_n
        goal_rel_e = tgt_e - curr_e
        dist_to_goal = math.sqrt(goal_rel_n**2 + goal_rel_e**2)

        if dist_to_goal < 0.1:
            return goal_vel_n, goal_vel_e, 0.0

        gd_n = goal_rel_n / dist_to_goal
        gd_e = goal_rel_e / dist_to_goal

        # --- Construir histograma polar (NED, centrado no drone) ---
        angles_ned = np.degrees(np.arctan2(e_pts, n_pts))
        angles_ned[angles_ned < 0] += 360

        sector_dist = np.full(N_RAYS, DETECT_DIST)
        sector_idx = (angles_ned / SECTOR_DEG).astype(int) % N_RAYS
        for si, sd in zip(sector_idx, dv):
            sector_dist[si] = min(sector_dist[si], sd)

        blocked = sector_dist < SAFE_DIST

        # Ângulo do goal no histograma
        goal_angle = math.degrees(math.atan2(gd_e, gd_n))
        if goal_angle < 0:
            goal_angle += 360
        goal_sector = int(goal_angle / SECTOR_DEG) % N_RAYS

        # Checar se linha ao goal está livre (cone de ±30° = ±6 setores)
        goal_line_blocked = False
        for i in range(-6, 7):
            s = (goal_sector + i) % N_RAYS
            if blocked[s]:
                goal_line_blocked = True
                break

        # Distância mínima a qualquer obstáculo
        nearest_idx = np.argmin(dv)
        min_dist_val = float(dv[nearest_idx])

        # Se qualquer obstáculo está muito perto, considerar bloqueado
        if min_dist_val < SAFE_DIST:
            goal_line_blocked = True

        # --- ESTADOS ---
        if not hasattr(self, '_tbug_state'):
            self._tbug_state = "MOTION_TO_GOAL"
            self._tbug_d_reach = dist_to_goal
            self._tbug_follow_dir = 1

        # Repulsão global
        if min_dist_val < SAFE_DIST:
            rep_n = -n_pts[nearest_idx] / min_dist_val
            rep_e = -e_pts[nearest_idx] / min_dist_val
            rep_weight = min((SAFE_DIST - min_dist_val) / SAFE_DIST * 2.0, 1.0)
        else:
            rep_n, rep_e, rep_weight = 0.0, 0.0, 0.0

        if self._tbug_state == "MOTION_TO_GOAL":
            if not goal_line_blocked:
                vel_n = gd_n * MAX_SPEED * (1.0 - rep_weight) + rep_n * MAX_SPEED * rep_weight
                vel_e = gd_e * MAX_SPEED * (1.0 - rep_weight) + rep_e * MAX_SPEED * rep_weight
            else:
                # Obstáculo na linha ao goal — encontrar tangentes
                # Varrer do goal_sector para a esquerda e direita para achar bordas
                left_tangent = goal_sector
                for i in range(1, N_RAYS):
                    s = (goal_sector + i) % N_RAYS
                    if not blocked[s]:
                        left_tangent = s
                        break

                right_tangent = goal_sector
                for i in range(1, N_RAYS):
                    s = (goal_sector - i) % N_RAYS
                    if not blocked[s]:
                        right_tangent = s
                        break

                # Calcular qual tangente leva mais perto do goal
                left_angle_rad = math.radians(left_tangent * SECTOR_DEG)
                right_angle_rad = math.radians(right_tangent * SECTOR_DEG)

                # Ponto projetado na direção da tangente a SAFE_DIST
                left_n = math.cos(left_angle_rad) * SAFE_DIST
                left_e = math.sin(left_angle_rad) * SAFE_DIST
                right_n = math.cos(right_angle_rad) * SAFE_DIST
                right_e = math.sin(right_angle_rad) * SAFE_DIST

                left_goal_dist = math.sqrt((tgt_n - (curr_n + left_n))**2 + (tgt_e - (curr_e + left_e))**2)
                right_goal_dist = math.sqrt((tgt_n - (curr_n + right_n))**2 + (tgt_e - (curr_e + right_e))**2)

                # Escolher tangente que reduz mais a distância
                if right_goal_dist <= left_goal_dist:
                    chosen_angle = right_angle_rad
                    self._tbug_follow_dir = -1  # anti-horário
                else:
                    chosen_angle = left_angle_rad
                    self._tbug_follow_dir = 1   # horário

                vel_n = math.cos(chosen_angle) * MAX_SPEED
                vel_e = math.sin(chosen_angle) * MAX_SPEED

                self._tbug_state = "BOUNDARY_FOLLOW"
                self._tbug_d_reach = dist_to_goal

        elif self._tbug_state == "BOUNDARY_FOLLOW":
            if not goal_line_blocked and min_dist_val > SAFE_DIST:
                self._tbug_state = "MOTION_TO_GOAL"
                vel_n = gd_n * MAX_SPEED
                vel_e = gd_e * MAX_SPEED
            else:
                # Encontrar setor livre adjacente à parede na direção de follow
                follow_sector = goal_sector
                for i in range(N_RAYS):
                    s = (goal_sector + self._tbug_follow_dir * i) % N_RAYS
                    if not blocked[s]:
                        prev = (s - self._tbug_follow_dir) % N_RAYS
                        if blocked[prev]:
                            follow_sector = s
                            break

                follow_angle = math.radians(follow_sector * SECTOR_DEG)
                follow_n = math.cos(follow_angle)
                follow_e = math.sin(follow_angle)

                # Repulsão: encontrar obstáculo mais próximo e afastar-se
                nearest_idx = np.argmin(dv)
                min_dist_val = dv[nearest_idx]
                repulse_n = -n_pts[nearest_idx]
                repulse_e = -e_pts[nearest_idx]
                rep_mag = math.sqrt(repulse_n**2 + repulse_e**2)
                if rep_mag > 0.001:
                    repulse_n /= rep_mag
                    repulse_e /= rep_mag

                # Misturar follow + repulsão baseado na proximidade
                if min_dist_val < SAFE_DIST:
                    rep_weight = (SAFE_DIST - min_dist_val) / SAFE_DIST
                    rep_weight = min(rep_weight * 2.0, 1.0)
                else:
                    rep_weight = 0.0

                vel_n = follow_n * MAX_SPEED * (1.0 - rep_weight) + repulse_n * MAX_SPEED * rep_weight
                vel_e = follow_e * MAX_SPEED * (1.0 - rep_weight) + repulse_e * MAX_SPEED * rep_weight

                if dist_to_goal < self._tbug_d_reach - 0.5:
                    self._tbug_d_reach = dist_to_goal

        else:
            vel_n = goal_vel_n
            vel_e = goal_vel_e

        vel_z = 0.0

        # Clamp
        speed = math.sqrt(vel_n**2 + vel_e**2)
        if speed > MAX_SPEED:
            vel_n = (vel_n / speed) * MAX_SPEED
            vel_e = (vel_e / speed) * MAX_SPEED

        if -self.odometry.position[2] < 1.5 and vel_z > 0:
            vel_z = 0.0

        if self.cmdloop_control % 50 == 0:
            min_dist = float(np.min(dv))
            n_blocked = int(np.sum(blocked))
            self.log_debug(
                f"TBUG [Drone {self.instance_id}]: state={self._tbug_state} "
                f"blocked={n_blocked}/{N_RAYS} min={min_dist:.2f}m "
                f"goal_blk={'Y' if goal_line_blocked else 'N'} d_tgt={dist_to_goal:.1f}m "
                f"NED=({vel_n:.2f},{vel_e:.2f},{vel_z:.2f})"
            )

        return vel_n, vel_e, vel_z

    def avoid_obstacles_3d_OLD_EGO(self, goal_vel_n, goal_vel_e):
        """OLD Ego-Planner.

        Baseado no EGO-Planner (Zhou et al., 2020):
        1. Gera trajetória B-spline do drone ao goal
        2. Verifica colisões com obstáculos do lidar
        3. Nos pontos de colisão, gera "force guides" que empurram a trajetória para longe
        4. Re-otimiza a trajetória com os force guides
        5. Segue o primeiro segmento

        Simplificado: usa trajetória poligonal em vez de B-spline.
        Force guides empurram waypoints para fora de obstáculos.
        """
        DETECT_DIST = 5.0
        N_WP = 8
        HORIZON = 4.0
        OPT_ITERS = 20
        SAFE_DIST = 1.5

        if self.lidar_points_3d is None or len(self.lidar_points_3d) == 0:
            return goal_vel_n, goal_vel_e, 0.0

        points = self.lidar_points_3d
        x_b, y_b, z_b = points[:, 0], points[:, 1], points[:, 2]
        dist_2d = np.sqrt(x_b**2 + y_b**2)

        valid = (dist_2d >= 0.3) & (dist_2d <= DETECT_DIST) & (np.abs(z_b) < 2.0)
        if not np.any(valid):
            return goal_vel_n, goal_vel_e, 0.0

        xv, yv = x_b[valid], y_b[valid]

        # Body → NED
        cos_yaw = np.cos(self.true_yaw)
        sin_yaw = np.sin(self.true_yaw)
        obs_n = xv * cos_yaw - yv * sin_yaw
        obs_e = xv * sin_yaw + yv * cos_yaw
        obs = np.column_stack((obs_n, obs_e))

        # Goal relativo ao drone em NED
        curr_n = self.odometry.position[0]
        curr_e = self.odometry.position[1]
        tgt_n = self.finish_mission_position['N']
        tgt_e = self.finish_mission_position['E']
        goal_rel_n = tgt_n - curr_n
        goal_rel_e = tgt_e - curr_e
        goal_dist = math.sqrt(goal_rel_n**2 + goal_rel_e**2)

        if goal_dist < 0.1:
            return goal_vel_n, goal_vel_e, 0.0

        gd_n = goal_rel_n / goal_dist
        gd_e = goal_rel_e / goal_dist
        traj_len = min(HORIZON, goal_dist)

        # Trajetória inicial: linha reta
        wp_n = np.linspace(0, gd_n * traj_len, N_WP)
        wp_e = np.linspace(0, gd_e * traj_len, N_WP)

        # --- EGO-PLANNER: Iterative force guide optimization ---
        for iteration in range(OPT_ITERS):
            for i in range(1, N_WP - 1):
                wp = np.array([wp_n[i], wp_e[i]])

                # Distância de cada obstáculo a este waypoint
                diffs = obs - wp
                dists = np.sqrt(diffs[:, 0]**2 + diffs[:, 1]**2)

                # Encontrar obstáculo mais próximo
                nearest_idx = np.argmin(dists)
                nearest_dist = dists[nearest_idx]

                if nearest_dist < SAFE_DIST:
                    # Force guide: empurra waypoint para longe do obstáculo
                    push_dir = wp - obs[nearest_idx]
                    push_mag = math.sqrt(push_dir[0]**2 + push_dir[1]**2)
                    if push_mag > 0.001:
                        push_dir /= push_mag
                    else:
                        push_dir = np.array([gd_e, -gd_n])  # perpendicular ao goal

                    force = (SAFE_DIST - nearest_dist) * 0.5
                    wp_n[i] += push_dir[0] * force
                    wp_e[i] += push_dir[1] * force

                # Suavização: puxa para média dos vizinhos
                avg_n = (wp_n[i-1] + wp_n[i+1]) / 2.0
                avg_e = (wp_e[i-1] + wp_e[i+1]) / 2.0
                wp_n[i] = wp_n[i] * 0.7 + avg_n * 0.3
                wp_e[i] = wp_e[i] * 0.7 + avg_e * 0.3

        # Seguir segundo waypoint
        target_n = wp_n[1]
        target_e = wp_e[1]
        target_dist = math.sqrt(target_n**2 + target_e**2)

        if target_dist > 0.05:
            vel_n = (target_n / target_dist) * MAX_SPEED
            vel_e = (target_e / target_dist) * MAX_SPEED
        else:
            vel_n = goal_vel_n
            vel_e = goal_vel_e

        vel_z = 0.0

        speed = math.sqrt(vel_n**2 + vel_e**2)
        if speed > MAX_SPEED:
            vel_n = (vel_n / speed) * MAX_SPEED
            vel_e = (vel_e / speed) * MAX_SPEED

        if -self.odometry.position[2] < 1.5 and vel_z > 0:
            vel_z = 0.0

        if self.cmdloop_control % 50 == 0:
            min_obs = float(np.min(dist_2d[valid]))
            self.log_debug(
                f"EGO [Drone {self.instance_id}]: min={min_obs:.2f}m "
                f"wp1=({target_n:.2f},{target_e:.2f}) "
                f"NED=({vel_n:.2f},{vel_e:.2f},{vel_z:.2f})"
            )

        return vel_n, vel_e, vel_z

    def avoid_obstacles_3d_OLD_RRT2(self, goal_vel_n, goal_vel_e):
        """OLD RRT2"""
        DETECT_DIST = 5.0
        GRID_SIZE = 10.0        # metros, área local
        GRID_RES = 0.2          # metros por célula
        GRID_CELLS = int(GRID_SIZE / GRID_RES)  # 50x50
        RRT_ITERS = 500         # iterações por planejamento
        RRT_STEP = 1.0          # metros, tamanho do passo
        RRT_REWIRE_RADIUS = 2.0 # metros, raio de rewiring
        GOAL_BIAS = 0.4         # probabilidade de amostrar o goal
        REPLAN_INTERVAL = 1     # replanejar a cada tick
        DRONE_RADIUS = 0.4      # raio de segurança

        if self.lidar_points_3d is None or len(self.lidar_points_3d) == 0:
            return goal_vel_n, goal_vel_e, 0.0

        # --- OCCUPANCY GRID LOCAL (body frame projetado em 2D) ---
        points = self.lidar_points_3d
        x_b, y_b, z_b = points[:, 0], points[:, 1], points[:, 2]
        dist_2d = np.sqrt(x_b**2 + y_b**2)

        valid = (dist_2d >= 0.3) & (dist_2d <= DETECT_DIST) & (np.abs(z_b) < 2.0)
        if not np.any(valid):
            return goal_vel_n, goal_vel_e, 0.0

        xv, yv = x_b[valid], y_b[valid]

        # Rotacionar body → NED
        cos_yaw = np.cos(self.true_yaw)
        sin_yaw = np.sin(self.true_yaw)
        n_pts = xv * cos_yaw - yv * sin_yaw
        e_pts = xv * sin_yaw + yv * cos_yaw

        # Grid centrado no drone (NED frame local)
        grid = np.zeros((GRID_CELLS, GRID_CELLS), dtype=np.uint8)
        half = GRID_SIZE / 2.0

        # Marcar obstáculos no grid (inflados pelo raio do drone)
        inflate_cells = int(math.ceil(DRONE_RADIUS / GRID_RES))
        gi = ((n_pts + half) / GRID_RES).astype(int)
        gj = ((e_pts + half) / GRID_RES).astype(int)
        for ii, jj in zip(gi, gj):
            for di in range(-inflate_cells, inflate_cells + 1):
                for dj in range(-inflate_cells, inflate_cells + 1):
                    ni, nj = ii + di, jj + dj
                    if 0 <= ni < GRID_CELLS and 0 <= nj < GRID_CELLS:
                        grid[ni, nj] = 1

        # Posição do drone no grid (centro)
        drone_gi = GRID_CELLS // 2
        drone_gj = GRID_CELLS // 2

        # Goal em NED relativo ao drone
        curr_n = self.odometry.position[0]
        curr_e = self.odometry.position[1]
        tgt_n = self.finish_mission_position['N']
        tgt_e = self.finish_mission_position['E']
        goal_rel_n = tgt_n - curr_n
        goal_rel_e = tgt_e - curr_e
        goal_dist = math.sqrt(goal_rel_n**2 + goal_rel_e**2)

        # Limitar goal ao grid local
        if goal_dist > half * 0.9:
            goal_rel_n = (goal_rel_n / goal_dist) * half * 0.9
            goal_rel_e = (goal_rel_e / goal_dist) * half * 0.9

        goal_gi = int((goal_rel_n + half) / GRID_RES)
        goal_gj = int((goal_rel_e + half) / GRID_RES)
        goal_gi = max(0, min(GRID_CELLS - 1, goal_gi))
        goal_gj = max(0, min(GRID_CELLS - 1, goal_gj))

        # Se goal está em obstáculo, mover para célula livre mais próxima
        if grid[goal_gi, goal_gj] == 1:
            best_d = 9999
            for di in range(-5, 6):
                for dj in range(-5, 6):
                    ni, nj = goal_gi + di, goal_gj + dj
                    if 0 <= ni < GRID_CELLS and 0 <= nj < GRID_CELLS and grid[ni, nj] == 0:
                        d = di*di + dj*dj
                        if d < best_d:
                            best_d = d
                            goal_gi, goal_gj = ni, nj

        # --- RRT* ---
        # Verificar se precisa replanejar
        if not hasattr(self, '_rrt_path') or self._rrt_path is None or self.cmdloop_control % REPLAN_INTERVAL == 0:
            self._rrt_path = self._run_rrt_star(
                grid, GRID_CELLS, GRID_RES, half,
                drone_gi, drone_gj, goal_gi, goal_gj,
                RRT_ITERS, RRT_STEP, RRT_REWIRE_RADIUS, GOAL_BIAS
            )

        # --- SEGUIR O CAMINHO ---
        if self._rrt_path is not None and len(self._rrt_path) >= 2:
            # Próximo waypoint (pular o primeiro que é a posição atual)
            wp_idx = min(2, len(self._rrt_path) - 1)
            wp_gi, wp_gj = self._rrt_path[wp_idx]

            # Converter grid → NED relativo ao drone
            wp_n = wp_gi * GRID_RES - half
            wp_e = wp_gj * GRID_RES - half

            # Velocidade na direção do waypoint
            wp_dist = math.sqrt(wp_n**2 + wp_e**2)
            if wp_dist > 0.1:
                vel_n = (wp_n / wp_dist) * MAX_SPEED
                vel_e = (wp_e / wp_dist) * MAX_SPEED
            else:
                vel_n = goal_vel_n
                vel_e = goal_vel_e
        else:
            # Sem caminho encontrado — subir
            vel_n = 0.0
            vel_e = 0.0

        # Vertical
        zv_valid = z_b[valid]
        dv_valid = dist_2d[valid]
        danger_above = float(np.sum(1.0 / (dv_valid[zv_valid > 0.5]**2))) if np.any(zv_valid > 0.5) else 0.0
        danger_below = float(np.sum(1.0 / (dv_valid[zv_valid < -0.5]**2))) if np.any(zv_valid < -0.5) else 0.0

        if self._rrt_path is None or len(self._rrt_path) < 2:
            vel_z = -MAX_SPEED  # sem caminho → subir
        elif danger_above > 0 or danger_below > 0:
            vel_z = -MAX_SPEED * 0.3 if danger_above < danger_below else MAX_SPEED * 0.3
        else:
            vel_z = 0.0

        # Clamp
        speed = math.sqrt(vel_n**2 + vel_e**2)
        if speed > MAX_SPEED:
            vel_n = (vel_n / speed) * MAX_SPEED
            vel_e = (vel_e / speed) * MAX_SPEED

        vel_z = max(-MAX_SPEED, min(MAX_SPEED, vel_z))
        if -self.odometry.position[2] < 1.5 and vel_z > 0:
            vel_z = 0.0

        if self.cmdloop_control % 50 == 0:
            path_len = len(self._rrt_path) if self._rrt_path else 0
            n_obs = int(np.sum(grid))
            self.log_debug(
                f"RRT* [Drone {self.instance_id}]: path={path_len} obs_cells={n_obs} "
                f"goal_grid=({goal_gi},{goal_gj}) "
                f"NED=({vel_n:.2f},{vel_e:.2f},{vel_z:.2f})"
            )

        return vel_n, vel_e, vel_z

    def _run_rrt_star(self, grid, grid_cells, grid_res, half,
                      start_i, start_j, goal_i, goal_j,
                      max_iters, step_size, rewire_radius, goal_bias):
        """Executa RRT* no grid e retorna caminho como lista de (i, j)."""
        step_cells = step_size / grid_res
        rewire_cells = rewire_radius / grid_res

        # Árvore: listas de nós
        nodes_i = [float(start_i)]
        nodes_j = [float(start_j)]
        parents = [-1]
        costs = [0.0]

        goal_reached = False
        goal_node_idx = -1

        for iteration in range(max_iters):
            # Amostrar ponto aleatório (com goal bias)
            if np.random.random() < goal_bias:
                sample_i = float(goal_i)
                sample_j = float(goal_j)
            else:
                sample_i = np.random.uniform(0, grid_cells)
                sample_j = np.random.uniform(0, grid_cells)

            # Encontrar nó mais próximo
            ni = np.array(nodes_i)
            nj = np.array(nodes_j)
            dists = (ni - sample_i)**2 + (nj - sample_j)**2
            nearest_idx = int(np.argmin(dists))

            # Steer: mover do mais próximo na direção da amostra
            dx = sample_i - nodes_i[nearest_idx]
            dy = sample_j - nodes_j[nearest_idx]
            d = math.sqrt(dx**2 + dy**2)
            if d < 0.1:
                continue
            if d > step_cells:
                dx = dx / d * step_cells
                dy = dy / d * step_cells

            new_i = nodes_i[nearest_idx] + dx
            new_j = nodes_j[nearest_idx] + dy

            # Verificar limites
            if new_i < 0 or new_i >= grid_cells or new_j < 0 or new_j >= grid_cells:
                continue

            # Verificar colisão no segmento (amostragem de pontos)
            if self._check_collision(grid, grid_cells,
                                     nodes_i[nearest_idx], nodes_j[nearest_idx],
                                     new_i, new_j):
                continue

            # Custo do novo nó via nearest
            seg_cost = math.sqrt(dx**2 + dy**2)
            new_cost = costs[nearest_idx] + seg_cost

            # RRT*: procurar vizinhos para rewiring
            best_parent = nearest_idx
            best_cost = new_cost

            ni_arr = np.array(nodes_i)
            nj_arr = np.array(nodes_j)
            neighbor_dists = (ni_arr - new_i)**2 + (nj_arr - new_j)**2
            neighbors = np.where(neighbor_dists < rewire_cells**2)[0]

            for n_idx in neighbors:
                n_cost = costs[n_idx] + math.sqrt((nodes_i[n_idx] - new_i)**2 + (nodes_j[n_idx] - new_j)**2)
                if n_cost < best_cost:
                    if not self._check_collision(grid, grid_cells,
                                                 nodes_i[n_idx], nodes_j[n_idx],
                                                 new_i, new_j):
                        best_cost = n_cost
                        best_parent = int(n_idx)

            # Adicionar nó
            new_idx = len(nodes_i)
            nodes_i.append(new_i)
            nodes_j.append(new_j)
            parents.append(best_parent)
            costs.append(best_cost)

            # Rewire vizinhos pelo novo nó
            for n_idx in neighbors:
                if int(n_idx) == best_parent:
                    continue
                potential_cost = best_cost + math.sqrt((nodes_i[n_idx] - new_i)**2 + (nodes_j[n_idx] - new_j)**2)
                if potential_cost < costs[n_idx]:
                    if not self._check_collision(grid, grid_cells,
                                                 new_i, new_j,
                                                 nodes_i[n_idx], nodes_j[n_idx]):
                        parents[int(n_idx)] = new_idx
                        costs[int(n_idx)] = potential_cost

            # Checar se alcançou o goal
            dist_to_goal = math.sqrt((new_i - goal_i)**2 + (new_j - goal_j)**2)
            if dist_to_goal < step_cells:
                if not goal_reached or best_cost + dist_to_goal < (costs[goal_node_idx] if goal_node_idx >= 0 else 9999):
                    goal_reached = True
                    goal_node_idx = new_idx

        if not goal_reached:
            return None

        # Extrair caminho
        path = []
        idx = goal_node_idx
        while idx >= 0:
            path.append((int(nodes_i[idx]), int(nodes_j[idx])))
            idx = parents[idx]
        path.reverse()
        return path

    def _check_collision(self, grid, grid_cells, i1, j1, i2, j2):
        """Check collision along segment."""
        n_samples = max(int(math.sqrt((i2-i1)**2 + (j2-j1)**2)) + 1, 2)
        for t in range(n_samples + 1):
            frac = t / n_samples
            ci = int(i1 + (i2 - i1) * frac)
            cj = int(j1 + (j2 - j1) * frac)
            if ci < 0 or ci >= grid_cells or cj < 0 or cj >= grid_cells:
                return True
            if grid[ci, cj] == 1:
                return True
        return False

    def avoid_uav(self, horizontal_velocity, targetZ, odometryZ):
        velocity = Vector3()
        velocity.x = horizontal_velocity.x
        velocity.y = horizontal_velocity.y

        sanitized_z = -odometryZ
        finalZVelocity = self.velocity.z if self.velocity.z != 0 else 0.001

        if odometryZ > -1.1:
            finalZVelocity = -MAX_SPEED
        elif sanitized_z > targetZ + 0.1:
            finalZVelocity = (1 + abs(sanitized_z - targetZ)) * MAX_SPEED

        elif sanitized_z < targetZ - 0.1:
            finalZVelocity = -(1 + abs(sanitized_z - targetZ)) * MAX_SPEED

        elif sanitized_z <= targetZ - 0.01 or sanitized_z >= targetZ + 0.01:
            finalZVelocity = (-1 if sanitized_z <= targetZ - 0.01 else 1) * MAX_SPEED

        self.get_logger().info(f"\nTARGET: {targetZ}\nRESULT FOR CONDITION: {(-1 if sanitized_z <= targetZ - 0.01 else 1)}\nODOMETRY Z: {odometryZ}\nSANITIZED_Z: {sanitized_z}\nFINAL VELOCITY: {finalZVelocity}")

        velocity.z = float(finalZVelocity)

        return velocity

    def avoid_uav_hardcore(self, horizontal_velocity, targetZ, odometryZ):
        velocity = Vector3()
        velocity.x = horizontal_velocity.x
        velocity.y = horizontal_velocity.y

        current_alt = -odometryZ
        error = targetZ - current_alt

        if current_alt < 1.0:
            finalZVelocity = -MAX_SPEED
        else:
            finalZVelocity = -error * 2.0
            finalZVelocity = max(-MAX_SPEED, min(MAX_SPEED, finalZVelocity))

        velocity.z = float(finalZVelocity)

        return velocity


class Vector2:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Vector2(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector2(self.x - other.x, self.y - other.y)

    def __mul__(self, other: float):
        return Vector2(self.x * other, self.y * other)

    def __div__(self, other: float):
        return Vector2(self.x / other, self.y / other)


# auxiliar functions

def get_coordinates(step):
    coordinates = step.strip("go:").split(",")
    coordinates = [float(c) for c in coordinates]

    return coordinates


def conversion_XYZ_NED(coordinates):
    return (coordinates[1], coordinates[0], -coordinates[2])


def deg2rad(deg: float):
    return deg * math.pi / 180.0


def is_in_danger_zone(reference_uav, intruder_uav):
    if np.sqrt((reference_uav[0] - intruder_uav[0])**2 + (reference_uav[1] - intruder_uav[1])**2 + (reference_uav[2] - intruder_uav[2])**2) < DANGER_ZONE_RADIUS:
        return True
    
    return False


import json
import os

MISSION_CONFIG_CACHE = None

def get_spawn_position(spawn_configuration, uav_instance, uav_number):
    global MISSION_CONFIG_CACHE
    
    if isinstance(spawn_configuration, str) and spawn_configuration.endswith('.json'):
        if MISSION_CONFIG_CACHE is None:
            try:
                with open(spawn_configuration, 'r') as f:
                    MISSION_CONFIG_CACHE = json.load(f)
            except Exception as e:
                print(f"Error loading mission config: {e}")
                return [0.0, 0.0]

        if uav_instance < len(MISSION_CONFIG_CACHE):
             spawn = MISSION_CONFIG_CACHE[uav_instance].get('spawn', [0.0, 0.0, 0.0])
             return [spawn[0], spawn[1]]
        else:
            return [0.0, 0.0]

    if spawn_configuration == 'l':
        coord_x = uav_instance
        coord_y = 0
    elif spawn_configuration == 's':
        width = math.ceil(math.sqrt(uav_number))
        coord_x = uav_instance % width
        coord_y = uav_instance // width
    else:
        coord_x = uav_instance
        coord_y = 0

    return [coord_x*3, coord_y*3]


def main(args=None):
    rclpy.init(args=args)

    namespace = sys.argv[1]
    uav_number = int(sys.argv[2])
    mission_mode = True if sys.argv[3] == 'true' else False
    mission_steps = sys.argv[4].strip("\n").split(";")
    spawn_configuration = sys.argv[5]
    lidar_topic = sys.argv[6] if len(sys.argv) > 6 else None

    offboard_control = OffboardControl(namespace, uav_number, mission_mode, mission_steps, spawn_configuration, lidar_topic)

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
