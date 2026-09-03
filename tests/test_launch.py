#!/usr/bin/env python
"""
Launch file para testes automatizados.
Igual ao offboard_velocity_control.launch.py mas sem spawnar processes.py
(MicroXRCEAgent e PX4 são lançados externamente pelo script de teste).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os


def generate_uav_nodes(context):
    import json

    mission_json_path = LaunchConfiguration('mission_json').perform(context)
    world_name = LaunchConfiguration('world_name').perform(context)

    with open(mission_json_path, 'r') as f:
        drones_config = json.load(f)

    uav_number = len(drones_config)
    mission_mode = LaunchConfiguration('mission_mode').perform(context)

    print(f"[test_launch] UAVs: {uav_number} | Mission mode: {mission_mode} | Config: {mission_json_path}")

    nodes = []

    for i, drone in enumerate(drones_config):
        spawn_pos = drone.get('spawn', [0.0, 0.0, 0.0])
        spawn_position = f"{spawn_pos[0]},{spawn_pos[1]},{spawn_pos[2]}"

        mission_steps_list = drone.get('mission', [])
        mission_steps = ";".join(mission_steps_list)

        if mission_mode != 'true':
            mission_steps = "go:0.0,0.0,0.0"

        ros_topic = f'/x500_lidar_3d_{i+1}/lidar_3d/points'

        gz_topic_candidates = []
        base_gz = f'/world/{world_name}/model'
        suffix_lidar = 'model/lidar_3d/link/lidar_3d_link/sensor/lidar_3d_sensor/scan/points'

        if i == 0:
            gz_topic_candidates.append(f'{base_gz}/x500_lidar_3d_1/{suffix_lidar}')
            gz_topic_candidates.append(f'{base_gz}/x500_lidar_3d/{suffix_lidar}')
            gz_topic_candidates.append(f'{base_gz}/x500_lidar_3d_0/{suffix_lidar}')
        else:
            gz_topic_candidates.append(f'{base_gz}/x500_lidar_3d_{i+1}/{suffix_lidar}')

        config_content = []
        for gz_topic in gz_topic_candidates:
            config_content.append(f'- gz_topic_name: {gz_topic}')
            config_content.append(f'  ros_topic_name: {ros_topic}')
            config_content.append('  ros_type_name: sensor_msgs/msg/PointCloud2')
            config_content.append('  gz_type_name: gz.msgs.PointCloudPacked')
            config_content.append('  direction: GZ_TO_ROS')

        config_file_path = f'/tmp/lidar_bridge_test_{i+1}.yaml'
        with open(config_file_path, 'w') as f:
            f.write('\n'.join(config_content) + '\n')

        nodes.append(
            Node(
                package='px4_offboard',
                namespace=f'px4_{i+1}',
                executable='visualizer',
                name='visualizer',
                arguments=[f'px4_{i+1}']
            )
        )

        nodes.append(
            Node(
                package='px4_offboard',
                namespace=f'px4_{i+1}',
                executable='velocity_control',
                name='velocity',
                arguments=[f'px4_{i+1}', f'{uav_number}', mission_mode, mission_steps, mission_json_path, ros_topic],
            )
        )

        nodes.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name=f'lidar_bridge_{i+1}',
                namespace=f'px4_{i+1}',
                parameters=[{'config_file': config_file_path}],
                output='screen'
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mission_json'),
        DeclareLaunchArgument('mission_mode', default_value='true'),
        DeclareLaunchArgument('world_name', default_value='test'),
        OpaqueFunction(function=generate_uav_nodes)
    ])
