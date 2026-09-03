#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Braden Wagstaff"
__contact__ = "braden@arkelectron.com"

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import math
import os


LINE = 'l'
SQUARE = 's'


def get_spawn_position(spawn_configuration, uav_instance, uav_number):
    if spawn_configuration == LINE:
        coord_x = uav_instance
        coord_y = 0
    elif spawn_configuration == SQUARE:
        width = math.ceil(math.sqrt(uav_number))
        coord_x = uav_instance % width
        coord_y = uav_instance // width

    return f"{coord_y*3},{coord_x*3}"



def generate_uav_nodes(context):
    import json

    current_directory = os.path.dirname(__file__)
    mission_json_path = os.path.join(current_directory, 'mission.json')

    with open(mission_json_path, 'r') as f:
        drones_config = json.load(f)

    uav_number = len(drones_config)
    mission_mode = LaunchConfiguration('mission_mode').perform(context)

    print(f"UAV number: {uav_number} | Mission mode: {mission_mode} | Source: {mission_json_path}")

    nodes = []

    nodes.append(
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='processes',
            name='processes',
            arguments=['0']
        )
    )

    for i, drone in enumerate(drones_config):
        spawn_pos = drone.get('spawn', [0.0, 0.0, 0.0])
        spawn_position = f"{spawn_pos[0]},{spawn_pos[1]},{spawn_pos[2]}"

        mission_steps_list = drone.get('mission', [])
        mission_steps = ";".join(mission_steps_list)

        if mission_mode != 'true':
            mission_steps = "go:0.0,0.0,0.0"

        ros_topic = f'/x500_lidar_3d_{i+1}/lidar_3d/points'

        gz_topic_candidates = []
        base_gz = f'/world/experimento_6/model'
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
            config_content.append( '  ros_type_name: sensor_msgs/msg/PointCloud2')
            config_content.append( '  gz_type_name: gz.msgs.PointCloudPacked')
            config_content.append( '  direction: GZ_TO_ROS')

        config_file_path = f'/tmp/lidar_bridge_config_{i+1}.yaml'
        with open(config_file_path, 'w') as f:
            f.write('\n'.join(config_content) + '\n')


        nodes.extend([
            Node(
                package='px4_offboard',
                namespace=f'px4_{i+1}',
                executable='processes',
                name='processes',
                arguments=[f'{i+1}', spawn_position]
            ),
            Node(
                package='px4_offboard',
                namespace=f'px4_{i+1}',
                executable='visualizer',
                name='visualizer',
                arguments=[f'px4_{i+1}']
            )
        ])

        if mission_mode != 'true':
            nodes.append(
                Node(
                    package='px4_offboard',
                    namespace=f'px4_{i+1}',
                    executable='teleoperator',
                    name='teleoperator',
                    arguments=[f'px4_{i+1}'],
                    prefix='gnome-terminal --'
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
    uav_number_argument = DeclareLaunchArgument(
        'uav_number',
        default_value='1',
    )

    spawn_configuration_argument = DeclareLaunchArgument(
        'spawn_configuration',
        default_value=LINE,
    )

    mission_mode_argument = DeclareLaunchArgument(
        'mission_mode',
        default_value='false'
    )
    
    return LaunchDescription([
        uav_number_argument,
        spawn_configuration_argument,
        mission_mode_argument,
        OpaqueFunction(function=generate_uav_nodes)
    ])