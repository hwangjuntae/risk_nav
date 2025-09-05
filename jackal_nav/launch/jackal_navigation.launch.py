#!/usr/bin/env python3

# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    map_dir = LaunchConfiguration(
        "map",
        default=os.path.join(
            get_package_share_directory("jackal_nav"), "maps", "office.yaml"
        ),
    )
    param_dir = LaunchConfiguration(
        "params_file",
        default=os.path.join(
            get_package_share_directory("jackal_nav"), "params", "jackal_navigation_params.yaml"
        ),
    )

    nav2_bringup_launch_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")

    rviz_config_dir = os.path.join(get_package_share_directory("jackal_nav"), "rviz", "jackal_navigation.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=map_dir, description="Full path to map file to load"),
            DeclareLaunchArgument(
                "params_file", default_value=param_dir, description="Full path to param file to load"
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use simulation clock if true"
            ),

            # 정적 TF 브로드캐스터 - map 프레임을 odom 프레임에 연결
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_transform_publisher',
                arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
                parameters=[{'use_sim_time': True}]
            ),

            # 맵 서버 - 정적 맵을 로드하여 RViz에 표시
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'yaml_filename': map_dir
                }]
            ),

            # TF 프레임 연결 (Isaac Sim 호환)
            # Isaac Sim은 이미 World -> odom -> base_link 연결을 제공함
            # map -> odom -> base_link 연결을 완성함
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, "rviz_launch.py")),
                launch_arguments={"namespace": "", "use_namespace": "False", "rviz_config": rviz_config_dir}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_bringup_launch_dir, "/bringup_launch.py"]),
                launch_arguments={"map": map_dir, "use_sim_time": use_sim_time, "params_file": param_dir}.items(),
            ),

            # Pointcloud to laserscan conversion for Jackal's 3D LiDAR
            Node(
                package='pointcloud_to_laserscan', 
                executable='pointcloud_to_laserscan_node',
                remappings=[('cloud_in', ['/Lidar/point_cloud']),
                            ('scan', ['/scan_converted'])],
                parameters=[{
                    'target_frame': 'sensor',  # velodyne_link 대신 sensor 사용
                    'transform_tolerance': 0.01,
                    'min_height': -0.4,
                    'max_height': 1.5,
                    'angle_min': -3.14159,  # -M_PI
                    'angle_max': 3.14159,   # M_PI
                    'angle_increment': 0.0087,  # M_PI/360.0
                    'scan_time': 0.1,
                    'range_min': 0.05,
                    'range_max': 100.0,
                    'use_inf': True,
                    'inf_epsilon': 1.0,
                    'use_sim_time': True,
                }],
                name='pointcloud_to_laserscan'
            ),
        ]
    )
