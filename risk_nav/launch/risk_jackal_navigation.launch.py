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
                }],
                name='pointcloud_to_laserscan'
            ),

            # === 추가: Risk Nav Core 노드들 ===

            # 1) LLM 위험 추론(mock)
            Node(
                package='risk_nav',
                executable='llm_risk_node',
                name='llm_risk_node',
                parameters=[{'gating': True, 'unknown_only': True, 'danger_keywords': ['door','corner','blind']}],
                output='screen'
            ),

            # 2) Risk Layer (OccupancyGrid 퍼블리시)
            Node(
                package='risk_nav',
                executable='risk_layer_node',
                name='risk_layer_node',
                parameters=[{'resolution':0.1, 'width':200, 'height':200, 'origin_x':-10.0, 'origin_y':-10.0}],
                output='screen'
            ),

            # 3) cmd_vel 리스크 필터 (출력을 /cmd_vel 로 리매핑 → 실제 제어에 적용)
            Node(
                package='risk_nav',
                executable='cmd_vel_risk_filter',
                name='cmd_vel_risk_filter',
                parameters=[{'risk_gain': 0.8}],
                remappings=[('/cmd_vel_risk', '/cmd_vel')],   # 필터 출력이 /cmd_vel 로 나가도록
                output='screen'
            ),

            # (옵션) /scan_converted → /scan 으로 리매핑하여 Nav2 기존 파라미터와 맞추기
            # Node(
            #     package='topic_tools',
            #     executable='relay',
            #     name='scan_alias',
            #     arguments=['/scan_converted', '/scan'],
            # ),
        ]
    ) 