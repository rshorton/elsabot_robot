# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, EqualsSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    laser_launch_path = PathJoinSubstitution(
        [FindPackageShare('elsabot_robot'), 'launch', 'lasers.launch.py']
    )
    
    return LaunchDescription([

        DeclareLaunchArgument(
            name='robot_base', 
            default_value='ebot_2wd',
            description='Elsabot robot base variant to use.'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(laser_launch_path),
            launch_arguments={
                'robot_base': LaunchConfiguration('robot_base')
            }.items()
        ),

        # Jeep base uses IMU mounted on the compute rack
        Node(
            condition=IfCondition(
                EqualsSubstitution(LaunchConfiguration('robot_base'), 'jeep')),
            package='bno08x_driver',  
            executable='bno08x_driver',  
            name='bno08x_driver',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('elsabot_robot'), 'config', 'bno085_i2c.yaml')],
            remappings=[('/imu', '/ebot/imu/data'), ('/magnetic_field', '/ebot/imu/mag')]
        )
    ])

