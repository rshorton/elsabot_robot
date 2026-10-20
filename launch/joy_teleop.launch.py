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

# Various modifications by Scott Horton for Elsabot robots

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, logging
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    def prepare_nodes(context, *args, **kwargs):
        logger = logging.get_logger('launch.user')

        robot_base = context.launch_configurations['robot_base']
        cmd_vel_topic = context.launch_configurations['cmd_vel_topic']

        if robot_base == 'jeep':
            config_path = os.path.join(get_package_share_directory('elsabot_robot'), 'config', 'joy_jeep.yaml')
        else:
            # Fix - use separate configs for other base variants            
            config_path = os.path.join(get_package_share_directory('elsabot_robot'), 'config', 'joy.yaml')

        logger.info("joy_telop launch: robot_base: %s, config_path: %s" % (robot_base, config_path))

        return [
            Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='teleop_twist_joy_node',
                output='screen',
                parameters=[config_path],
                remappings=[('cmd_vel', cmd_vel_topic)]
            )
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            name='robot_base', 
            default_value='ebot_2wd',
            description='Elsabot robot base variant to use.'
        ),

        DeclareLaunchArgument(
            name='cmd_vel_topic', 
            default_value='cmd_vel',
            description='Topic to publish cmd_vel messages'
        ),        

        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_linux_node',
            output='screen',
        ),

        OpaqueFunction(function=prepare_nodes)
    ])