# Portions of this file are from the Elsabot project by Scott Horton
# and others from LinoRobot2

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
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, logging
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, PythonExpression

def generate_launch_description():

    def prepare_nodes(context, *args, **kwargs):

        logger = logging.get_logger('launch.user')

        robot_base = context.launch_configurations['robot_base']
        topic_name = context.launch_configurations['topic_name']
        use_filter = context.launch_configurations['use_filter']

        lidar_topic = topic_name
        filter_config_path = ''

        if use_filter == 'True':
            filter_config_path = os.path.join(get_package_share_directory('elsabot_robot'), 'config',
                                              robot_base + '_lidar_scan_range_filter.yaml')
            # Only use filter if a config exists for the selected robot base
            if Path(filter_config_path).is_file():
                lidar_topic = 'scan_raw'
            else:                
                use_filter = False
                filter_config_path = ''
                

        logger.info("laser launch: robot_base: %s, lidar_topic: %s, use_filter: %s, filter_config_path: %s" %
                     (robot_base, lidar_topic, use_filter, filter_config_path))

        return [
            Node(
                name='rplidar_composition',
                package='rplidar_ros',
                executable='rplidar_composition',
                output='screen',
                remappings=[('scan', lidar_topic)],
                parameters=[{
                    'serial_port': [EnvironmentVariable('RPLIDAR_SERIAL_DEVICE'), ''],
                    'serial_baudrate': 115200,  # A1 / A2
                    'frame_id': LaunchConfiguration('frame_id'),
                    'inverted': False,
                    'angle_compensate': True,
                }]
            ),

            Node (
                package='laser_filters',
                executable='scan_to_scan_filter_chain',
                output='screen',
                remappings=[('scan_filtered', topic_name),
                            ('scan', lidar_topic)],
                parameters=[filter_config_path],
                condition=IfCondition(PythonExpression([str(use_filter)]))
            )
        ]        

    return LaunchDescription([

        DeclareLaunchArgument(
            name='robot_base', 
            default_value='ebot_2wd',
            description='Elsabot robot base variant to use.'
        ),

        DeclareLaunchArgument(
            name='topic_name', 
            default_value='scan',
            description='Laser Topic Name'
        ),

        DeclareLaunchArgument(
            name='frame_id', 
            default_value='laser',
            description='Laser Frame ID'
        ),

        DeclareLaunchArgument(
            name='use_filter', 
            default_value='True',
            description='True to use scan filter.'
        ),

        OpaqueFunction(function=prepare_nodes)
    ])

