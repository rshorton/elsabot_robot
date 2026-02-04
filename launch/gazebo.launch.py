# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Various modifications by Scott Horton for Elsabot robots

import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, logging
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    logger = logging.get_logger('launch.user')

    use_gps = LaunchConfiguration('use_gps')

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('elsabot_robot'), 'launch', 'description.launch.py']
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_path),
        launch_arguments={
            'use_sim_time': 'true',
            'publish_joints': 'true',
        }.items()
    )

    # Read the world SDF file and update the latitude and longitude of the origin        
    def prepare_sdf(context, *args, **kwargs):

        world_path = os.path.join(get_package_share_directory('elsabot_robot'), 'gazebo_worlds',
            context.launch_configurations['world_sdf_file'])

        with open(world_path) as f:
            world_sdf = f.read()

            world_sdf = world_sdf.replace('REPLACE_LATITUDE', context.launch_configurations['gps_origin_lat'])
            world_sdf = world_sdf.replace('REPLACE_LONGITUDE', context.launch_configurations['gps_origin_lon'])
            logger.info("prepared world sdf:\n %s" % world_sdf)

            prepared_sdf = tempfile.NamedTemporaryFile(mode='w', delete=False)
            f = open(prepared_sdf.name, "w")
            f.write(world_sdf)
            f.close()

            # Gazebo Sim
            return IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
                launch_arguments={
                    #'gz_args': '-r empty.sdf'
                    'gz_args': '-r ' +  prepared_sdf.name
                }.items(),
            ),   


    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),

        DeclareLaunchArgument(
            name='use_gps', 
            default_value='False',
            description='Set to True to use GPS'
        ),

        DeclareLaunchArgument(
            name='world_sdf_file', 
            default_value='my_world4.sdf',
            description='Default world SDF file'
        ),

        DeclareLaunchArgument(
            name='gps_origin_lat', 
            default_value='30.609866',
            description='Latitude of origin when using GPS'
        ),

        DeclareLaunchArgument(
            name='gps_origin_lon', 
            default_value='-96.340424',
            description='Longitude of origin when using GPS'
        ),

        OpaqueFunction(function=prepare_sdf),

        # Robot            
        Node(package='ros_gz_sim', executable='create',
            arguments=[
                '-name', 'elsabot_robot',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '-0.02',
                '-topic', '/robot_description'
            ],
            output='screen'
        ),

        # Bridge
        # The topics used were selected to mirror those used by the real Elsabot robot
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel/muxed@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/ebot/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/ebot/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
                '/ebot/imu/mag@sensor_msgs/msg/MagneticField@gz.msgs.Magnetometer',
                '/model/elsabot_robot/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/gps/fix@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat',
                '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/nav_camera/image@sensor_msgs/msg/Image@gz.msgs.Image'
            ],
            parameters=[
                {'qos_overrides./model/elsabot_robot.subscriber.reliability': 'reliable'}
            ],
            remappings=[
                # map from, to
                ('elsabot_robot/base_footprint/navsat', 'gps_link'),
                ('/world/empty/model/elsabot_robot/joint_state', 'joint_states'),
                ('/model/elsabot_robot/tf', 'tf')
                #('/navsat','/gps/fix'),
                #('/scan', '/scan_filtered'),
                # Seemed to have trouble setting the topic in the urdf to /imu/data_raw so just remap it here
                #('/imu/data','/imu/data_raw')
            ],
            output='screen'
        ),

        Node(
            package='cmd_vel_timeout',
            executable='command_timeout',
            name='command_timeout_node',
            output='screen',
            remappings=[
                ('cmd_vel', 'cmd_vel/muxed')
            ]
        ),

        # Used to simulate cmd_vel filtering for drive control mode
        # where the driver enables movement along path when
        # accel pedal pressed.
        #  Enable mode: ros2 topic pub /driver_control_mode std_msgs/msg/Int8 data:\ 2\
        #  Sim pedal press: ros2 topic pub /accel_pedal_pressed std_msgs/msg/Bool data:\ true\ 
        # Defaults to accel pedal not controlling.
        # This node publishes to cmd_vel_filtered

        #Node(
        #    package='elsabot_jeep_sim',
        #    executable='elsabot_jeep_sim',
        #    name='elsabot_jeep_sim',
        #    output='screen'
        #),

        robot_description
    ])
  
