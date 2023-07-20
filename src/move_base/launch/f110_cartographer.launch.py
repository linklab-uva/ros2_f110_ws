"""
  Copyright 2018 The Cartographer Authors
  Copyright 2022 Wyca Robotics (for the ros2 conversion)
  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at
       http://www.apache.org/licenses/LICENSE-2.0
  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
"""

from launch import LaunchDescription
import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch.actions import Shutdown

def generate_launch_description():

    ## ***** Launch arguments *****
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value = 'False')

    ## ***** File paths ******
    # pkg_share = FindPackageShare('cav_cartographer').find('cav_cartographer')
    # urdf_dir = os.path.join(pkg_share, 'urdf')
    # urdf_file = os.path.join(urdf_dir, 'f110.urdf')
    # with open(urdf_file, 'r') as infp:
    #     robot_desc = infp.read()

    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments = [
            '-configuration_directory', FindPackageShare('cav_cartographer').find('cav_cartographer') + '/config',
            '-configuration_basename', 'f110.lua'],
        # remappings = [
        #     ('scan', 'horizontal_laser_2d')],
        output = 'screen'
        )

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'occupancy_grid_node',
        parameters = [
            {'use_sim_time': False},
            {'resolution': 0.05}],
        )

    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        on_exit = Shutdown(),
        arguments = ['-d', FindPackageShare('cav_cartographer').find('cav_cartographer') + '/rviz' + 'demo_2d.rviz'],
        parameters = [{'use_sim_time': False}],
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        # Nodes
        #robot_state_publisher_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
        # rviz_node,
        # imu_converter
        # launch.actions.ExecuteProcess(
        #     cmd=['ros2', 'bag', 'play', '/home/john/IAC_bags/candidate_bag_files/rosbag2_2022_01_05-15_33_00/', '--topics', '/novatel_top/rawimu', '/luminar_points', '/vehicle/uva_odometry', '/tf', '/tf_static', '--remap', '/luminar_points:=/points2', '/vehicle/uva_odometry:=/odom', '-r', '.5'],
        #     output='screen'
        # )
    ])