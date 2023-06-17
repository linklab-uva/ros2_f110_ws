# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    car_namespace = ""
    use_sim_time = False

    vesc_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'vesc.yaml'
    )
    ekf_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'ekf.yaml'
    )
    urdf_file = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'urdf',
        'f110.urdf'
    )
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    sensors_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'sensors.yaml'
    )
    mux_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'mux.yaml'
    )

    vesc_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config,
        description='Descriptions for vesc configs')
    sensors_la = DeclareLaunchArgument(
        'sensors_config',
        default_value=sensors_config,
        description='Descriptions for sensor configs')
    mux_la = DeclareLaunchArgument(
        'mux_config',
        default_value=mux_config,
        description='Descriptions for ackermann mux configs')

    ld = LaunchDescription([vesc_la, sensors_la, mux_la])

    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        namespace=car_namespace,
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        namespace=car_namespace,
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        namespace=car_namespace,
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    throttle_interpolator_node = Node(
        package='f1tenth_stack',
        executable='throttle_interpolator',
        namespace=car_namespace,
        name='throttle_interpolator',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        namespace=car_namespace,
        name='urg_node',
        parameters=[LaunchConfiguration('sensors_config')]
    )
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        namespace=car_namespace,
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
        remappings=[('ackermann_cmd_out', 'ackermann_drive')]
    )

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=car_namespace,
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf_file]
    )

    ekf_node = Node(
         package='robot_localization',
         executable='ekf_node',
         name='ekf_filter_node',
         output='screen',
         parameters=[ekf_config, {'use_sim_time': use_sim_time}]
    )

    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    ld.add_action(throttle_interpolator_node)
    ld.add_action(urg_node)
    ld.add_action(ekf_node)
    ld.add_action(ackermann_mux_node)
    ld.add_action(robot_state_publisher)

    return ld