from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # config and args
    raceline = os.path.join(get_package_share_directory('pure_pursuit'), 'lines', 'line3.csv')

    raceline_la = DeclareLaunchArgument(
        'raceline',
        default_value=raceline,
        description='Localization configs')
    ld = LaunchDescription([raceline_la])

    # nodes
    vc_node = Node(
        package='pure_pursuit',
        executable='vehicle_controller',
        name='vehicle_controller'
    )
    fp_node = Node(
        package='pure_pursuit',
        executable='find_nearest_pose',
        name='find_nearest_pose',
        parameters=[{'trajectory_file_path': raceline}]
    )
    fg_node = Node(
        package='pure_pursuit',
        executable='find_nearest_goal',
        name='find_nearest_goal',
        parameters=[{'trajectory_file_path': raceline}]
    )
    # finalize
    ld.add_action(vc_node)
    ld.add_action(fp_node)
    ld.add_action(fg_node)

    return ld