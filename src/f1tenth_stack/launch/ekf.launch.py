from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('localization')
    f1tenth = os.path.join(get_package_share_directory('f1tenth_stack'), 'f1tenth_stack')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/config.rviz')
    map_path = os.path.join(get_package_share_directory('localization'), 'maps')

    sim_time = DeclareLaunchArgument(name='use_sim_time', default_value='False', description='Flag to enable use_sim_time')
    rviz = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file')

    ld = LaunchDescription([sim_time, rviz])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': os.path.join(map_path, 'map2' + '.yaml')},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'output': 'screen'},
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    map_updater = Node(
        package='localization',
        executable='map_pub',
        name='map_updater',
        output='screen',
    )
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    robot_localization_node = Node(
         package='robot_localization',
         executable='ekf_node',
         name='ekf_filter_node',
         output='screen',
         parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    start_ros2_navigation_cmd = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
    launch_arguments = {'namespace': namespace,
                        'use_namespace': use_namespace,
                        'slam': slam,
                        'map': map_yaml_file,
                        'use_sim_time': use_sim_time,
                        'params_file': params_file,
                        'default_bt_xml_filename': default_bt_xml_filename,
                        'autostart': True}.items())

    ld.add_action(rviz_node)
    ld.add_action(map_updater)
    ld.add_action(robot_localization_node)
    ld.add_action(map_server_node)
    ld.add_action(nav_lifecycle_node)

    return ld