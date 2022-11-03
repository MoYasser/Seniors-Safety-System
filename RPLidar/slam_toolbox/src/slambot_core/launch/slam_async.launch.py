import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='use_sim_time'),
]

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    pkg_slambot_description = get_package_share_directory('slambot_core')
    
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
                  
                  
    xacro_file = PathJoinSubstitution([pkg_slambot_description,
                                       'urdf',
                                       LaunchConfiguration('model'),
                                       'turtlebot4.urdf.xacro'])

    start_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': Command(['xacro', ' ', xacro_file, ' ', 'gazebo:=ignition'])},
        ],
    )

    slam_config = PathJoinSubstitution(
        ['slambot_core', 'config', 'slam_async.yaml'])

    start_async_slam_toolbox_node = Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
              slam_config,
              {'use_sim_time': True}
            ],
        )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(start_robot_state_publisher_node)
    ld.add_action(start_async_slam_toolbox_node)

    return ld
