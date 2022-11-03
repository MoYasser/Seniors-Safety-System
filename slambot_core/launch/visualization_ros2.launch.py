import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
]

def generate_launch_description():

    rviz_display = os.path.join(
      get_package_share_directory('slambot_core'),
      'rviz',
      'slamtool.rviz')

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_display],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen')

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(rviz2_node)
    return ld
