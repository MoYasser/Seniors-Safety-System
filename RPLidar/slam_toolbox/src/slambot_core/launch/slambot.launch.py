import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.actions import LogInfo

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('world', default_value='depot',
                          description='World name'),
    DeclareLaunchArgument('robot_name', default_value='turtlebot4',
                          description='Robot name')
]

def generate_launch_description():

    namespace = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    rplidar_launchDescription = Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'rplidar_link',
                'inverted': False,
                'angle_compensate': True,
            }])
            
    rplidar_stf = Node(
            name='rplidar_stf',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '0', '0', '0', '0', '0.0', '0.0',
                'rplidar_link', [LaunchConfiguration('robot_name'), '/rplidar_link/rplidar']]
        )
        
    lidar_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='lidar_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            ['/world/', LaunchConfiguration('world'),
             '/model/', LaunchConfiguration('robot_name'),
             '/link/rplidar_link/sensor/rplidar/scan' +
             '@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan']
        ],
        remappings=[
            (['/world/', LaunchConfiguration('world'),
              '/model/', LaunchConfiguration('robot_name'),
              '/link/rplidar_link/sensor/rplidar/scan'],
             '/scan')
        ])
        


    slambotaux_launchDescription = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slambot_core'),
                'launch',
                'slambotaux.launch.py')),
        launch_arguments={}.items()
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(rplidar_launchDescription)
    ld.add_action(rplidar_stf)
    ld.add_action(lidar_bridge)
    ld.add_action(slambotaux_launchDescription)

    return ld
