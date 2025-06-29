from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to SLAM Toolbox async launch file
    slam_toolbox_async_launch = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    nav2_launch = os.path.join(
        get_package_share_directory('advantech_patrol'),
        'launch',
        'navigation_launch.launch.py'
    )

    # Define the RPLidar A2 node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }]
    )

    # Define the Advantech Patrol node
    advantech_patrol_node = Node(
        package='advantech_patrol',
        executable='advantech_patrol_node',
        name='advantech_patrol_node',
        output='screen'
    )

    # Define the advantech camera AI node
    advantech_camera_ai_node = Node(
        package='advantech_camera_ai',
        executable='advantech_camera_ai_node',
        name='advantech_camera_ai_node',
        output='screen'
    )

    # # Include the SLAM Toolbox async launch file
    slam_async_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_async_launch)
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch)
    )



    # Create and return the LaunchDescription with all actions
    return LaunchDescription([
        rplidar_node,           # Launch the RPLidar A2 node
        advantech_patrol_node,  # Launch the Advantech Patrol node
        advantech_camera_ai_node,
        slam_async_launch,      # Include SLAM Toolbox async launch
        nav2_launch
    ])  