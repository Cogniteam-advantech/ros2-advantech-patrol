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

    rplidar_launch = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch',
        'sllidar_a2m8_launch.py'
    )

    tf_to_poses_launch = os.path.join(
        get_package_share_directory('tf_to_poses'),
        'launch',
        'bringup_launch.py'
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

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch)
    )

    tf_to_poses_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tf_to_poses_launch)
    )



    # Create and return the LaunchDescription with all actions
    return LaunchDescription([
        advantech_patrol_node,  # Launch the Advantech Patrol node
        advantech_camera_ai_node,
        slam_async_launch,      # Include SLAM Toolbox async launch
        nav2_launch,
        rplidar_launch,
        tf_to_poses_launch
    ])  