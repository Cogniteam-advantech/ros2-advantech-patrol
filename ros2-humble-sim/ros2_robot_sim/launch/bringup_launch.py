from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the other launch files
    first_launch_file = os.path.join(
        get_package_share_directory('ros2_robot_sim'),
        'launch',
        'localization_launch.py'
    )

    second_launch_file = os.path.join(
        get_package_share_directory('ros2_robot_sim'),
        'launch',
        'navigation_launch.py'
    )

    # Define the ros2_robot_sim node
    ros2_robot_sim_node = Node(
        package='ros2_robot_sim',
        executable='ros2_robot_sim_node', 
        name='ros2_robot_sim_node',
        output='screen'
    )

    # Define the advantech camera AI node
    advantech_camera_ai_node = Node(
        package='advantech_camera_ai',
        executable='advantech_camera_ai_node',
        name='advantech_camera_ai_node',
        output='screen'
    )

    # Include the first launch file
    first_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(first_launch_file)
    )

    # Include the second launch file
    second_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(second_launch_file)
    )

    # Create and return the LaunchDescription with all actions
    return LaunchDescription([
        ros2_robot_sim_node,           # Launch the ros2_robot_sim node
        advantech_camera_ai_node,      # Launch the advantech camera AI node
        first_launch,                  # Include first launch file
        second_launch                  # Include second launch file
    ])