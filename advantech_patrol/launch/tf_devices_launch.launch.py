from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Static transform publisher for lidar
    # 180 degree roll rotation = π radians around x-axis
    # Translation: 0.4m up in z-axis
    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf_publisher',
        arguments=[
            '0', '0', '0.4',        # x, y, z translation
            '3.14159', '0', '0',    # roll, pitch, yaw (180° roll = π radians)
            'base_link',            # parent frame
            'laser'                 # child frame
        ],
        output='screen'
    )

    # Create and return the LaunchDescription with the static transform
    return LaunchDescription([
        lidar_static_tf,        # Static transform for lidar
    ])