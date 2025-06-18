import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import Image
# import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import yaml
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseStamped, TransformStamped
# import tf_transformations
from tf2_ros import TransformBroadcaster
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import Path
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import Int32, String
import math
import yaml
import datetime
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile, ReliabilityPolicy
import random

class SimNode(Node):
    def __init__(self):
        super().__init__('ros2_robot_sim_node')
        
        
        self.image_map = cv2.imread("/home/cogniteam-user/advantech_patrol_ws/src/ros2-advantech-patrol/ros2-humble-sim/ros2_robot_sim/resource/map/map.pgm",0)
        self.image_map = cv2.flip(self.image_map, 0)

        self.map_info_dict = self.load_map_yaml("/home/cogniteam-user/advantech_patrol_ws/src/ros2-advantech-patrol/ros2-humble-sim/ros2_robot_sim/resource/map/map.yaml")
        
        self.max_lin_vel = 1.0
        self.min_lin_vel = -1.0
        
        self.min_rad_per_second = -0.785398
        self.max_rad_per_second = 0.785398
        
        self.global_frame = 'map'
        self.base_frame  = 'base_link'
        self.odom_frame = 'odom'
        
        self.set_initial_pose()  
        
        self.nav = BasicNavigator()   
        
        # Create callback groups
        self.cmd_vel_group = MutuallyExclusiveCallbackGroup()
        self.nav2_group = MutuallyExclusiveCallbackGroup() 
        self.timer_group = MutuallyExclusiveCallbackGroup()   
        self.cloud_group = MutuallyExclusiveCallbackGroup()   

        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        
        self.marker_array_subscriber = self.create_subscription(
            MarkerArray,
            '/cogniteam_ros2_sim/markers_input',
            self.markers_callback,
            qos_profile
        )

        # Subscribe to cmd_vel
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.cmd_vel_callback,
            10,
            callback_group=self.cmd_vel_group
        )
        
         # Subscribe goal
        self.goal_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/cogniteam_ros2_sim/goal',
            self.goal_callback,
            10,
            callback_group=self.nav2_group
        )
        
        # Subscribe joy
        self.joy_subscriber = self.create_subscription(
            Joy,
            '/cogniteam_ros2_sim/joy',
            self.joy_callback,
            10,
             callback_group=self.cmd_vel_group
        )

        # Publish Odometry
        self.robot_pose_publisher = self.create_publisher(PoseStamped, 
                                                            '/cogniteam_ros2_sim/robot_pose', 10)

        self.path_publisher = self.create_publisher(Path, '/cogniteam_ros2_sim/robot_path', 10)

        self.battery_voltage_publisher = self.create_publisher(Int32, '/cogniteam_ros2_sim/battery_voltage', 10)

        self.date_publisher = self.create_publisher(String, '/cogniteam_ros2_sim/date', 10)

        self.marker_array_publisher = self.create_publisher(MarkerArray, '/cogniteam_ros2_sim/marker_array',  qos_profile)
        
        
        self.scan_publisher = self.create_publisher(LaserScan, '/cogniteam_ros2_sim/scan', 10)

        self.cloud_publisher = self.create_publisher(PointCloud2, '/cogniteam_ros2_sim/pointcloud', 10)
        
        self.markers_output_publisher = self.create_publisher(MarkerArray, '/cogniteam_ros2_sim/markers_output',  qos_profile)


        # Add a publisher for the compressed image
        self.image_publisher = self.create_publisher(CompressedImage, '/cogniteam_ros2_sim/compressed_image', 10, )
        self.bridge = CvBridge()
        
        # Timer to publish odometry
        self.odom_timer = self.create_timer(0.1, self.publish_odometry, self.timer_group)
        
        

        # self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)

        # Initialize robot state
       
        self.current_velocity = Twist()
        
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_timer = self.create_timer(0.1, self.tf_timer_callback, callback_group=self.timer_group)


    
    def joy_callback(self, msg: Joy):
        # Right stick vertical axis 
        right_stick_y = msg.axes[3]  # Forward/Back movement of the right stick
        
        # Normalize to linear velocity range
        if right_stick_y > 0:
            # Moving forward (up), interpolate between 0 and max_lin_vel
            linear_velocity = right_stick_y * self.max_lin_vel
        else:
            # Moving backward (down), interpolate between min_lin_vel and 0
            linear_velocity = right_stick_y * abs(self.min_lin_vel)

        # Left stick horizontal axis (axes[0]) controls angular velocity
        left_stick_x = msg.axes[0]  # Left/Right movement of the left stick
        
        # Normalize to angular velocity range
        if left_stick_x > 0:
            # Moving right, interpolate between 0 and max_rad_per_second
            angular_velocity = left_stick_x * self.max_rad_per_second
        else:
            # Moving left, interpolate between min_rad_per_second and 0
            angular_velocity = left_stick_x * abs(self.min_rad_per_second)

        # Create Twist message and publish
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity
        
        if self.robot_pose is not None:

            self.apply_twist_to_pose(twist_msg, 0.1)

        
        
    def load_map_yaml(self,yaml_file_path):
        with open(yaml_file_path, 'r') as file:
            try:
                map_data = yaml.safe_load(file)
                return map_data
            except yaml.YAMLError as exc:
                print(f"Error loading YAML file: {exc}")
                return None
        
    def convert_pose_to_pix(self, pose: PoseStamped):
        # Calculate pixel coordinates
        x_pix = (pose.pose.position.x - self.map_info_dict['origin'][0]) / self.map_info_dict['resolution']
        y_pix = (pose.pose.position.y - self.map_info_dict['origin'][1]) / self.map_info_dict['resolution']

        # Create a point (in OpenCV, this would be a tuple)
        p = (int(x_pix), int(y_pix))

        return p

    def convert_pix_to_pose(self,pixel):
   
        pose = PoseStamped()
        
        pose.header.frame_id = self.global_frame

        pose.pose.position.x = (pixel[0] * self.map_info_dict['resolution']) + self.map_info_dict['origin'][0]
        pose.pose.position.y = (pixel[1] * self.map_info_dict['resolution']) + self.map_info_dict['origin'][1]
        pose.pose.position.z = 0.0

        # Assuming `q` is an instance of Quaternion
        pose.pose.orientation.w = 1.0

        return pose
 
    def raycast_to_black_pixel(self, image, robot_pos, yaw, degree, max_range=None):
        
        height, width = image.shape

        # Convert degree to radians and adjust with yaw
        angle = yaw + math.radians(degree)

        # Unit direction vector for the ray
        direction = (math.cos(angle), math.sin(angle))
        
        # Initialize the robot's position
        x_robot, y_robot = robot_pos
        x, y = robot_pos
        
        step_size = 1  # Step by 1 pixel each time (can be adjusted)
        distance = 0

        
        while 0 <= int(x) < width and 0 <= int(y) < height:
            # Check the pixel value at the current ray position
            if image[int(y), int(x)] == 0:  # Check if the pixel is black (0)

                distance_from_robot = math.sqrt((x_robot - x) ** 2 + (y_robot - y) ** 2)

                return (int(x), int(y)), distance_from_robot
            
            # Move along the ray direction
            x += direction[0] * step_size
            y += direction[1] * step_size

            # Increase distance covered
            distance += step_size

            # If a max range is provided, stop if we exceed it
            if max_range and distance >= max_range:
                break
       
        # Return None if no black pixel was found
        return None, None

    
        
    def markers_callback(self, msg):
        
        self.markers_output_publisher.publish(msg)
        
    def goal_callback(self, msg):
        # navigation_launch.py
        print('goal !!')

        self.nav.setInitialPose(self.robot_pose)

        nav_goal = PoseStamped()
        # Copy the header from PoseWithCovarianceStamped to PoseStamped
        nav_goal.header = msg.header
        # Copy the pose from PoseWithCovarianceStamped to PoseStamped (ignoring covariance)
        nav_goal.pose = msg.pose.pose
        wanted_path = self.nav.getPath(self.robot_pose, nav_goal)
        
        if wanted_path == None:
            return
        
        if len(wanted_path.poses) ==  0:
            self.get_logger().info('bad path !!')
            return
            
        smoothed_path = self.nav.smoothPath(wanted_path)
        
        self.path_publisher.publish(wanted_path)


        self.nav.goToPose(nav_goal)
        # while not self.nav.isTaskComplete():   
        #     rclpy.spin_once(self)        
            # feedback = self.nav.getFeedback()
            # if feedback.navigation_time.nanoseconds / 1e9 > 600:
            #     self.nav.cancelTask()      

        # result = self.nav.getResult()
        # if result == TaskResult.SUCCEEDED:
        #     print('Goal succeeded!')
        # elif result == TaskResult.CANCELED:
        #     print('Goal was canceled!')
        # elif result == TaskResult.FAILED:
        #     print('Goal failed!')
        
        self.get_logger().info('roal reached !!')


    def publish_date(self):
        
        msg = String()
        msg.data = str(datetime.datetime.now().strftime("%d/%m/%Y %H:%M:%S"))
        
        self.date_publisher.publish(msg)
    
    
    def publish_battery_voltage(self):
        msg = Int32()
        msg.data = 32

        self.battery_voltage_publisher.publish(msg)
        
    def quaternion_to_euler(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        
        return qx, qy, qz, qw

    def apply_twist_to_pose(self, twist, delta_time):
        # Extract the current position and orientation
        x = self.robot_pose.pose.position.x
        y = self.robot_pose.pose.position.y
        z = self.robot_pose.pose.position.z
        orientation = self.robot_pose.pose.orientation

        # Convert orientation quaternion to Euler angles
        _, _, yaw = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)

        # Linear velocities (in the robot's local frame)
        linear_x = twist.linear.x
        linear_y = twist.linear.y
        linear_z = twist.linear.z

        # Angular velocity (yaw)
        angular_z = twist.angular.z

        # Update yaw with angular velocity
        yaw += angular_z * delta_time

        # Calculate the new position in the world frame
        x += (linear_x * math.cos(yaw) - linear_y * math.sin(yaw)) * delta_time
        y += (linear_x * math.sin(yaw) + linear_y * math.cos(yaw)) * delta_time
        z += linear_z * delta_time

        # Convert the updated yaw back to a quaternion
        new_qx, new_qy, new_qz, new_qw = self.euler_to_quaternion(0, 0, yaw)

        # Update the PoseStamped message
        self.robot_pose = PoseStamped()
        self.robot_pose.header.frame_id = self.global_frame
        self.robot_pose.header.stamp = self.get_clock().now().to_msg()
        self.robot_pose.pose.position.x = x
        self.robot_pose.pose.position.y = y
        self.robot_pose.pose.position.z = z
        self.robot_pose.pose.orientation.x = new_qx
        self.robot_pose.pose.orientation.y = new_qy
        self.robot_pose.pose.orientation.z = new_qz
        self.robot_pose.pose.orientation.w = new_qw


    def set_initial_pose(self):
        # Initialize PoseStamped message
        self.robot_pose = PoseStamped()

        # Set the header
        self.robot_pose.header.frame_id = self.global_frame
        self.robot_pose.header.stamp = Time()

        # Set the position
        self.robot_pose.pose.position.x = -3.0
        self.robot_pose.pose.position.y = 0.0
        self.robot_pose.pose.position.z = 0.0

        # Set the orientation (Quaternion)
        self.robot_pose.pose.orientation.x = 0.0
        self.robot_pose.pose.orientation.y = 0.0
        self.robot_pose.pose.orientation.z = 0.711624
        self.robot_pose.pose.orientation.w = 0.702561
        
   

    def cmd_vel_callback(self, twist_msg):
        
        if self.robot_pose is not None:
            self.get_logger().info('cmd_vel_callback !!')

            self.apply_twist_to_pose(twist_msg, 0.1)


    def get_unique_color(self):
        """Generate a random unique color."""
        return random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1)

    def create_arrow_marker(self):
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "cogniteam_sim"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = 1.0
        marker.pose.position.y = 1.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r, marker.color.g, marker.color.b = self.get_unique_color()
        marker.color.a = 1.0
        return marker

    def create_cube_marker(self):
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "cogniteam_sim"
        marker.id = 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = 1.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.5
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.r, marker.color.g, marker.color.b = self.get_unique_color()
        marker.color.a = 1.0
        return marker

    def create_sphere_marker(self):
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "cogniteam_sim"
        marker.id = 2
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 2.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.5
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.r, marker.color.g, marker.color.b = self.get_unique_color()
        marker.color.a = 1.0
        return marker

    def create_cylinder_marker(self):
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "cogniteam_sim"
        marker.id = 3
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = 3.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.5
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 1.0
        marker.color.r, marker.color.g, marker.color.b = self.get_unique_color()
        marker.color.a = 1.0
        return marker

    def create_line_strip_marker(self):
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "cogniteam_sim"
        marker.id = 4
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color.r, marker.color.g, marker.color.b = self.get_unique_color()
        marker.color.a = 1.0

        # Triangle in the center
        p1 = Point(x=0.0, y=0.0, z=0.0)
        p2 = Point(x=1.0, y=0.0, z=0.0)
        p3 = Point(x=0.5, y=1.0, z=0.0)
        marker.points.extend([p1, p2, p3, p1])
        return marker

    def create_line_list_marker(self):
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "cogniteam_sim"
        marker.id = 5
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color.r, marker.color.g, marker.color.b = self.get_unique_color()
        marker.color.a = 1.0

        # Three lines, each 3 meters long
        marker.points.extend([
            Point(x=0.0, y=0.0, z=0.0), Point(x=3.0, y=0.0, z=0.0),
            Point(x=0.0, y=1.0, z=0.0), Point(x=3.0, y=1.0, z=0.0),
            Point(x=0.0, y=2.0, z=0.0), Point(x=3.0, y=2.0, z=0.0)
        ])
        return marker

    def create_text_marker(self):
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "cogniteam_sim"
        marker.id = 6
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 2.0
        marker.scale.z = 0.5
        marker.color.r, marker.color.g, marker.color.b = self.get_unique_color()
        marker.color.a = 1.0
        marker.text = "hello from cogniteam"
        return marker


    def publish_marker_array(self):
        
        marker_array = MarkerArray()       
       

        # Add the marker to the marker array
        marker_array.markers.append(self.create_arrow_marker())
        marker_array.markers.append(self.create_cube_marker())
        marker_array.markers.append(self.create_sphere_marker())
        marker_array.markers.append(self.create_cylinder_marker())
        marker_array.markers.append(self.create_line_strip_marker())
        marker_array.markers.append(self.create_line_list_marker())
        marker_array.markers.append(self.create_text_marker())





        # Publish the marker array
        self.marker_array_publisher.publish(marker_array)
    

    def publish_odometry(self):
        
        if self.robot_pose is not None:           
           
            self.robot_pose_publisher.publish(self.robot_pose)

    
    
            
    def create_laserscan_from_dict(self, distance_dict, angle_min=-math.pi, angle_max=math.pi, range_min=0.0, range_max=1000.0):
        # Initialize LaserScan message
        scan = LaserScan()
        scan.header.frame_id = self.global_frame
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = math.radians(1)  # 1 degree increments using math.radians
        scan.range_min = range_min
        scan.range_max = range_max

        # Prepare ranges and intensities
        num_readings = int((angle_max - angle_min) / scan.angle_increment) + 1
        scan.ranges = [float('inf')] * num_readings  # Initialize with inf
        scan.intensities = [0.0] * num_readings  # Assuming no intensities

        # Populate ranges based on the distance dictionary
        for angle, distance in distance_dict.items():
            if angle >= 0 and angle < 360:
                
                index = int((math.radians(angle) - angle_min) / scan.angle_increment)
                
                if distance == 0.0:
                    scan.ranges[index] = float('inf')  # Use float('inf') instead of string
                    
                if index >= num_readings:  # Changed from > 360 to >= num_readings for proper bounds checking
                    continue
                scan.ranges[index] = min(distance, range_max)  # Clamp to range_max

        return scan

    def tf_timer_callback(self):
        # Convert Pose to TransformStamped
        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.odom_frame 
        transform.child_frame_id = self.base_frame  

        # Set translation (position)
        transform.transform.translation.x = self.robot_pose.pose.position.x
        transform.transform.translation.y = self.robot_pose.pose.position.y
        transform.transform.translation.z = self.robot_pose.pose.position.z

        # Set rotation (orientation)
        transform.transform.rotation.x = self.robot_pose.pose.orientation.x
        transform.transform.rotation.y = self.robot_pose.pose.orientation.y
        transform.transform.rotation.z = self.robot_pose.pose.orientation.z
        transform.transform.rotation.w = self.robot_pose.pose.orientation.w

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)
        
        ################################################################
        
        odom_transform = TransformStamped()

        odom_transform.header.stamp = self.get_clock().now().to_msg()
        odom_transform.header.frame_id = self.global_frame 
        odom_transform.child_frame_id = self.odom_frame 

        # Set translation (position) from pose
        odom_transform.transform.translation.x = 0.0
        odom_transform.transform.translation.y = 0.0
        odom_transform.transform.translation.z = 0.0
        odom_transform.transform.rotation.w = 1.0

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(odom_transform)
        
        self.publish_battery_voltage()
        
        self.publish_date()
        
        # self.publish_compressed_image()
        
        self.publish_marker_array()

def main(args=None):
    rclpy.init(args=args)
    sim_node = SimNode()
    rclpy.spin(sim_node)
    robot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


