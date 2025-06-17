// /*
//  * PatrolManager.cpp
//  *
//  *  Created on: Nov 9, 2023
//  *      Author: yakirhuri
//  *  Converted to ROS2 Humble
//  */

// #include "PatrolManager.hpp"
// #include <angles/angles.h>

// PatrolManager::PatrolManager(vector<WayPoint> waypoints) 
//     : Node("patrol_manager"), waypoints_(waypoints) {
    
//     // Initialize TF2
//     tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
//     tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
//     // Declare and get parameters
//     this->declare_parameter("robot_state", "IDLE");
//     this->declare_parameter("b_box_ratio", 0.2);
    
//     this->get_parameter("robot_state", robot_state_);
//     this->get_parameter("b_box_ratio", b_box_ratio_);
    
//     robot_state_ = "IDLE";
    
//     // Initialize publishers
//     robot_history_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
//         "/robot_history_path", 1);
    
//     goals_marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
//         "/waypoints_markers", 10);
    
//     person_alarm_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
//         "/person_alarm", 10);
    
//     twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
//         "/cmd_vel", 1);
    
//     // Initialize subscribers
//     start_patrol_sub_ = this->create_subscription<std_msgs::msg::Bool>(
//         "/start_patrol", 1,
//         std::bind(&PatrolManager::startCallback, this, std::placeholders::_1));
    
//     objects_string_sub_ = this->create_subscription<std_msgs::msg::String>(
//         "/objects_string_detections", 1,
//         std::bind(&PatrolManager::objectsCallback, this, std::placeholders::_1));
    
//     // Initialize service client
//     clear_costmaps_client_ = this->create_client<std_srvs::srv::Empty>(
//         "/global_costmap/clear_entirely_global_costmap");
    
//     // Initialize move base controller
//     // move_base_controller_ = std::make_shared<MoveBaseController>(shared_from_this());
    
//     // Initialize timer for main loop
//     main_timer_ = this->create_wall_timer(
//         100ms, std::bind(&PatrolManager::mainLoop, this));
    
//     // Wait for move base controller
//     // RCLCPP_INFO(this->get_logger(), "Waiting for navigation action server...");
//     // if (!move_base_controller_->waitForServer()) {
//     //     RCLCPP_ERROR(this->get_logger(), "Failed to connect to navigation server!");
//     // } else {
//     //     RCLCPP_INFO(this->get_logger(), "Connected to navigation server!");
//     // }
    
//     last_time_detected_ = std::chrono::steady_clock::now();
// }

// PatrolManager::~PatrolManager() {
// }

// void PatrolManager::mySigintHandler(int sig) {
//     RCLCPP_INFO(rclcpp::get_logger("patrol_manager"), "User pressed CTRL+C");
//     rclcpp::shutdown();
// }

// std::vector<ObjectDnn> PatrolManager::parseObjects(const std::string& input) {
//     std::vector<ObjectDnn> objects;
    
//     // Tokenize the string using ';' as a delimiter
//     std::istringstream tokenStream(input);
//     std::string token;
    
//     while (std::getline(tokenStream, token, ';')) {
//         // Tokenize each element using ',' as a delimiter
//         std::istringstream elementStream(token);
//         std::string element;
        
//         ObjectDnn obj;
        
//         for (int i = 0; i < 5; ++i) {
//             if (std::getline(elementStream, element, ',')) {
//                 // Convert the string to int and store in the struct
//                 switch (i) {
//                     case 0:
//                         obj.id = std::atoi(element.c_str());
//                         break;
//                     case 1:
//                         obj.cx = std::atoi(element.c_str());
//                         break;
//                     case 2:
//                         obj.cy = std::atoi(element.c_str());
//                         break;
//                     case 3:
//                         obj.width = std::atoi(element.c_str());
//                         break;
//                     case 4:
//                         obj.height = std::atoi(element.c_str());
//                         break;
//                     default:
//                         break;
//                 }
//             }
//         }
//         objects.push_back(obj);
//     }
    
//     return objects;
// }

// void PatrolManager::setState(RobotState state) {
//     switch (state) {
//         case IDLE: {
//             this->set_parameter(rclcpp::Parameter("robot_state", "IDLE"));
//             robot_state_ = "IDLE";
//             return;
//         }
//         case CHARGING: {
//             this->set_parameter(rclcpp::Parameter("robot_state", "CHARGING"));
//             robot_state_ = "CHARGING";
//             return;
//         }
//         case PATROL: {
//             this->set_parameter(rclcpp::Parameter("robot_state", "PATROL"));
//             robot_state_ = "PATROL";
//             return;
//         }
//         case NAV_TO_DOCKING_STATION: {
//             this->set_parameter(rclcpp::Parameter("robot_state", "NAV_TO_DOCKING_STATION"));
//             robot_state_ = "NAV_TO_DOCKING_STATION";
//             return;
//         }
//         case ERROR: {
//             this->set_parameter(rclcpp::Parameter("robot_state", "ERROR"));
//             robot_state_ = "ERROR";
//             return;
//         }
//     }
// }

// bool PatrolManager::checkLocalizationOk() {
//     try {
//         geometry_msgs::msg::TransformStamped transform;
//         transform = tf_buffer_->lookupTransform(
//             global_frame_, odom_frame_, tf2::TimePointZero);
//         return true;
//     }
//     catch (tf2::TransformException& ex) {
//         return false;
//     }
// }

// double PatrolManager::distanceCalculate(cv::Point2d p1, cv::Point2d p2) {
//     double x = p1.x - p2.x;
//     double y = p1.y - p2.y;
//     double dist = sqrt(pow(x, 2) + pow(y, 2));
//     return dist;
// }

// double PatrolManager::pixelToAbsoluteDegree(int pixel, int imageWidth, double cameraFOV) {
//     double pixelFOV = cameraFOV / imageWidth;
//     double absoluteDegree = (pixel - imageWidth / 2) * pixelFOV;
//     return absoluteDegree;
// }

// void PatrolManager::publishRobotHistoryPath() {
//     robot_history_path_msg_.header.stamp = this->get_clock()->now();
//     robot_history_path_msg_.header.frame_id = global_frame_;
//     robot_history_path_pub_->publish(robot_history_path_msg_);
// }

// void PatrolManager::clearAllCostMaps() {
//     RCLCPP_INFO(this->get_logger(), "Clearing costmaps...");
    
//     if (!clear_costmaps_client_->wait_for_service(1s)) {
//         RCLCPP_WARN(this->get_logger(), "Clear costmaps service not available");
//         return;
//     }
    
//     auto request = std::make_shared<std_srvs::srv::Empty::Request>();
//     auto future = clear_costmaps_client_->async_send_request(request);
    
//     if (rclcpp::spin_until_future_complete(shared_from_this(), future, 2s) == 
//         rclcpp::FutureReturnCode::SUCCESS) {
//         RCLCPP_INFO(this->get_logger(), "Costmaps cleared successfully");
//     } else {
//         RCLCPP_ERROR(this->get_logger(), "Failed to clear costmaps");
//     }
// }

// void PatrolManager::run() {
//     // Wait for localization
//     bool recv_map_odom = false;
//     RCLCPP_INFO(this->get_logger(), "Waiting for robot's location...");
    
//     while (rclcpp::ok() && !recv_map_odom) {
//         rclcpp::spin_some(shared_from_this());
//         recv_map_odom = checkLocalizationOk();
//         std::this_thread::sleep_for(100ms);
//     }
    
//     RCLCPP_INFO(this->get_logger(), "Robot localization OK, ready to patrol");
    
//     // Wait for start command
//     while (rclcpp::ok() && !start_patrol_cmd_) {
//         rclcpp::spin_some(shared_from_this());
//         updateRobotLocation();
//         publishPersonAlarm();
//         std::this_thread::sleep_for(100ms);
//     }
    
//     RCLCPP_INFO(this->get_logger(), "Starting patrol...");
//     patrol_active_ = true;
//     setState(PATROL);
// }

// void PatrolManager::mainLoop() {
//     if (!patrol_active_) {
//         return;
//     }
    
//     updateRobotLocation();
//     publishWaypointsWithStatus();
//     publishPersonAlarm();
    
//     // Main patrol logic
//     if (current_waypoint_index_ < waypoints_.size()) {
//         RCLCPP_INFO(this->get_logger(), "Starting waypoint %zu", current_waypoint_index_);
        
//         // Check for person alarm before moving
//         if (person_alarm_) {
//             RCLCPP_WARN(this->get_logger(), "Person detected, waiting...");
//             return;
//         }
        
//         // // Navigate to waypoint
//         // if (!move_base_controller_->isNavigating()) {
//         //     clearAllCostMaps();
//         //     move_base_controller_->navigate(waypoints_[current_waypoint_index_].w_pose_);
//         // }
        
//         // Check if person detected during navigation
//         // if (person_alarm_ && move_base_controller_->isNavigating()) {
//         //     RCLCPP_WARN(this->get_logger(), "Person detected during navigation, canceling...");
//         //     // move_base_controller_->cancelNavigation();
//         //     return;
//         // }
        
//         // Check if navigation completed
//         if (/*!move_base_controller_->isNavigating()*/false) {
//             // auto result = move_base_controller_->getNavigationResult();
//             // if (result == rclcpp_action::ResultCode::SUCCEEDED) {
//             //     RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu", current_waypoint_index_);
//             //     waypoints_[current_waypoint_index_].status_ = true;
//             //     current_waypoint_index_++;
//             // } else {
//             //     RCLCPP_WARN(this->get_logger(), "Failed to reach waypoint %zu", current_waypoint_index_);
//             //     current_waypoint_index_++;
//             // }
//         }
//     } else {
//         RCLCPP_INFO(this->get_logger(), "FINISHED PATROL!");
//         patrol_active_ = false;
//         current_waypoint_index_ = 0;
//         setState(IDLE);
        
//         // Reset waypoint status
//         for (auto& waypoint : waypoints_) {
//             waypoint.status_ = false;
//         }
//     }
// }

// void PatrolManager::startCallback(const std_msgs::msg::Bool::SharedPtr msg) {
//     if (msg->data == true) {
//         start_patrol_cmd_ = true;
//         RCLCPP_INFO(this->get_logger(), "Patrol start command received");
//     }
// }

// void PatrolManager::objectsCallback(const std_msgs::msg::String::SharedPtr msg) {
//     if (msg->data == "") {
//         auto now = std::chrono::steady_clock::now();
//         auto duration_without_person = std::chrono::duration_cast<std::chrono::seconds>(
//             now - last_time_detected_).count();
        
//         if (duration_without_person < DURATION_WITHOUT_PERSONS_THRESHOLD) {
//             person_alarm_ = true;
//         } else {
//             person_alarm_ = false;
//         }
//         return;
//     }
    
//     // Parse the input string and get the vector of objects
//     std::vector<ObjectDnn> objectVector = parseObjects(msg->data);
    
//     // Display the parsed objects
//     for (const auto& obj : objectVector) {
//         // not a person
//         if (obj.id != 1) {
//             continue;
//         }
        
//         float ratio = image_w_ * (b_box_ratio_);
//         double absoluteDegree = pixelToAbsoluteDegree(obj.cx, image_w_, camera_fov_);
        
//         if (obj.width >= ratio) {
//             person_alarm_ = true;
//             last_time_detected_ = std::chrono::steady_clock::now();
//             return;
//         }
//     }
    
//     auto now = std::chrono::steady_clock::now();
//     auto duration_without_person = std::chrono::duration_cast<std::chrono::seconds>(
//         now - last_time_detected_).count();
    
//     if (duration_without_person < DURATION_WITHOUT_PERSONS_THRESHOLD) {
//         person_alarm_ = true;
//     } else {
//         person_alarm_ = false;
//     }
// }

// void PatrolManager::publishPersonAlarm() {
//     visualization_msgs::msg::Marker line_strip;
//     line_strip.header.frame_id = base_frame_;
//     line_strip.header.stamp = this->get_clock()->now();
//     line_strip.ns = "points_and_lines";
//     line_strip.pose.orientation.w = 1.0;
//     line_strip.id = 6000;
//     line_strip.lifetime = rclcpp::Duration::from_seconds(1.0);
//     line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    
//     if (person_alarm_) {
//         line_strip.scale.x = 0.1;
//         line_strip.color.b = 0.0;
//         line_strip.color.g = 0.0;
//         line_strip.color.r = 1.0;
//         line_strip.color.a = 1.0;
//         line_strip.text = "PERSON_DETECTED";
//     } else {
//         line_strip.scale.x = 0.1;
//         line_strip.color.b = 0.5;
//         line_strip.color.g = 1.0;
//         line_strip.color.r = 0.0;
//         line_strip.color.a = 1.0;
//         line_strip.text = "NO_PERSON";
//     }
    
//     for (uint32_t i = 0; i < 360; ++i) {
//         geometry_msgs::msg::Point p;
//         p.y = (0.5) * sin(angles::from_degrees(i));
//         p.x = (0.5) * cos(angles::from_degrees(i));
//         p.z = 0.5;
//         line_strip.points.push_back(p);
//     }
    
//     person_alarm_marker_pub_->publish(line_strip);
// }

// void PatrolManager::publishWaypointsWithStatus() {
//     visualization_msgs::msg::MarkerArray markers;
    
//     for (size_t i = 0; i < waypoints_.size(); i++) {
//         visualization_msgs::msg::Marker marker;
//         marker.lifetime = rclcpp::Duration::from_seconds(100.0);
//         marker.action = visualization_msgs::msg::Marker::ADD;
//         marker.type = visualization_msgs::msg::Marker::SPHERE;
//         marker.header.frame_id = "map";
//         marker.header.stamp = this->get_clock()->now();
//         marker.id = i;
//         marker.pose.position = waypoints_[i].w_pose_.pose.position;
//         marker.pose.orientation = waypoints_[i].w_pose_.pose.orientation;
//         marker.scale.x = 0.2;
//         marker.scale.y = 0.2;
//         marker.scale.z = 0.2;
        
//         if (waypoints_[i].status_) {
//             marker.color.r = 0.0f;
//             marker.color.g = 1.0f;
//             marker.color.b = 0.0f;
//             marker.color.a = 1.0;
//         } else {
//             marker.color.r = 1.0f;
//             marker.color.g = 0.0f;
//             marker.color.b = 0.0f;
//             marker.color.a = 1.0;
//         }
        
//         markers.markers.push_back(marker);
//     }
    
//     goals_marker_array_publisher_->publish(markers);
// }

// bool PatrolManager::updateRobotLocation() {
//     try {
//         geometry_msgs::msg::TransformStamped transform;
//         transform = tf_buffer_->lookupTransform(
//             global_frame_, base_frame_, tf2::TimePointZero);
        
//         robot_pose_.header.frame_id = global_frame_;
//         robot_pose_.header.stamp = this->get_clock()->now();
//         robot_pose_.pose.position.x = transform.transform.translation.x;
//         robot_pose_.pose.position.y = transform.transform.translation.y;
//         robot_pose_.pose.position.z = 0;
//         robot_pose_.pose.orientation = transform.transform.rotation;
        
//         return true;
//     }
//     catch (tf2::TransformException& ex) {
//         RCLCPP_ERROR(this->get_logger(), "Error between %s to %s: %s", 
//                     global_frame_.c_str(), base_frame_.c_str(), ex.what());
//         return false;
//     }
// }
