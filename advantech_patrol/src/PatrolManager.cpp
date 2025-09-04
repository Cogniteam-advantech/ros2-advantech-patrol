/*
 * PatrolManager.cpp
 *
 *  Created on: Nov 9, 2023
 *      Author: yakirhuri
 *  Converted to ROS2 Humble
 */

#include "PatrolManager.hpp"

// Nav2GoalManager Implementation
Nav2GoalManager::Nav2GoalManager() : Node("nav2_goal_manager"), goal_status_(GoalStatus::IDLE)
{
    // Create action client for navigate_to_pose
    navigate_client_ = rclcpp_action::create_client<NavigateToPose>(
        this, "navigate_to_pose");
    
    RCLCPP_INFO(this->get_logger(), "Nav2 Goal Manager initialized");
    
    // Wait for action server
    if (!navigate_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    }
}

bool Nav2GoalManager::sendGoal(double x, double y, double yaw, const std::string& frame_id)
{
    if (!navigate_client_->action_server_is_ready()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not ready");
        return false;
    }

    // Reset status
    goal_status_ = GoalStatus::PENDING;
    current_goal_handle_ = nullptr;

    // Create goal message
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.header.frame_id = frame_id;
    
    // Set position
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.position.z = 0.0;
    
    // Convert yaw to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    goal_msg.pose.pose.orientation = tf2::toMsg(q);

    // Set up goal options
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    
    send_goal_options.goal_response_callback =
        std::bind(&Nav2GoalManager::goalResponseCallback, this, std::placeholders::_1);
    
    send_goal_options.feedback_callback =
        std::bind(&Nav2GoalManager::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    
    send_goal_options.result_callback =
        std::bind(&Nav2GoalManager::resultCallback, this, std::placeholders::_1);

    RCLCPP_INFO(this->get_logger(), "Sending goal: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);
    
    // Send goal
    navigate_client_->async_send_goal(goal_msg, send_goal_options);
    
    return true;
}

bool Nav2GoalManager::abortGoal()
{
    if (!current_goal_handle_) {
        RCLCPP_WARN(this->get_logger(), "No active goal to abort");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Aborting current goal");
    
    auto cancel_future = navigate_client_->async_cancel_goal(current_goal_handle_);
    
    // Wait for cancel response
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), cancel_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto cancel_response = cancel_future.get();
        if (cancel_response->return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE) {
            RCLCPP_INFO(this->get_logger(), "Goal successfully cancelled");
            goal_status_ = GoalStatus::CANCELED;
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to cancel goal");
            return false;
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to cancel goal - timeout");
        return false;
    }
}

bool Nav2GoalManager::isGoalActive() const
{
    return goal_status_ == GoalStatus::ACTIVE || goal_status_ == GoalStatus::PENDING;
}

bool Nav2GoalManager::isGoalReached() const
{
    return goal_status_ == GoalStatus::SUCCEEDED;
}

bool Nav2GoalManager::isGoalFinished() const
{
    return goal_status_ == GoalStatus::SUCCEEDED || 
           goal_status_ == GoalStatus::ABORTED || 
           goal_status_ == GoalStatus::CANCELED;
}

Nav2GoalManager::GoalStatus Nav2GoalManager::getGoalStatus() const
{
    return goal_status_;
}

std::string Nav2GoalManager::getGoalStatusString() const
{
    switch (goal_status_) {
        case GoalStatus::IDLE: return "IDLE";
        case GoalStatus::PENDING: return "PENDING";
        case GoalStatus::ACTIVE: return "ACTIVE";
        case GoalStatus::SUCCEEDED: return "SUCCEEDED";
        case GoalStatus::ABORTED: return "ABORTED";
        case GoalStatus::CANCELED: return "CANCELED";
        default: return "UNKNOWN";
    }
}

bool Nav2GoalManager::waitForGoalCompletion(std::chrono::milliseconds timeout)
{
    auto start_time = std::chrono::steady_clock::now();
    
    while (rclcpp::ok() && !isGoalFinished()) {
        rclcpp::spin_some(this->get_node_base_interface());
        
        if (std::chrono::steady_clock::now() - start_time > timeout) {
            RCLCPP_WARN(this->get_logger(), "Timeout waiting for goal completion");
            return false;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    return isGoalReached();
}

double Nav2GoalManager::getRemainingDistance() const
{
    return distance_remaining_;
}

void Nav2GoalManager::goalResponseCallback(const GoalHandleNavigate::SharedPtr& goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        current_goal_handle_ = nullptr;
        goal_status_ = GoalStatus::ABORTED;
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        current_goal_handle_ = goal_handle;
        goal_status_ = GoalStatus::ACTIVE;
    }
}

void Nav2GoalManager::feedbackCallback(
    GoalHandleNavigate::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    auto current_pose = feedback->current_pose.pose;
    distance_remaining_ = feedback->distance_remaining;
    
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), 
        *this->get_clock(), 
        2000,  // Log every 2 seconds
        "Current position: x=%.2f, y=%.2f, distance remaining=%.2f",
        current_pose.position.x,
        current_pose.position.y,
        distance_remaining_
    );
}

void Nav2GoalManager::resultCallback(const GoalHandleNavigate::WrappedResult& result)
{
    current_goal_handle_ = nullptr;
    
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Navigation succeeded!");
            goal_status_ = GoalStatus::SUCCEEDED;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Navigation was aborted");
            goal_status_ = GoalStatus::ABORTED;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(this->get_logger(), "Navigation was canceled");
            goal_status_ = GoalStatus::CANCELED;
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            goal_status_ = GoalStatus::ABORTED;
            break;
    }
}

// PatrolManager Implementation
PatrolManager::PatrolManager() : Node("platril_manager")
{
    // Initialize TF2
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Declare and get parameters
    this->declare_parameter("robot_state", "IDLE");
    this->get_parameter("robot_state", robot_state_);
    robot_state_ = "IDLE";
    
    // Initialize publishers
    robot_history_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/robot_history_path", 1);
    
    goals_marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/waypoints_markers", 10);
    
    person_alarm_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/person_alarm", 10);

    // Create subscriber for person_width_exceeded topic
    person_width_exceeded_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/person_width_exceeded", 
        10,
        std::bind(&PatrolManager::person_width_callback, this, std::placeholders::_1)
    );

    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 1);
    
    // Initialize subscribers
    start_patrol_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/start_patrol", 1,
        std::bind(&PatrolManager::startCallback, this, std::placeholders::_1));
    
    // Initialize service client
    clear_costmaps_client_ = this->create_client<std_srvs::srv::Empty>(
        "/global_costmap/clear_entirely_global_costmap");
}

PatrolManager::~PatrolManager()
{
}

void PatrolManager::person_width_callback(const std_msgs::msg::Bool::SharedPtr msg)
{   
    person_alarm_ = msg->data;
}

void PatrolManager::run()
{
    auto nav_manager = std::make_shared<Nav2GoalManager>();

    // Declare and get waypoint parameters
    this->declare_parameter("waypoint1.x", -3.0);
    this->declare_parameter("waypoint1.y", 0.0);
    this->declare_parameter("waypoint1.rad", 0.0);
    
    this->declare_parameter("waypoint2.x", -3.0);
    this->declare_parameter("waypoint2.y", 3.0);
    this->declare_parameter("waypoint2.rad", 3.14);

    // Get waypoint parameters
    double wp1_x, wp1_y, wp1_rad;
    double wp2_x, wp2_y, wp2_rad;
    
    this->get_parameter("waypoint1.x", wp1_x);
    this->get_parameter("waypoint1.y", wp1_y);
    this->get_parameter("waypoint1.rad", wp1_rad);
    
    this->get_parameter("waypoint2.x", wp2_x);
    this->get_parameter("waypoint2.y", wp2_y);
    this->get_parameter("waypoint2.rad", wp2_rad);

    // Create waypoints with parameters
    WayPoint p1;
    p1.x = wp1_x;
    p1.y = wp1_y;
    p1.rad = wp1_rad;

    WayPoint p2;
    p2.x = wp2_x;
    p2.y = wp2_y;
    p2.rad = wp2_rad;

    waypoints_.push_back(p1);
    waypoints_.push_back(p2);
    
    RCLCPP_INFO(this->get_logger(), "Waypoint 1: x=%.2f, y=%.2f, rad=%.2f", wp1_x, wp1_y, wp1_rad);
    RCLCPP_INFO(this->get_logger(), "Waypoint 2: x=%.2f, y=%.2f, rad=%.2f", wp2_x, wp2_y, wp2_rad);
    
    //verify amcl works and the robot have location
    bool recv_map_odom = false;
    cerr << " waiting for robot's location ... " << endl;

    while(rclcpp::ok() && !recv_map_odom) {	
        rclcpp::spin_some(shared_from_this());
        recv_map_odom = checkLocalizationOk();
    }
    
    cerr << " locatoion is good " << endl;
    
    while (rclcpp::ok()) {
        rclcpp::spin_some(shared_from_this());

        if (start_patrol_cmd_) {
            break;
        }

        updateRobotLocation();
        publishPersonAlarm();				
    }
    
    while (rclcpp::ok())
    {   
        rclcpp::spin_some(nav_manager);
        rclcpp::spin_some(shared_from_this());

        cerr << " person_alarm_ " << person_alarm_ << endl;

        publishWaypointsWithStatus();

        for (int i = 0; i < waypoints_.size(); i++) {
            rclcpp::spin_some(nav_manager);
            rclcpp::spin_some(shared_from_this());

            publishPersonAlarm();			                

            if (person_alarm_) {
                // The robot waits for the person to pass
                while(rclcpp::ok()) {
                    rclcpp::spin_some(shared_from_this());
                    publishPersonAlarm();

                    if (person_alarm_ == false) {													
                        break;
                    } 
                }
            }				
            
            //clearAllCostMaps();                

            ///////////
            nav_manager->sendGoal(waypoints_[i].x, waypoints_[i].y, waypoints_[i].rad);
            
            while (rclcpp::ok() && nav_manager->isGoalActive()) {
                rclcpp::spin_some(nav_manager);
                rclcpp::spin_some(shared_from_this());

                updateRobotLocation();
                publishWaypointsWithStatus();
                publishPersonAlarm();

                if (person_alarm_){
                    nav_manager->abortGoal();

                    while(rclcpp::ok()) {                            
                        rclcpp::spin_some(shared_from_this());
                        rclcpp::spin_some(nav_manager);

                        publishPersonAlarm();

                        if (person_alarm_ == false) {                                   
                            //clearAllCostMaps();

                            // The robot continues the navigation to the same point it canceled
                            nav_manager->sendGoal(waypoints_[i].x, waypoints_[i].y, waypoints_[i].rad);
                            break;
                        } 
                    }
                }                       

                if (nav_manager->isGoalReached()) {
                    std::cout << "Success!" << std::endl;
                    break;
                }    
            }                
        }

        cerr << "FINSIHED PATROL!!!!! " << endl;
    }
}

bool PatrolManager::checkLocalizationOk() 
{
    try {
        geometry_msgs::msg::TransformStamped transform;
        transform = tf_buffer_->lookupTransform(
            global_frame_, odom_frame_, tf2::TimePointZero);
        return true;
    }
    catch (tf2::TransformException& ex) {
        return false;
    }
}

bool PatrolManager::updateRobotLocation() 
{
    try {
        geometry_msgs::msg::TransformStamped transform;
        transform = tf_buffer_->lookupTransform(
            global_frame_, base_frame_, tf2::TimePointZero);
        
        robot_pose_.header.frame_id = global_frame_;
        robot_pose_.header.stamp = this->get_clock()->now();
        robot_pose_.pose.position.x = transform.transform.translation.x;
        robot_pose_.pose.position.y = transform.transform.translation.y;
        robot_pose_.pose.position.z = 0;
        robot_pose_.pose.orientation = transform.transform.rotation;
        
        return true;
    }
    catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Error between %s to %s: %s", 
                    global_frame_.c_str(), base_frame_.c_str(), ex.what());
        return false;
    }
}

void PatrolManager::publishPersonAlarm() 
{
    visualization_msgs::msg::Marker line_strip;
    line_strip.header.frame_id = base_frame_;
    line_strip.header.stamp = this->get_clock()->now();
    line_strip.ns = "points_and_lines";
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 6000;
    line_strip.lifetime = rclcpp::Duration::from_seconds(1.0);
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    
    if (person_alarm_) {
        line_strip.scale.x = 0.1;
        line_strip.color.b = 0.0;
        line_strip.color.g = 0.0;
        line_strip.color.r = 1.0;
        line_strip.color.a = 1.0;
        line_strip.text = "PERSON_DETECTED";
    } else {
        line_strip.scale.x = 0.1;
        line_strip.color.b = 0.5;
        line_strip.color.g = 1.0;
        line_strip.color.r = 0.0;
        line_strip.color.a = 1.0;
        line_strip.text = "NO_PERSON";
    }
    
    for (uint32_t i = 0; i < 360; ++i) {
        geometry_msgs::msg::Point p;
        p.y = (0.5) * sin(angles::from_degrees(i));
        p.x = (0.5) * cos(angles::from_degrees(i));
        p.z = 0.5;
        line_strip.points.push_back(p);
    }
    
    person_alarm_marker_pub_->publish(line_strip);
}

void PatrolManager::publishWaypointsWithStatus() 
{
    // This method was commented out in the original code
    // Keeping it as a placeholder for future implementation
}

void PatrolManager::startCallback(const std_msgs::msg::Bool::SharedPtr msg) 
{
    if (msg->data == true) {
        start_patrol_cmd_ = true;
        RCLCPP_INFO(this->get_logger(), "Patrol start command received");
    }
}