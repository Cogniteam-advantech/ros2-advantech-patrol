// /*
//  * PatrolManager.hpp
//  *
//  *  Created on: Nov 9, 2023
//  *      Author: yakirhuri
//  *  Converted to ROS2 Humble
//  */

// #ifndef INCLUDE_PATROL_MANAGER_HPP
// #define INCLUDE_PATROL_MANAGER_HPP

#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <angles/angles.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_srvs/srv/empty.hpp>


#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace std::chrono_literals;   






#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


struct WayPoint {
    float x = 0.0;
    float y = 0.0;
    float rad = 0.0;
    bool status_ = false;
};



class Nav2GoalManager : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    enum class GoalStatus {
        IDLE,           // No goal active
        PENDING,        // Goal sent, waiting for acceptance
        ACTIVE,         // Goal accepted and executing
        SUCCEEDED,      // Goal completed successfully
        ABORTED,        // Goal aborted due to error
        CANCELED        // Goal was canceled
    };

    Nav2GoalManager() : Node("nav2_goal_manager"), goal_status_(GoalStatus::IDLE)
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

    bool sendGoal(double x, double y, double yaw, const std::string& frame_id = "map")
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

    bool abortGoal()
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

    // Check if goal is currently active (being executed)
    bool isGoalActive() const
    {
        return goal_status_ == GoalStatus::ACTIVE || goal_status_ == GoalStatus::PENDING;
    }

    // Check if goal has been reached successfully
    bool isGoalReached() const
    {
        return goal_status_ == GoalStatus::SUCCEEDED;
    }

    // Check if goal is finished (regardless of success/failure)
    bool isGoalFinished() const
    {
        return goal_status_ == GoalStatus::SUCCEEDED || 
               goal_status_ == GoalStatus::ABORTED || 
               goal_status_ == GoalStatus::CANCELED;
    }

    // Get current goal status
    GoalStatus getGoalStatus() const
    {
        return goal_status_;
    }

    // Get status as string for logging/debugging
    std::string getGoalStatusString() const
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

    // Wait for goal to finish with timeout
    bool waitForGoalCompletion(std::chrono::milliseconds timeout = std::chrono::milliseconds(30000))
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

    // Get remaining distance to goal (from last feedback)
    double getRemainingDistance() const
    {
        return distance_remaining_;
    }

private:
    void goalResponseCallback(const GoalHandleNavigate::SharedPtr& goal_handle)
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

    void feedbackCallback(
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

    void resultCallback(const GoalHandleNavigate::WrappedResult& result)
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

    rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_client_;
    GoalHandleNavigate::SharedPtr current_goal_handle_;
    GoalStatus goal_status_;
    double distance_remaining_ = 0.0;
};



class PatrolManager:  public rclcpp::Node {


public:

    PatrolManager():  Node("platril_manager"){

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

    ~PatrolManager(){

    }
   

    void person_width_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {   

        person_alarm_ = msg->data;
        
    }

    
    void run(){


        auto nav_manager = std::make_shared<Nav2GoalManager>();

        WayPoint p1;
        p1.x = -3.0;
        p1.rad = 0.0;

        WayPoint p2;
        p2.x =  -3.0;
        p2.y =   3.0;
        p2.rad = 3.14;


        waypoints_.push_back(p1);
        waypoints_.push_back(p2);
        //verify amcl works and the robot have location
        bool recv_map_odom = false;
        cerr<<" waiting for robot's location ... "<<endl;

        while( rclcpp::ok() && !recv_map_odom ) {	
            
            // cerr<<" waiting for robot's location ... "<<endl;
            rclcpp::spin_some(shared_from_this());

            recv_map_odom =  checkLocalizationOk();
            
        }
        cerr<<" locatoion is good "<<endl;
        while (rclcpp::ok()){

            rclcpp::spin_some(shared_from_this());

            if ( start_patrol_cmd_){
                break;
            }

            updateRobotLocation();


            publishPersonAlarm();				

        }
        
        while (rclcpp::ok())
        {   
            rclcpp::spin_some(nav_manager);

            rclcpp::spin_some(shared_from_this());

            cerr<<" person_alarm_ "<<person_alarm_<<endl;

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

            cerr<<"FINSIHED PATROL!!!!! "<<endl;
        }



    }


private:

    bool checkLocalizationOk() {
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


    bool updateRobotLocation() {
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


    void publishPersonAlarm() {
        
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

    void publishWaypointsWithStatus() {
        
        // visualization_msgs::msg::MarkerArray markers;
        
        // for (size_t i = 0; i < waypoints_.size(); i++) {
        //     visualization_msgs::msg::Marker marker;
        //     marker.lifetime = rclcpp::Duration::from_seconds(100.0);
        //     marker.action = visualization_msgs::msg::Marker::ADD;
        //     marker.type = visualization_msgs::msg::Marker::SPHERE;
        //     marker.header.frame_id = "map";
        //     marker.header.stamp = this->get_clock()->now();
        //     marker.id = i;
        //     marker.pose.position = waypoints_[i].w_pose_.pose.position;
        //     // marker.pose.orientation = waypoints_[i].w_pose_.pose.orientation;
        //     marker.scale.x = 0.2;
        //     marker.scale.y = 0.2;
        //     marker.scale.z = 0.2;
            
        //     if (waypoints_[i].status_) {
        //         marker.color.r = 0.0f;
        //         marker.color.g = 1.0f;
        //         marker.color.b = 0.0f;
        //         marker.color.a = 1.0;
        //     } else {
        //         marker.color.r = 1.0f;
        //         marker.color.g = 0.0f;
        //         marker.color.b = 0.0f;
        //         marker.color.a = 1.0;
        //     }
            
        //     markers.markers.push_back(marker);
        // }
        
        // goals_marker_array_publisher_->publish(markers);
    }

    


    void startCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data == true) {
            start_patrol_cmd_ = true;
            RCLCPP_INFO(this->get_logger(), "Patrol start command received");
        }
    }


private:
    // Core ROS2 components
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    
    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr robot_history_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr goals_marker_array_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr person_alarm_marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr person_width_exceeded_sub_;

    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_patrol_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr objects_string_sub_;
    
    // Service clients
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_costmaps_client_;
    
    // Timer

    
    // Robot state and pose
    geometry_msgs::msg::PoseStamped robot_pose_;
    string robot_state_ = "IDLE";
    
    // Frame IDs
    string global_frame_ = "map";
    string base_frame_ = "base_link";
    string odom_frame_ = "odom";
    
    // Camera parameters
    float image_w_ = 640;
    float camera_fov_ = 80.0;
    float image_c_x_ = 640 / 2.0;
    double b_box_ratio_ = 0.25;
    
    // Control flags
    bool start_patrol_cmd_ = false;
    bool person_alarm_ = false;
    std::chrono::steady_clock::time_point last_time_detected_;
    
    // Path and waypoints
    nav_msgs::msg::Path robot_history_path_msg_;
    vector<WayPoint> waypoints_;
    size_t current_waypoint_index_ = 0;
    bool patrol_active_ = false;



};