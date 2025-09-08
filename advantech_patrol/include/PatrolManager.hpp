/*
 * PatrolManager.hpp
 *
 *  Created on: Nov 9, 2023
 *      Author: yakirhuri
 *  Converted to ROS2 Humble
 */

#ifndef INCLUDE_PATROL_MANAGER_HPP
#define INCLUDE_PATROL_MANAGER_HPP

#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <angles/angles.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
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

    Nav2GoalManager();
    ~Nav2GoalManager() = default;

    bool sendGoal(double x, double y, double yaw, const std::string& frame_id = "map");
    bool abortGoal();
    
    // Status checking methods
    bool isGoalActive() const;
    bool isGoalReached() const;
    bool isGoalFinished() const;
    GoalStatus getGoalStatus() const;
    std::string getGoalStatusString() const;
    
    // Utility methods
    bool waitForGoalCompletion(std::chrono::milliseconds timeout = std::chrono::milliseconds(30000));
    double getRemainingDistance() const;

private:
    // Callback methods
    void goalResponseCallback(const GoalHandleNavigate::SharedPtr& goal_handle);
    void feedbackCallback(GoalHandleNavigate::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void resultCallback(const GoalHandleNavigate::WrappedResult& result);

    // Private members
    rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_client_;
    GoalHandleNavigate::SharedPtr current_goal_handle_;
    GoalStatus goal_status_;
    double distance_remaining_ = 0.0;
};

class PatrolManager : public rclcpp::Node 
{
public:
    PatrolManager();
    ~PatrolManager();
    
    void run();

private:
    // Callback methods
    void person_width_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void startCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void stopPatrolCallback(const std_msgs::msg::Empty::SharedPtr msg); 

    
    // Utility methods
    bool checkLocalizationOk();
    bool updateRobotLocation();
    void publishPersonAlarm();
    void publishWaypointsWithStatus();

    // Core ROS2 components
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    
    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr robot_history_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr goals_marker_array_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr person_alarm_marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr person_width_exceeded_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_patrol_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_patrol_sub_;    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr objects_string_sub_;
    
    // Service clients
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_costmaps_client_;
    
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
    bool stop_cmd_ = false;
    
    // Path and waypoints
    nav_msgs::msg::Path robot_history_path_msg_;
    vector<WayPoint> waypoints_;
    size_t current_waypoint_index_ = 0;
    bool patrol_active_ = false;
};

#endif // INCLUDE_PATROL_MANAGER_HPP