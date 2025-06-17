/*
 * MoveBaseController.cpp
 *
 *  Created on: Mar 7, 2016
 *      Author: blackpc
 *  Converted to ROS2 Humble
 */

#include "MoveBaseController.hpp"

using namespace std::placeholders;

MoveBaseController::MoveBaseController(rclcpp::Node::SharedPtr node) 
    : node_(node), navigation_active_(false), last_result_code_(rclcpp_action::ResultCode::UNKNOWN) {
    
    action_client_ = rclcpp_action::create_client<NavigateToPose>(
        node_, "navigate_to_pose");
}

MoveBaseController::~MoveBaseController() {
    if (navigation_active_) {
        cancelNavigation();
    }
}

void MoveBaseController::navigate(const geometry_msgs::msg::PoseStamped& goal) {
    if (!action_client_->action_server_is_ready()) {
        RCLCPP_ERROR(node_->get_logger(), "Action server not available");
        return;
    }
    
    auto goal_msg = createGoalMessage(goal);
    
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&MoveBaseController::goalResponseCallback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&MoveBaseController::feedbackCallback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&MoveBaseController::resultCallback, this, _1);
    
    std::lock_guard<std::mutex> lock(navigation_mutex_);
    navigation_active_ = true;
    last_result_code_ = rclcpp_action::ResultCode::UNKNOWN;
    
    auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);
    
    RCLCPP_DEBUG(node_->get_logger(), "Navigation goal sent");
}

NavigateToPose::Goal MoveBaseController::createGoalMessage(
        const geometry_msgs::msg::PoseStamped& goal) const {
    NavigateToPose::Goal goal_msg;
    goal_msg.pose = goal;
    goal_msg.pose.header.stamp = node_->get_clock()->now();
    return goal_msg;
}

void MoveBaseController::goalResponseCallback(const GoalHandleNavigateToPose::SharedPtr& goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
        std::lock_guard<std::mutex> lock(navigation_mutex_);
        navigation_active_ = false;
        last_result_code_ = rclcpp_action::ResultCode::REJECTED;
    } else {
        RCLCPP_DEBUG(node_->get_logger(), "Goal accepted by server, waiting for result");
        std::lock_guard<std::mutex> lock(navigation_mutex_);
        goal_handle_ = goal_handle;
    }
}

void MoveBaseController::feedbackCallback(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
    RCLCPP_DEBUG(node_->get_logger(), "Navigation feedback received");
    // You can process feedback here if needed
    // For example: current pose, distance remaining, etc.
}

void MoveBaseController::resultCallback(const GoalHandleNavigateToPose::WrappedResult& result) {
    std::lock_guard<std::mutex> lock(navigation_mutex_);
    navigation_active_ = false;
    last_result_code_ = result.code;
    goal_handle_.reset();
    
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_DEBUG(node_->get_logger(), "Navigation succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(node_->get_logger(), "Navigation was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(node_->get_logger(), "Navigation was canceled");
            break;
        default:
            RCLCPP_ERROR(node_->get_logger(), "Unknown navigation result");
            break;
    }
}

bool MoveBaseController::wait() {
    RCLCPP_DEBUG(node_->get_logger(), "Waiting for navigation result...");
    
    while (rclcpp::ok() && navigation_active_) {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    RCLCPP_DEBUG(node_->get_logger(), "Finished waiting");
    return last_result_code_ == rclcpp_action::ResultCode::SUCCEEDED;
}

bool MoveBaseController::waitForServer(const std::chrono::seconds& timeout) {
    return action_client_->wait_for_action_server(timeout);
}

void MoveBaseController::cancelNavigation() {
    std::lock_guard<std::mutex> lock(navigation_mutex_);
    
    if (navigation_active_ && goal_handle_) {
        RCLCPP_INFO(node_->get_logger(), "Canceling navigation");
        auto cancel_future = action_client_->async_cancel_goal(goal_handle_);
        
        // Wait for cancellation to complete
        if (rclcpp::spin_until_future_complete(node_, cancel_future, std::chrono::seconds(1)) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_DEBUG(node_->get_logger(), "Navigation canceled successfully");
        } else {
            RCLCPP_WARN(node_->get_logger(), "Failed to cancel navigation in time");
        }
        
        navigation_active_ = false;
        goal_handle_.reset();
        last_result_code_ = rclcpp_action::ResultCode::CANCELED;
    }
}

bool MoveBaseController::isNavigating() const {
    std::lock_guard<std::mutex> lock(navigation_mutex_);
    return navigation_active_;
}

rclcpp_action::ResultCode MoveBaseController::getNavigationResult() const {
    std::lock_guard<std::mutex> lock(navigation_mutex_);
    return last_result_code_;
}