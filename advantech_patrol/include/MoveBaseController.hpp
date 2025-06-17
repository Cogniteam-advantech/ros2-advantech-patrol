/*
 * MoveBaseController.hpp
 *
 *  Created on: Mar 7, 2016
 *      Author: blackpc
 *  Converted to ROS2 Humble
 */

#ifndef INCLUDE_MOVEBASECONTROLLER_HPP_
#define INCLUDE_MOVEBASECONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

/**
 * Wrapper of nav2's navigation action client for convenient way to send goals and get results
 */
class MoveBaseController {

public:
    MoveBaseController(rclcpp::Node::SharedPtr node);
    virtual ~MoveBaseController();

    /**
     * Sending a goal to nav2
     * @note This method doesn't wait until navigation is finished
     * @param goal
     */
    void navigate(const geometry_msgs::msg::PoseStamped& goal);

    /**
     * Waits for nav2 to finish
     * @return True if navigation succeeded, false otherwise
     */
    bool wait();

    /**
     * Waits for the ActionServer to connect to this client
     * @param timeout
     * @return
     */
    bool waitForServer(const std::chrono::seconds& timeout = std::chrono::seconds(10));

    /**
     * Cancels navigation process
     */
    void cancelNavigation();

    /**
     * Check if navigation is active
     */
    bool isNavigating() const;

    /**
     * Get current navigation state
     */
    rclcpp_action::ResultCode getNavigationResult() const;

private:
    /**
     * Creates nav2's navigate to pose goal message
     * @param goal Goal pose
     * @return
     */
    NavigateToPose::Goal createGoalMessage(const geometry_msgs::msg::PoseStamped& goal) const;

    /**
     * Goal response callback
     */
    void goalResponseCallback(const GoalHandleNavigateToPose::SharedPtr& goal_handle);

    /**
     * Feedback callback
     */
    void feedbackCallback(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback);

    /**
     * Result callback
     */
    void resultCallback(const GoalHandleNavigateToPose::WrappedResult& result);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    GoalHandleNavigateToPose::SharedPtr goal_handle_;
    
    bool navigation_active_;
    rclcpp_action::ResultCode last_result_code_;
    std::mutex navigation_mutex_;
};

#endif /* INCLUDE_MOVEBASECONTROLLER_HPP_ */