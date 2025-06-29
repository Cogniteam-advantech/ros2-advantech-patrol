/*
 * tracer_base_node.cpp
 *
 * Created on: 3 2, 2021 16:39
 * Description: ROS2 Humble compatible version
 *
 * Copyright (c) 2022 agilex Robot Pte. Ltd.
 */

#include <memory>
#include <csignal>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "tracer_base/tracer_base_ros.hpp"

using namespace westonrobot;

std::shared_ptr<TracerBaseRos> robot;

void DetachRobot(int signal) {
  (void)signal;
  if (robot) {
    robot->Stop();
  }
  rclcpp::shutdown();
}

int main(int argc, char **argv) {
  // setup ROS node
  rclcpp::init(argc, argv);
  
  // Set up signal handler for graceful shutdown
  std::signal(SIGINT, DetachRobot);
  std::signal(SIGTERM, DetachRobot);

  try {
    robot = std::make_shared<TracerBaseRos>("tracer");
    
    if (!robot->Initialize()) {
      RCLCPP_ERROR(robot->get_logger(), "Failed to initialize robot");
      return -1;
    }
    
    RCLCPP_INFO(robot->get_logger(), "Robot initialized, start running ...");
    robot->Run();
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("tracer_base_node"), "Exception in main: %s", e.what());
    return -1;
  }

  rclcpp::shutdown();
  return 0;
}