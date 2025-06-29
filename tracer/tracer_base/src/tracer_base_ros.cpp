/*
 * tracer_base_ros.cpp
 *
 * Created on: 3 2, 2022 16:38
 * Description: ROS2 Humble compatible version
 *
 * Copyright (c) 2022 Agilex Robot Pte. Ltd.
 */

#include "tracer_base/tracer_base_ros.hpp"

#include <iostream>
#include <memory>

#include "tracer_base/tracer_messenger.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"

namespace westonrobot {
TracerBaseRos::TracerBaseRos(const std::string& node_name)
    : rclcpp::Node(node_name), keep_running_(false) {
  
  // Declare parameters with default values
  this->declare_parameter("port_name", std::string("can0"));
  this->declare_parameter("odom_frame", std::string("odom"));
  this->declare_parameter("base_frame", std::string("base_link"));
  this->declare_parameter("odom_topic_name", std::string("odom"));
  this->declare_parameter("is_tracer_mini", false);
  this->declare_parameter("simulated_robot", false);
  this->declare_parameter("control_rate", 50);

  LoadParameters();
}

void TracerBaseRos::LoadParameters() {
  // Get parameters with fallback to default values
  port_name_ = this->get_parameter("port_name").as_string();
  odom_frame_ = this->get_parameter("odom_frame").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();
  odom_topic_name_ = this->get_parameter("odom_topic_name").as_string();
  is_tracer_mini_ = this->get_parameter("is_tracer_mini").as_bool();
  simulated_robot_ = this->get_parameter("simulated_robot").as_bool();
  sim_control_rate_ = this->get_parameter("control_rate").as_int();

  RCLCPP_INFO(this->get_logger(), "Loading parameters:");
  RCLCPP_INFO(this->get_logger(), "- port name: %s", port_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "- odom frame name: %s", odom_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "- base frame name: %s", base_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "- odom topic name: %s", odom_topic_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "- is tracer mini: %s", is_tracer_mini_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "- simulated robot: %s", simulated_robot_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "- sim control rate: %d", sim_control_rate_);
  RCLCPP_INFO(this->get_logger(), "----------------------------");
}

bool TracerBaseRos::Initialize() {
  if (is_tracer_mini_) {
    RCLCPP_INFO(this->get_logger(), "Robot base: Tracer Mini");
  } else {
    RCLCPP_INFO(this->get_logger(), "Robot base: Tracer");
  }

  // Skip hardware initialization if in simulation mode
  if (simulated_robot_) {
    RCLCPP_INFO(this->get_logger(), "Running in simulation mode");
    robot_ = std::make_shared<TracerRobot>();
    return true;
  }

  ProtocolDetector detector;
  if (detector.Connect(port_name_)) {
    auto proto = detector.DetectProtocolVersion(5);
    if (proto == ProtocolVersion::AGX_V2) {
      RCLCPP_INFO(this->get_logger(), "Detected protocol: AGX_V2");
      robot_ = std::make_shared<TracerRobot>();
      RCLCPP_INFO(this->get_logger(), "Creating interface for Tracer with AGX_V2 Protocol");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Detected protocol: UNKNOWN");
      return false;
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to port: %s", port_name_.c_str());
    return false;
  }

  return true;
}

void TracerBaseRos::Stop() { 
  keep_running_ = false; 
  RCLCPP_INFO(this->get_logger(), "Stopping tracer base node...");
}

void TracerBaseRos::Run() {
  if (!robot_) {
    RCLCPP_ERROR(this->get_logger(), "Robot not initialized, cannot run");
    return;
  }

  // Create messenger
  auto messenger = std::make_unique<TracerMessenger<TracerRobot>>(robot_, this);

  messenger->SetOdometryFrame(odom_frame_);
  messenger->SetBaseFrame(base_frame_);
  messenger->SetOdometryTopicName(odom_topic_name_);
  
  if (simulated_robot_) {
    messenger->SetSimulationMode(sim_control_rate_);
  }

  // Connect to robot if not in simulation mode
  if (!simulated_robot_) {
    if (port_name_.find("can") != std::string::npos) {
      if (robot_->Connect(port_name_)) {
        robot_->EnableCommandedMode();
        RCLCPP_INFO(this->get_logger(), "Using CAN bus to talk with the robot");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to the robot CAN bus");
        return;
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Please check the specified port name is a CAN port");
      return;
    }
  }

  // Setup ROS subscriptions and publishers
  messenger->SetupSubscription();
  
  keep_running_ = true;
  rclcpp::Rate rate(50);
  
  RCLCPP_INFO(this->get_logger(), "Starting main loop...");
  
  while (keep_running_ && rclcpp::ok()) {
    try {
      messenger->PublishStateToROS();
      rclcpp::spin_some(shared_from_this());
      rate.sleep();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error in main loop: %s", e.what());
      break;
    }
  }
  
  RCLCPP_INFO(this->get_logger(), "Main loop ended");
}
}  // namespace westonrobot