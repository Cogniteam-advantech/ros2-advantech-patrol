/*
 * tracer_base_ros.hpp
 *
 * Created on: 3 2, 2022 16:38
 * Description: ROS2 Humble compatible version
 *
 * Copyright (c) 2022 Agilex Robot Pte. Ltd.
 */

#ifndef TRACER_BASE_ROS_HPP
#define TRACER_BASE_ROS_HPP

#include <atomic>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "ugv_sdk/mobile_robot/tracer_robot.hpp"

namespace westonrobot {
class TracerBaseRos : public rclcpp::Node {
 public:
  explicit TracerBaseRos(const std::string& node_name);
  ~TracerBaseRos() = default;

  bool Initialize();
  void Run();
  void Stop();

 private:
  std::string port_name_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_name_;

  bool is_tracer_mini_ = false;
  bool simulated_robot_ = false;
  int sim_control_rate_ = 50;

  std::shared_ptr<TracerRobot> robot_;
  std::atomic<bool> keep_running_{false};

  void LoadParameters();
};
}  // namespace westonrobot

#endif /* TRACER_BASE_ROS_HPP */