#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "pipebot_services/srv/genes.hpp"  
#include "gazebo_ros_simple_motor_msgs/msg/motor_control.hpp"  

#include <math.h> 
#include <stdlib.h> 
#include <memory>
#include <utility>
#include <vector>

#include "ctrnn.h"

class PipebotControl : public rclcpp::Node
{
public:
  PipebotControl();

private:
  void init(std::vector<double> weights, std::vector<double> biases, bool symmetrical=false);
    
  void OnSensorMsg(const sensor_msgs::msg::LaserScan::SharedPtr _msg);
  void OnGeneSrv(const std::shared_ptr<pipebot_services::srv::Genes::Request> request, std::shared_ptr<pipebot_services::srv::Genes::Response> response);

  /// \brief Laser messages subscriber
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

  /// \brief Motor command publisher
  rclcpp::Publisher<gazebo_ros_simple_motor_msgs::msg::MotorControl>::SharedPtr cmd_left_pub_;
  rclcpp::Publisher<gazebo_ros_simple_motor_msgs::msg::MotorControl>::SharedPtr cmd_right_pub_;
  
  CTRNN nn;
  rclcpp::Service<pipebot_services::srv::Genes>::SharedPtr init_service;
  int last_scan_nsec;
};
