#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "pipebot_services/srv/genes.hpp"  

#include <memory>
#include <utility>
#include <vector>

#include "ctrnn.h"

class PipebotControl : public rclcpp::Node
{
public:
  PipebotControl();

private:
  void init(std::vector<double> weights, std::vector<double> biases);
    
  void OnSensorMsg(const sensor_msgs::msg::LaserScan::SharedPtr _msg);
  void OnGeneSrv(const std::shared_ptr<pipebot_services::srv::Genes::Request> request, std::shared_ptr<pipebot_services::srv::Genes::Response> response);

  /// \brief Laser messages subscriber
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

  /// \brief Velocity command publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  
  CTRNN nn;
  rclcpp::Service<pipebot_services::srv::Genes>::SharedPtr init_service;
};
