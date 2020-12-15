/*
Copyright (c) 2020 University of Leeds.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/// @brief This file sends commands to the plugin for visual verification.

#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "gazebo_ros_simple_motor_msgs/msg/motor_control.hpp"


/// This delay is set to the smallest value that allows the tests to pass every time.
static const std::chrono::milliseconds kConnectionDelayMs(250);
/// This controls the timer callback rate.
static const std::chrono::milliseconds kPublisherDelayMs(2500);
/// The topic name to use to test the plugin.
static const char kTopicName[] = "/test/cmd_motor";


class TestPublisher : public rclcpp::Node
{
public:
  TestPublisher()
  : Node("test_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<gazebo_ros_simple_motor_msgs::msg::MotorControl>(
      kTopicName,
      10);
    timer_ =
      this->create_wall_timer(kPublisherDelayMs, std::bind(&TestPublisher::timer_callback, this));
  }

private:
  void SetMessage(gazebo_ros_simple_motor_msgs::msg::MotorControl * message)
  {
    switch (count_) {
      // Test absolute positions.
      case 0:
        message->mode = 0;
        message->angle_radians = -M_PI;
        message->rpm = 0.0;
        break;
      case 1:
        message->mode = 0;
        message->angle_radians = +M_PI;
        message->rpm = 0.0;
        break;
      case 2:
        message->mode = 0;
        message->angle_radians = -3;
        message->rpm = 0.0;
        break;
      case 3:
        message->mode = 0;
        message->angle_radians = +2;
        message->rpm = 0.0;
        break;
      case 4:
        message->mode = 0;
        message->angle_radians = -1;
        message->rpm = 0.0;
        break;
      case 5:
        message->mode = 0;
        message->angle_radians = 0;
        message->rpm = 0.0;
        break;
      case 6:
        message->mode = 0;
        message->angle_radians = 0;
        // Check rpm is ignored.
        message->rpm = 100.0;
        break;
      // Test relative changes.
      case 7:
        message->mode = 1;
        message->angle_radians = -1;
        message->rpm = 0.0;
        break;
      case 8:
        message->mode = 1;
        message->angle_radians = 2;
        message->rpm = 0.0;
        break;
      case 9:
        message->mode = 1;
        message->angle_radians = -3;
        message->rpm = 0.0;
        break;
      case 10:
        message->mode = 1;
        message->angle_radians = 4;
        message->rpm = 0.0;
        break;
      case 11:
        message->mode = 1;
        message->angle_radians = 0.5;
        message->rpm = 0.0;
        break;
      case 12:
        message->mode = 1;
        message->angle_radians = 0.0;
        // Check rpm is ignored.
        message->rpm = 100.0;
        break;
      // Test rpm.
      case 13:
        message->mode = 2;
        message->angle_radians = 0.0;
        message->rpm = 50.0;
        break;
      // Accelerate to max speed.
      // 600rpm is greater than set in SDf file of 500.
      case 14:
      case 15:
      case 16:
      case 17:
        message->mode = 2;
        message->angle_radians = 0.0;
        message->rpm = 600.0;
        break;
      // Reverse from high speed.  Needs some time to do this.
      case 18:
      case 19:
      case 20:
      case 21:
      case 22:
      case 23:
      case 24:
      case 25:
        message->mode = 2;
        message->angle_radians = 0.0;
        message->rpm = -600.0;
        break;
      case 26:
      case 27:
      case 28:
        message->mode = 2;
        message->angle_radians = 0.0;
        message->rpm = -50.0;
        break;
      case 29:
        message->mode = 2;
        message->angle_radians = 0.0;
        message->rpm = 0.0;
        break;
      case 30:
        message->mode = 2;
        // Check angle is ignored.
        message->angle_radians = 1.0;
        message->rpm = 0.0;
        // Reset count.  -1 as incremented after call.
        count_ = -1;
        break;
    }
  }

  void timer_callback()
  {
    auto message = gazebo_ros_simple_motor_msgs::msg::MotorControl();
    SetMessage(&message);
    RCLCPP_INFO(
      this->get_logger(), "Publishing: mode %d, angle %fr, rpm %f",
      message.mode, message.angle_radians, message.rpm);
    publisher_->publish(message);
    ++count_;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<gazebo_ros_simple_motor_msgs::msg::MotorControl>::SharedPtr publisher_;
  int count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestPublisher>());
  rclcpp::shutdown();
  return 0;
}
