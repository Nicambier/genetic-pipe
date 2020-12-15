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

#include "gazebo_ros_simple_motor/gazebo_ros_simple_motor.hpp"

#include <common/common.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo/physics/Model.hh>
#include <physics/physics.hh>
#include <sdf/sdf.hh>

#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros_simple_motor_msgs/msg/motor_control.hpp>

#include <cmath>
#include <memory>
#include <string>


namespace gazebo
{

typedef enum
{
  MODE_ABSOLUTE,
  MODE_RELATIVE,
  MODE_SPEED
} mode_t;

/**
 * @brief Handles the acceleration and deceleration of the motor to the
 * requested speed.
 */
class SimpleMotor
{
public:
  SimpleMotor(
    physics::JointPtr joint, double max_acceleration, double max_rpm,
    double max_change_radians);

  void Reset();

  void MoveRelative(double delta_radians);

  void MoveAbsolute(double new_position_radians);

  void SetSpeed(double new_rpm);

  void Update();

private:
  void UpdatePosition();
  void UpdateSpeed();

  /// The revolutions per minute (rpm) value that was last set for the motor.
  double current_rpm_;
  /// The rpm value that was requested for this motor.
  double target_rpm_;
  /// The maximum change in rpm per call to the Update function.
  double max_change_rpm_;
  /// The maximum rpm for the motor.
  double max_rpm_;
  /// The maximum change in radians per call to the Update function.
  double max_change_radians_;
  /// Pointer to the joint.
  physics::JointPtr joint_;
  /// The axis of rotation of the joint.
  unsigned int axis_;
  /// Mode
  mode_t mode_;
  /// Angle to move through in radians.
  double delta_radians_;
  /// Target angle.
  double target_angle_radians_;
};

SimpleMotor::SimpleMotor(
  physics::JointPtr joint, double max_change_rpm, double max_rpm,
  double max_change_radians)
: current_rpm_(0.0),
  target_rpm_(0.0),
  max_change_rpm_(max_change_rpm),
  max_rpm_(max_rpm),
  max_change_radians_(max_change_radians),
  joint_(joint),
  axis_(0),
  mode_(MODE_ABSOLUTE),
  delta_radians_(0.0),
  target_angle_radians_(0.0)
{
}

void SimpleMotor::Reset()
{
  current_rpm_ = 0.0;
  target_rpm_ = 0.0;
  delta_radians_ = 0.0;
  target_angle_radians_ = 0.0;
  mode_ = MODE_ABSOLUTE;
}

void SimpleMotor::MoveRelative(double delta_radians)
{
  mode_ = MODE_RELATIVE;
  // printf("%s: delta %f radians\n", __func__, delta_radians);
  delta_radians_ = delta_radians;
  double position_radians = joint_->Position(axis_);
  target_angle_radians_ = position_radians + delta_radians_;
  // printf(
  //   "%s: position_radians %f, target_angle_radians_ %f, delta_radians_ %f\n",
  //   __func__, position_radians, target_angle_radians_, delta_radians_);
}

void SimpleMotor::MoveAbsolute(double new_position_radians)
{
  // The tricky thing about this is that the new position should be in the
  // range -PI radians to +PI radians.
  mode_ = MODE_ABSOLUTE;
  // printf("%s: new position %f radians\n", __func__, new_position_radians);
  // Set the target angle.
  target_angle_radians_ = new_position_radians;

  // Calculate the current position in range -PI to +PI.
  double position_radians = joint_->Position(axis_);
  double position_radians_modulo = fmod(position_radians, M_PI);

  // The delta is the difference between the two values.
  delta_radians_ = new_position_radians - position_radians_modulo;
  // printf(
  //   "%s: position_radians %f, target_angle_radians_ %f, delta_radians_ %f\n",
  //   __func__, position_radians, target_angle_radians_, delta_radians_);
}

void SimpleMotor::SetSpeed(double new_rpm)
{
  mode_ = MODE_SPEED;
  // Clamp requested speed to maximum rpm in requested direction.
  if (abs(new_rpm) > abs(max_rpm_)) {
    bool clockwise = std::signbit(new_rpm);
    if (clockwise) {
      // Clockwise is negative.
      target_rpm_ = -max_rpm_;
    } else {
      target_rpm_ = max_rpm_;
    }
  } else {
    target_rpm_ = new_rpm;
  }
}

void SimpleMotor::Update()
{
  if (mode_ == MODE_SPEED) {
    UpdateSpeed();
  } else {
    UpdatePosition();
  }
}

void SimpleMotor::UpdatePosition()
{
  double next_position_radians = 0.0;
  double position_radians = joint_->Position(axis_);
  if (mode_ == MODE_ABSOLUTE) {
    // The 0.0001 is there to deal with rounding errors that caused oscillation.
    position_radians = fmod(position_radians, M_PI + 0.0001);
  }
  double absolute_difference_radians = abs(target_angle_radians_ - position_radians);
  if (absolute_difference_radians < max_change_radians_) {
    // Close enough.
    next_position_radians = target_angle_radians_;
    delta_radians_ = 0.0;
  } else {
    // Need to move a step closer.
    if (std::signbit(delta_radians_)) {
      // Negative
      next_position_radians = position_radians - max_change_radians_;
      delta_radians_ += max_change_radians_;
    } else {
      // Positive
      next_position_radians = position_radians + max_change_radians_;
      delta_radians_ -= max_change_radians_;
    }
    // Only move if there a need to.
    joint_->SetPosition(axis_, next_position_radians);
  }
  // printf(
  //   "%s: absolute_difference_radians %f, next_position_radians %f,"
  //   " target_angle_radians_ %f, delta_radians_ %f\n",
  //   __func__, absolute_difference_radians, next_position_radians,
  //   target_angle_radians_, delta_radians_);
}

void SimpleMotor::UpdateSpeed()
{
  // Velocity is in radians per second, +ve is CCW, -ve is CW.
  // Convert using 1 rad/s = 9.55 rpm.
  const double minimum_rpm = 0.1;
  double current_rad_s = joint_->GetVelocity(axis_);
  double current_rpm = 9.55 * current_rad_s;
  // Default action is keep speed constant.
  double next_rpm = current_rpm;
  // Any rpm value less than the minimum speed is treated as 0.
  // This simulates friction in the motor bearings.
  if (abs(target_rpm_) < minimum_rpm) {
    next_rpm = 0.0;
    // printf(
    //   "%s: 0 current %f, target %f, next %f\n",
    //   __func__, current_rpm, target_rpm_, next_rpm);
  } else {
    // See if the target rpm has been reached.
    if (std::signbit(current_rpm) == std::signbit(target_rpm_) && \
      abs(current_rpm - target_rpm_) < max_change_rpm_)
    {
      // Reached target rpm.
      next_rpm = target_rpm_;
      // printf(
      //   "%s: 1 current %f, target %f, next %f\n", __func__, current_rpm, target_rpm_,
      //   next_rpm);
    } else {
      // Change speed.
      if (current_rpm < target_rpm_) {
        next_rpm += max_change_rpm_;
        // printf(
        //   "%s: 2 current %f, target %f, next %f\n", __func__, current_rpm, target_rpm_,
        //   next_rpm);
      } else {
        next_rpm -= max_change_rpm_;
        // printf(
        //   "%s: 3 current %f, target %f, next %f\n", __func__, current_rpm, target_rpm_,
        //   next_rpm);
      }
    }
  }
  // Update the joint velocity.
  double new_rad_s = next_rpm / 9.55;
  joint_->SetVelocity(axis_, new_rad_s);
}


/**
 * @brief Implementation class that deals with the interface between ROS and
 * the SimpleMotor class.
 */
class GazeboRosSimpleMotorPrivate
{
public:
  GazeboRosSimpleMotorPrivate()
  : update_period_s_(0.0)
  {
  }
  rclcpp::Logger GetLogger();
  void Reset();
  bool SetupROSNode(sdf::ElementPtr sdf);
  bool SetupMotor(physics::ModelPtr model, sdf::ElementPtr sdf);

private:
  void OnUpdate(const gazebo::common::UpdateInfo & _info);
  void OnCmdMotor(const gazebo_ros_simple_motor_msgs::msg::MotorControl::SharedPtr msg);

  /// Pointer to model.
  gazebo::physics::ModelPtr model_;
  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;
  /// Subscriber to command velocities
  rclcpp::Subscription<gazebo_ros_simple_motor_msgs::msg::MotorControl>::SharedPtr cmd_motor_;
  /// The minimum interval between updates.
  double update_period_s_;
  // The motor instance.
  std::unique_ptr<SimpleMotor> motor_;
  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;
  /// Last update time.
  gazebo::common::Time last_update_time_;
};

rclcpp::Logger GazeboRosSimpleMotorPrivate::GetLogger()
{
  return ros_node_->get_logger();
}

void GazeboRosSimpleMotorPrivate::Reset()
{
  last_update_time_ = model_->GetWorld()->SimTime();
  update_period_s_ = 0.0;
  motor_->Reset();
}

bool GazeboRosSimpleMotorPrivate::SetupROSNode(sdf::ElementPtr sdf)
{
  // Initialize ROS node.  This is done early so that the logger can be used.
  ros_node_ = gazebo_ros::Node::Get(sdf);

  // Get QoS profiles.  USed by subscribers and publishers etc.
  const gazebo_ros::QoS & qos = ros_node_->get_qos();

  cmd_motor_ =
    ros_node_->create_subscription<gazebo_ros_simple_motor_msgs::msg::MotorControl>(
    "cmd_motor", qos.get_subscription_qos("cmd_motor", rclcpp::QoS(1)),
    std::bind(&GazeboRosSimpleMotorPrivate::OnCmdMotor, this, std::placeholders::_1));

  RCLCPP_INFO(GetLogger(), "Subscribed to [%s]", cmd_motor_->get_topic_name());

  // Listen to the update event (broadcast every simulation iteration)
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosSimpleMotorPrivate::OnUpdate, this, std::placeholders::_1));

  return true;
}

bool GazeboRosSimpleMotorPrivate::SetupMotor(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  bool success = false;

  // Save model for later use.
  model_ = model;

  // Read SDF file values.
  auto joint_name_ = sdf->Get<std::string>("motor_shaft_name", "").first;
  auto max_change_rpm = abs(sdf->Get<double>("max_change_rpm", 1.0).first);
  auto max_rpm = abs(sdf->Get<double>("max_rpm", 120.0).first);
  auto max_change_radians = abs(sdf->Get<double>("max_change_radians", 18.0).first);
  auto update_rate_hz = abs(sdf->Get<double>("update_rate", 10.0).first);
  if (update_rate_hz > 0.0) {
    update_period_s_ = 1.0 / update_rate_hz;
  } else {
    update_period_s_ = 0.0;
  }
  RCLCPP_INFO(GetLogger(), "Using joint [%s]", joint_name_.c_str());
  RCLCPP_INFO(GetLogger(), "Using max change rpm %f", max_change_rpm);
  RCLCPP_INFO(GetLogger(), "Using max rpm %f", max_rpm);
  RCLCPP_INFO(GetLogger(), "Using max change radians %f", max_change_radians);
  RCLCPP_INFO(GetLogger(), "Using update period %f", update_period_s_);

  // Create the motor instance.
  auto joint = model_->GetJoint(joint_name_);
  if (joint) {
    motor_ = std::make_unique<SimpleMotor>(joint, max_change_rpm, max_rpm, max_change_radians);
    success = true;
  } else {
    RCLCPP_ERROR(GetLogger(), "Could not get joint [%s]", joint_name_.c_str());
  }
  // Set update time.
  last_update_time_ = model_->GetWorld()->SimTime();
  return success;
}

void GazeboRosSimpleMotorPrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
  double since_last_update_s = (_info.simTime - last_update_time_).Double();
  if (since_last_update_s > update_period_s_) {
    motor_->Update();
    last_update_time_ = _info.simTime;
  }
}

void GazeboRosSimpleMotorPrivate::OnCmdMotor(
  const gazebo_ros_simple_motor_msgs::msg::MotorControl::SharedPtr msg)
{
  /*RCLCPP_INFO(
    GetLogger(), "Received: mode %d, rpm %f, angle_radians %f",
    msg->mode, msg->rpm, msg->angle_radians);*/
  if (msg->mode == MODE_SPEED) {
    motor_->SetSpeed(msg->rpm);
  } else if (msg->mode == MODE_ABSOLUTE) {
    motor_->MoveAbsolute(msg->angle_radians);
  } else if (msg->mode == MODE_RELATIVE) {
    motor_->MoveRelative(msg->angle_radians);
  }
}

/*****************************************************************************/

GazeboRosSimpleMotor::GazeboRosSimpleMotor()
: impl_(std::make_unique<GazeboRosSimpleMotorPrivate>())
{
}

GazeboRosSimpleMotor::~GazeboRosSimpleMotor()
{
}

void GazeboRosSimpleMotor::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Safety check
  unsigned int joint_count = model->GetJointCount();
  if (joint_count == 0) {
    std::cerr << "Invalid joint count, plugin not loaded\n";
  } else {
    // std::cout << "Found " << joint_count << " joints\n";
    bool success = impl_->SetupROSNode(sdf);
    if (success) {
      success = impl_->SetupMotor(model, sdf);
      if (success) {
        RCLCPP_INFO(impl_->GetLogger(), "Attached to Gazebo");
      }
    }
  }
}

void GazeboRosSimpleMotor::Reset()
{
  impl_->Reset();
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosSimpleMotor)

}  // namespace gazebo
