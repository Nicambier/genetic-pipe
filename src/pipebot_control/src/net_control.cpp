#include "net_control.h"

using namespace std;

/// \brief PipebotControl node, which subscribes to laser scan messages and publishes
/// velocity commands.
PipebotControl::PipebotControl() : Node("PipebotControl")
{
    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    // Subscribe to sensor messages
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "laser_scan", default_qos,
        bind(&PipebotControl::OnSensorMsg, this, placeholders::_1));
    
    // Advertise velocity commands
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", default_qos); 
    
    init_service = this->create_service<pipebot_services::srv::Genes>("neural_network", bind(&PipebotControl::OnGeneSrv, this, placeholders::_1,placeholders::_2));
    
    init(vector<double>(16,0),vector<double>(8,0));
}

void PipebotControl::init(vector<double> weights, vector<double> biases) {
    unsigned int input = 3, hidden_layer = 2, output = 3;
    
    nn = CTRNN(input,hidden_layer,output);
    //INPUT
    nn.setWeight(0,3,weights[0]);
    nn.setWeight(1,3,weights[1]);
    nn.setWeight(2,3,weights[2]);
    nn.setWeight(0,4,weights[3]);
    nn.setWeight(1,4,weights[4]);
    nn.setWeight(2,4,weights[5]);
    
    //HIDDEN
    nn.setWeight(3,3,weights[6]);
    nn.setWeight(3,4,weights[7]);
    nn.setWeight(4,3,weights[8]);
    nn.setWeight(4,4,weights[9]);
    
    //OUTPUT
    nn.setWeight(3,5,weights[10]);
    nn.setWeight(3,6,weights[11]);
    nn.setWeight(3,7,weights[12]);
    nn.setWeight(4,5,weights[13]);
    nn.setWeight(4,6,weights[14]);
    nn.setWeight(4,7,weights[15]);
    
    for(unsigned int i=0; i<input+hidden_layer+output; ++i)
        nn.setBias(i,biases[i]);
}

/// \brief Callback for sensor message subscriber
/// \param[in] _msg Laser scan message
void PipebotControl::OnSensorMsg(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
{
    vector<double> input;
    for (auto i = 0u; i < _msg->ranges.size(); ++i) {
      input.push_back(_msg->ranges[i]>100?1:_msg->ranges[i]);
    }

    // Calculate desired yaw change
    //double turn = _msg->angle_min + _msg->angle_increment * idx;
    vector<double> output = nn.run(input);
    auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
    cmd_msg->linear.x = output[0] - 0.5;
    cmd_msg->linear.y = output[1] - 0.5;
    cmd_msg->angular.z = output[2] - 0.5;
    
    /*auto cmd_msg = make_unique<geometry_msgs::msg::Twist>();
    cmd_msg->linear.x = 0.5;
    cmd_msg->linear.y = 0;
    cmd_msg->angular.z = 0;*/
    
    cmd_pub_->publish(move(cmd_msg));
}

void PipebotControl::OnGeneSrv(const shared_ptr<pipebot_services::srv::Genes::Request> request, shared_ptr<pipebot_services::srv::Genes::Response> response)
{
    cmd_pub_->publish(move(std::make_unique<geometry_msgs::msg::Twist>()));
    init(request->weights, request->biases);
    response->success = true;
}

int main(int argc, char * argv[])
{
  // Forward command line arguments to ROS
  rclcpp::init(argc, argv);

  // Create a node
  auto node = make_shared<PipebotControl>();

  // Run node until it's exited
  rclcpp::spin(node);

  // Clean up
  rclcpp::shutdown();
  return 0;
}
