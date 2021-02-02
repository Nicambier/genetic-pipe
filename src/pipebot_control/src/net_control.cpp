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
    cmd_right_pub_ = this->create_publisher<gazebo_ros_simple_motor_msgs::msg::MotorControl>("cmd_right_motor", default_qos); 
    cmd_left_pub_ = this->create_publisher<gazebo_ros_simple_motor_msgs::msg::MotorControl>("cmd_left_motor", default_qos); 
    
    init_service = this->create_service<pipebot_services::srv::Genes>("neural_network", bind(&PipebotControl::OnGeneSrv, this, placeholders::_1,placeholders::_2));
    
    init(vector<double>(36,0),vector<double>(6,0));
}

void PipebotControl::init(vector<double> weights, vector<double> biases, bool symmetrical) {
    last_scan_nsec = 0;
    unsigned int input = 3, hidden_layer = 2, output = 2;
    
    nn = CTRNN(input,hidden_layer,output);
    
    unsigned int nb_nodes = input + hidden_layer + output;
    unsigned int weight_idx = 0;
    
    if(symmetrical) { //good luck to ever understand this. this should make the nn symmetrical but I'm not ever sure it works outside of CTRNN(3,2,2)
        unsigned int i_layer_start, i_layer_size, j_layer_start, j_layer_size;
        for(unsigned int i_layer=0; i_layer<3; ++i_layer) {
            switch(i_layer) {
                case 0:
                    i_layer_start = 0;
                    i_layer_size = input;
                    break;
                case 1:
                    i_layer_start = input;
                    i_layer_size = hidden_layer;
                    break;
                case 2:
                    i_layer_start = input + hidden_layer;
                    i_layer_size = output;
                    break;
            }
            
            for(unsigned int i=0; i<round(i_layer_size/2.0); ++i) {
                for(unsigned int j_layer=0; j_layer<3; ++j_layer) {
                    switch(j_layer) {
                        case 0:
                            j_layer_start = 0;
                            j_layer_size = input;
                            break;
                        case 1:
                            j_layer_start = input;
                            j_layer_size = hidden_layer;
                            break;
                        case 2:
                            j_layer_start = input + hidden_layer;
                            j_layer_size = output;
                            break;
                    }
                    //if in center row, only do half columns coz the other half is filled by symmetry
                    unsigned int cols = (i_layer_size%2==1 and i==i_layer_size/2)? round(j_layer_size / 2.0) : j_layer_size; 
                    for(unsigned int j=0; j<cols; ++j) {
                        nn.setWeight(i_layer_start + i,j_layer_start + j,weights[weight_idx]); 
                        nn.setWeight(i_layer_start + i_layer_size -1-i,j_layer_start + j_layer_size -1-j,weights[weight_idx]);
                        ++weight_idx;
                    }
                }
            }
        }
    } else {
        for(unsigned int i=0; i<nb_nodes; ++i) {
            for(unsigned int j=0; j<nb_nodes; ++j) {
                nn.setWeight(i,j,weights[weight_idx]);
                ++weight_idx;
            }
        }
    }
    
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), nn.print());
    
    for(unsigned int i=0; i<input+hidden_layer+output; ++i)
        nn.setBias(i,biases[i]);
}

/// \brief Callback for sensor message subscriber
/// \param[in] _msg Laser scan message
void PipebotControl::OnSensorMsg(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
{
    int nsec = _msg->header.stamp.nanosec;
    double deltaT = (nsec - last_scan_nsec) * 0.000000001;
    if(deltaT<0) deltaT += 1;
    last_scan_nsec = nsec;
    vector<double> input;
    //0=right 1=front 2=left
    for (auto i = 0u; i < _msg->ranges.size(); ++i) {
      input.push_back(1.0 - (_msg->ranges[i]>1.0?1.0:_msg->ranges[i])); //signal gets stronger as obstacle gets closer
    }

    vector<double> output = nn.run(input, deltaT);
    auto cmd_left_msg = gazebo_ros_simple_motor_msgs::msg::MotorControl();
    auto cmd_right_msg = gazebo_ros_simple_motor_msgs::msg::MotorControl();
    cmd_left_msg.mode = 2;
    cmd_left_msg.rpm = 60*(0.5 - output[0]);
    cmd_right_msg.mode = 2;
    cmd_right_msg.rpm = 60*(0.5 - output[1]);
    
    cmd_left_pub_->publish(cmd_left_msg);
    cmd_right_pub_->publish(cmd_right_msg);
}

void PipebotControl::OnGeneSrv(const shared_ptr<pipebot_services::srv::Genes::Request> request, shared_ptr<pipebot_services::srv::Genes::Response> response)
{
    cmd_left_pub_->publish(gazebo_ros_simple_motor_msgs::msg::MotorControl());
    cmd_right_pub_->publish(gazebo_ros_simple_motor_msgs::msg::MotorControl());
    init(request->weights, request->biases, true);
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
