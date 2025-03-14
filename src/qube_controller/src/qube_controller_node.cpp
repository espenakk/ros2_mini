#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <vector>

class QubeControllerNode : public rclcpp::Node
{
public:
  QubeControllerNode() : Node("qube_controller")
  {
    // Initialize PID parameters
    kp_ = this->declare_parameter<double>("kp", 1.0);
    ki_ = this->declare_parameter<double>("ki", 0.1);
    kd_ = this->declare_parameter<double>("kd", 0.05);
    
    integral_ = 0.0;
    prev_error_ = 0.0;
    prev_time_ = this->now();
    setpoint_ = this->declare_parameter<double>("setpoint", 0.0);
    
    // Create subscriber for joint states
    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, 
      std::bind(&QubeControllerNode::joint_states_callback, this, std::placeholders::_1));
    
    // Create publisher for velocity command
    velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/velocity_controller/command", 10);
    
    RCLCPP_INFO(this->get_logger(), "Qube Controller Node initialized");
  }

private:
  // PID controller parameters
  double kp_, ki_, kd_;
  double integral_, prev_error_;
  double setpoint_;
  rclcpp::Time prev_time_;
  
  // ROS2 communication
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
  
  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->position.empty() || msg->velocity.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty joint_states message");
      return;
    }
    
    // Extract position and velocity
    double position = msg->position[0];
    double velocity = msg->velocity[0];
    
    RCLCPP_DEBUG(this->get_logger(), "Received position: %f, velocity: %f", position, velocity);
    
    // Calculate control value using PID
    double control_value = calculate_pid(position);
    
    // Publish velocity command
    publish_velocity_command(control_value);
  }
  
  double calculate_pid(double current_position)
  {
    auto current_time = this->now();
    double dt = (current_time - prev_time_).seconds();
    prev_time_ = current_time;
    
    // Calculate error
    double error = setpoint_ - current_position;
    
    // Proportional term
    double p_term = kp_ * error;
    
    // Integral term
    integral_ += error * dt;
    double i_term = ki_ * integral_;
    
    // Derivative term
    double derivative = dt > 0 ? (error - prev_error_) / dt : 0.0;
    double d_term = kd_ * derivative;
    
    // Save current error for next iteration
    prev_error_ = error;
    
    // Sum all terms
    double control_value = p_term + i_term + d_term;
    
    RCLCPP_DEBUG(this->get_logger(), "PID output: %f (P: %f, I: %f, D: %f)", 
                control_value, p_term, i_term, d_term);
    
    return control_value;
  }
  
  void publish_velocity_command(double control_value)
  {
    auto command_msg = std_msgs::msg::Float64MultiArray();
    
    // Set up the message structure
    command_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    command_msg.layout.dim[0].size = 1;
    command_msg.layout.dim[0].stride = 1;
    command_msg.layout.dim[0].label = "velocity";
    
    // Set the data - adding the control value to the array
    command_msg.data.push_back(control_value);
    
    // Publish the message
    velocity_pub_->publish(command_msg);
    
    RCLCPP_DEBUG(this->get_logger(), "Published velocity command: %f", control_value);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<QubeControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}