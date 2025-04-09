/**
 * @class PIDControllerNode
 * @brief A ROS2 node that implements a PID controller for controlling a system.
 *
 * This node subscribes to joint state measurements, applies a PID control algorithm,
 * and publishes control commands (outputs) to a specified topic. The PID gains and
 * reference value can be dynamically updated via ROS2 parameters.
 *
 * @details
 * - Subscribes to the "joint_states" topic to receive the current position measurement.
 * - Publishes control commands to the "/velocity_controller/commands" topic.
 * - PID gains (kp, ki, kd) and reference value (ref) are configurable via ROS2 parameters.
 * - Supports dynamic parameter updates using the `add_on_set_parameters_callback` mechanism.
 *
 * @param kp Proportional gain of the PID controller (default: 4.0).
 * @param ki Integral gain of the PID controller (default: 0.0).
 * @param kd Derivative gain of the PID controller (default: 8.0).
 * @param ref Reference value for the PID controller (default: 0.0).
 *
 * @publisher
 * - /velocity_controller/commands (std_msgs::msg::Float64MultiArray): Publishes the computed control output.
 *
 * @subscription
 * - joint_states (sensor_msgs::msg::JointState): Subscribes to the joint state measurements.
 */

#include <chrono>
#include <cmath>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class PIDController {
 public:
  double kp;
  double ki;
  double kd;
  double reference;

  PIDController(double kp_val, double ki_val, double kd_val) {
    kp = kp_val;
    ki = ki_val;
    kd = kd_val;
    reference = 0.0;
    output = 0.0;
    previousError = 0.0;
    integral = 0.0;
  }

  double getOutput() { return output; }

  void update(double measured_angle) {
    double error = reference - measured_angle;
    integral += error;
    double derivative = error - previousError;
    output = (kp * error) + (ki * integral) + (kd * derivative);
    if (output > 200) {
      output = 200;
    }
    if (output < -200) {
      output = -200;
    }
    previousError = error;
  }

 private:
  double previousError;
  double integral;
  double output;
};

class PIDControllerNode : public rclcpp::Node {
 public:
  PIDControllerNode() : Node("qube_controller_node"), pid_(0, 0, 0) {
    publish_output_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controller/commands", 10);

    auto joint_state_listener = [this](sensor_msgs::msg::JointState::UniquePtr msg) -> void {
      double measured_angle = msg->position[0];
      pid_.update(measured_angle);

      auto message_output = std_msgs::msg::Float64MultiArray();
      message_output.data.push_back(pid_.getOutput());
      this->publish_output_->publish(message_output);
    };

    current_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, joint_state_listener);

    this->declare_parameter("kp", 4.0);
    this->declare_parameter("ki", 0.0);
    this->declare_parameter("kd", 8.0);
    this->declare_parameter("ref", 0.0);
    this->get_parameter("kp", pid_.kp);
    this->get_parameter("ki", pid_.ki);
    this->get_parameter("kd", pid_.kd);
    this->get_parameter("ref", pid_.reference);

    parameter_callback = this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &parameters) {
      for (const auto &param : parameters) {
        if (param.get_name() == "kp") {
          pid_.kp = param.as_double();
        } else if (param.get_name() == "ki") {
          pid_.ki = param.as_double();
        } else if (param.get_name() == "kd") {
          pid_.kd = param.as_double();
        } else if (param.get_name() == "ref") {
          pid_.reference = param.as_double();
        }
      }
      rcl_interfaces::msg::SetParametersResult result;
      result.set__successful(true);
      return result;
    });
  }

 private:
  PIDController pid_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publish_output_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr current_joint_state_;
  double kp_;
  double ki_;
  double kd_;
  double ref_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDControllerNode>());
  rclcpp::shutdown();
  return 0;
}