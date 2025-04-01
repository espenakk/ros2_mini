/**
 * @class PIDControllerNode
 * @brief A ROS2 node for controlling a system using a PID controller.
 *
 * This node subscribes to joint state measurements, applies a PID control algorithm,
 * and publishes the control voltage to a specified topic. It also provides a service
 * to set the reference value for the PID controller.
 *
 * @details
 * The node initializes with default PID parameters (kp, ki, kd) and a reference value.
 * These parameters can be dynamically updated through ROS2 parameter server.
 * The node subscribes to the "joint_states" topic to receive the current angle measurement,
 * computes the control voltage using the PID controller, and publishes the voltage to
 * the "/velocity_controller/commands" topic.
 *
 * The node also provides a service "qube_controller_node/set_reference" to set the reference
 * value for the PID controller. The reference value must be within the range [-π, π].
 *
 * @param kp_ Proportional gain of the PID controller.
 * @param ki_ Integral gain of the PID controller.
 * @param kd_ Derivative gain of the PID controller.
 * @param ref_ Reference value for the PID controller.
 *
 * @param pid_ Instance of the PID controller.
 * @param timer_ Timer for periodic tasks (not used in this implementation).
 * @param publish_voltage_ Publisher for the control voltage.
 * @param measured_angle_ Subscription for the joint state measurements.
 * @param parameter_callback Callback for dynamic parameter updates.
 * @param service_ Service for setting the reference value.
 *
 * @fn PIDControllerNode()
 * @brief Constructor for the PIDControllerNode class.
 *
 * @fn setReference(const std::shared_ptr<qube_controller_msgs::srv::SetReference::Request> request,
 *                  std::shared_ptr<qube_controller_msgs::srv::SetReference::Response> response)
 * @brief Callback function for the "set_reference" service.
 * @param request The service request containing the new reference value.
 * @param response The service response indicating success or failure.
 */

#include <chrono>
#include <cmath>
#include <cstdio>

#include "qube_controller_msgs/srv/set_reference.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
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
    voltage = 0.0;
    previousError = 0.0;
    integral = 0.0;
  }

  double getVoltage() { return voltage; }

  void update(double currentMeasurement) {
    double error = reference - currentMeasurement;
    integral += error;
    double derivative = error - previousError;
    voltage = (kp * error) + (ki * integral) + (kd * derivative);
    if (voltage > 100) {
      voltage = 100;
    }
    if (voltage < -100) {
      voltage = -100;
    }
    previousError = error;
  }

 private:
  double previousError;
  double integral;
  double voltage;
};

class PIDControllerNode : public rclcpp::Node {
 public:
  PIDControllerNode() : Node("qube_controller_node"), pid_(5.0, 0.001, 0.5) {
    publish_voltage_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controller/commands", 10);

    auto measurement_listener = [this](sensor_msgs::msg::JointState::UniquePtr msg) -> void {
      double currentMeasurement = msg->position[0];
      pid_.update(currentMeasurement);
      // RCLCPP_INFO(this->get_logger(), "I recievied this angle: angle='%f'", currentMeasurement * 180 / 3.1415);

      auto message_voltage = std_msgs::msg::Float64MultiArray();
      message_voltage.data.push_back(pid_.getVoltage());
      this->publish_voltage_->publish(message_voltage);
    };

    measured_angle_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, measurement_listener);

    this->declare_parameter("kp", 5.0);
    this->declare_parameter("ki", 0.001);
    this->declare_parameter("kd", 0.5);
    this->declare_parameter("ref", 0.0);
    this->get_parameter("kp", kp_);
    this->get_parameter("ki", ki_);
    this->get_parameter("kd", kd_);
    this->get_parameter("ref", ref_);

    pid_.kp = kp_;
    pid_.ki = ki_;
    pid_.kd = kd_;
    pid_.reference = ref_;

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
    // Create service for setting reference
    service_ = this->create_service<qube_controller_msgs::srv::SetReference>(
        "qube_controller_node/set_reference",
        std::bind(&PIDControllerNode::setReference, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to set reference");
  }

 private:
  PIDController pid_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publish_voltage_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr measured_angle_;
  double kp_;
  double ki_;
  double kd_;
  double ref_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback;

  // SetReference service callback
  void setReference(const std::shared_ptr<qube_controller_msgs::srv::SetReference::Request> request,
                    std::shared_ptr<qube_controller_msgs::srv::SetReference::Response> response) {
    if ((-M_PI) < (request->request.data) && (request->request.data) < (M_PI)) {
      pid_.reference = request->request.data;
      response->success.data = true;
    } else
      response->success.data = false;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request: [%f]", request->request.data);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%d]", response->success.data);
  }
  // Service for setting reference
  rclcpp::Service<qube_controller_msgs::srv::SetReference>::SharedPtr service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDControllerNode>());
  rclcpp::shutdown();
  return 0;
}