#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class JointSimulator {
 public:
  double angle;
  double angular_velocity;
  double voltage;
  double noise;
  double K;  // Radians
  double T;  // Radians per second

  void update() {
    double dt = 0.01;  // Time step
    angular_velocity += dt * (-angular_velocity / T + K * voltage / T);
    angle += (dt * angular_velocity) + noise;
  }
};

class JointSimulatorNode : public rclcpp::Node {
 public:
  JointSimulatorNode() : Node("joint_simulator") {
    publish_angle();
    input_voltage();
    set_parameters();

    RCLCPP_INFO(this->get_logger(), "Initial Noise parameter value: %f", jointSimulator.noise);
    RCLCPP_INFO(this->get_logger(), "Initial K parameter value: %f", jointSimulator.K);
    RCLCPP_INFO(this->get_logger(), "Initial T parameter value: %f", jointSimulator.T);
  }

 private:
  void set_parameters() {
    this->declare_parameter<double>("noise", 0.5);
    this->declare_parameter<double>("K", 230.0);
    this->declare_parameter<double>("T", 0.15);

    jointSimulator.noise = this->get_parameter("noise").as_double();
    jointSimulator.K = this->get_parameter("K").as_double();
    jointSimulator.T = this->get_parameter("T").as_double();

    parameter_callback = this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &parameters) {
      for (const auto &param : parameters) {
        if (param.get_name() == "noise") {
          jointSimulator.noise = param.as_double();
          RCLCPP_INFO(this->get_logger(), "Noise parameter set to: %f", jointSimulator.noise);
        } else if (param.get_name() == "K") {
          jointSimulator.K = param.as_double();
          RCLCPP_INFO(this->get_logger(), "K parameter set to: %f", jointSimulator.K);
        } else if (param.get_name() == "T") {
          jointSimulator.T = param.as_double();
          RCLCPP_INFO(this->get_logger(), "T parameter set to: %f", jointSimulator.T);
        }
      }
      rcl_interfaces::msg::SetParametersResult result;
      result.set__successful(true);
      return result;
    });
  }

  void publish_angle() {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("angle", 10);
    auto timer_callback = [this]() -> void {
      jointSimulator.update();
      auto message = std_msgs::msg::Float64();
      message.data = jointSimulator.angle;
      RCLCPP_INFO(this->get_logger(), "Publishing current angle: '%f'", message.data);
      this->publisher_->publish(message);
    };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

  void input_voltage() {
    auto voltage_listener = [this](std_msgs::msg::Float64::UniquePtr msg) -> void {
      RCLCPP_INFO(this->get_logger(), "I received: '%f'", msg->data);
      jointSimulator.voltage = msg->data;
    };
    subscription_ = this->create_subscription<std_msgs::msg::Float64>("voltage", 10, voltage_listener);
  }

  JointSimulator jointSimulator;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
