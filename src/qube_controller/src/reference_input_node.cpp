#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iostream>

#include "qube_controller_msgs/srv/set_reference.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("reference_input_node");

  auto client = node->create_client<qube_controller_msgs::srv::SetReference>("qube_controller_node/set_reference");

  while (rclcpp::ok()) {
    std::string input;
    std::cout << "Enter new reference value: ";
    std::getline(std::cin, input);

    auto request = std::make_shared<qube_controller_msgs::srv::SetReference::Request>();
    request->request.data = std::atof(input.c_str());

    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return 1;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
      if (result.get()->success.data != 0) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The new reference is: %f", request->request.data);
      } else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The requested reference is outside of valid range.");
      }
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service set_reference");
    }
  }

  rclcpp::shutdown();
  return 0;
}