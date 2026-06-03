#include <chrono>
#include <cctype>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/srv/set_suction.hpp"

namespace
{
using SetSuction = robot_msgs::srv::SetSuction;
using namespace std::chrono_literals;
}

class SuctionKeyboardClient : public rclcpp::Node
{
public:
  SuctionKeyboardClient()
  : Node("suction_keyboard_client")
  {
    service_name_ = declare_parameter<std::string>("service_name", "set_suction");
    client_ = create_client<SetSuction>(service_name_);
  }

  int run()
  {
    while (rclcpp::ok()) {
      std::cout << "Input y to enable suction, n to disable, q to quit: " << std::flush;

      std::string line;
      if (!std::getline(std::cin, line)) {
        if (std::cin.eof()) {
          std::cout << std::endl;
          return 0;
        }
        RCLCPP_ERROR(get_logger(), "Failed to read from stdin");
        return 1;
      }

      if (line.empty()) {
        continue;
      }

      const char command = static_cast<char>(std::tolower(static_cast<unsigned char>(line.front())));
      if (command == 'q') {
        return 0;
      }
      if (command != 'y' && command != 'n') {
        std::cout << "Unsupported input. Use y, n, or q." << std::endl;
        continue;
      }

      if (!client_->wait_for_service(2s)) {
        RCLCPP_ERROR(get_logger(), "Service '%s' is unavailable", service_name_.c_str());
        continue;
      }

      auto request = std::make_shared<SetSuction::Request>();
      request->activate = (command == 'y');

      auto future = client_->async_send_request(request);
      const auto result = rclcpp::spin_until_future_complete(shared_from_this(), future, 2s);
      if (result != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Service call failed or timed out");
        continue;
      }

      const auto response = future.get();
      if (response->success) {
        std::cout << "Suction " << (request->activate ? "enabled" : "disabled") << "." << std::endl;
      } else {
        std::cout << "ESP32 rejected suction command." << std::endl;
      }
    }

    return 0;
  }

private:
  std::string service_name_;
  rclcpp::Client<SetSuction>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SuctionKeyboardClient>();
  const int rc = node->run();
  rclcpp::shutdown();
  return rc;
}
