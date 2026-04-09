#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "ftservo_hls3625_teach/servo_bus.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int rc = 0;

  try {
    auto node = std::make_shared<rclcpp::Node>("set_torque");
    const auto config = ftservo_hls3625_teach::declare_bus_config(*node);
    const bool torque_enabled = node->declare_parameter<bool>("torque_enabled", false);

    ftservo_hls3625_teach::ServoBus bus(config);
    bus.ping_all(node->get_logger());
    bus.set_torque_enabled(torque_enabled);

    RCLCPP_INFO(
      node->get_logger(),
      "Torque %s for ids [%s]",
      torque_enabled ? "enabled" : "disabled",
      ftservo_hls3625_teach::join_ids(config.ids).c_str());
  } catch (const std::exception & error) {
    fprintf(stderr, "set_torque failed: %s\n", error.what());
    rc = 1;
  }

  rclcpp::shutdown();
  return rc;
}
