#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "ftservo_hls3625_teach/servo_bus.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int rc = 0;

  try {
    auto node = std::make_shared<rclcpp::Node>("capture_zero_offsets");
    auto config = ftservo_hls3625_teach::declare_bus_config(*node);
    const auto output_yaml =
      node->declare_parameter<std::string>("output_yaml", "./servo_bus.generated.yaml");

    ftservo_hls3625_teach::ServoBus bus(config);
    bus.ping_all(node->get_logger());

    config.zero_offsets_raw = bus.read_positions_raw();
    for (std::size_t i = 0; i < config.ids.size(); ++i) {
      RCLCPP_INFO(
        node->get_logger(),
        "Captured zero offset: id=%u raw=%d",
        config.ids[i],
        config.zero_offsets_raw[i]);
    }

    ftservo_hls3625_teach::write_bus_config_file(output_yaml, config);
    RCLCPP_INFO(node->get_logger(), "Wrote config to %s", output_yaml.c_str());
  } catch (const std::exception & error) {
    fprintf(stderr, "capture_zero_offsets failed: %s\n", error.what());
    rc = 1;
  }

  rclcpp::shutdown();
  return rc;
}
