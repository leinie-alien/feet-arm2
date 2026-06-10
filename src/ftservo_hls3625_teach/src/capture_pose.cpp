#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "ftservo_hls3625_teach/pose_file.hpp"
#include "ftservo_hls3625_teach/servo_bus.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int rc = 0;

  try {
    auto node = std::make_shared<rclcpp::Node>("capture_pose");
    const auto config = ftservo_hls3625_teach::declare_bus_config(*node);
    const auto pose_file =
      node->declare_parameter<std::string>("pose_file", "./last_pose.yaml");

    ftservo_hls3625_teach::ServoBus bus(config);
    bus.ping_all(node->get_logger());

    ftservo_hls3625_teach::RawPose pose;
    pose.ids = config.ids;
    pose.positions_raw = bus.read_positions_raw();

    for (std::size_t i = 0; i < pose.ids.size(); ++i) {
      RCLCPP_INFO(
        node->get_logger(),
        "Captured pose: id=%u raw=%d",
        pose.ids[i],
        pose.positions_raw[i]);
    }

    ftservo_hls3625_teach::write_pose_file(pose_file, pose);
    RCLCPP_INFO(node->get_logger(), "Wrote pose to %s", pose_file.c_str());
  } catch (const std::exception & error) {
    fprintf(stderr, "capture_pose failed: %s\n", error.what());
    rc = 1;
  }

  rclcpp::shutdown();
  return rc;
}
