#include <memory>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"

#include "ftservo_hls3625_teach/pose_file.hpp"
#include "ftservo_hls3625_teach/servo_bus.hpp"

namespace
{

bool ids_match(
  const std::vector<ftservo_hls3625_teach::ServoId> & left,
  const std::vector<ftservo_hls3625_teach::ServoId> & right)
{
  if (left.size() != right.size()) {
    return false;
  }
  for (std::size_t i = 0; i < left.size(); ++i) {
    if (left[i] != right[i]) {
      return false;
    }
  }
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int rc = 0;

  try {
    auto node = std::make_shared<rclcpp::Node>("play_pose");
    const auto config = ftservo_hls3625_teach::declare_bus_config(*node);
    const auto pose_file =
      node->declare_parameter<std::string>("pose_file", "./last_pose.yaml");

    const auto pose = ftservo_hls3625_teach::read_pose_file(pose_file);
    if (!ids_match(config.ids, pose.ids)) {
      throw std::runtime_error(
              "pose ids do not match configured ids. pose=[" +
              ftservo_hls3625_teach::join_ids(pose.ids) +
              "] config=[" +
              ftservo_hls3625_teach::join_ids(config.ids) + "]");
    }

    ftservo_hls3625_teach::ServoBus bus(config);
    bus.ping_all(node->get_logger());
    bus.move_to_raw_positions(pose.positions_raw, config.move_speed_raw, config.move_acc_raw);

    RCLCPP_INFO(
      node->get_logger(),
      "Sent pose from %s with speed=%d acc=%d",
      pose_file.c_str(),
      config.move_speed_raw,
      config.move_acc_raw);
  } catch (const std::exception & error) {
    fprintf(stderr, "play_pose failed: %s\n", error.what());
    rc = 1;
  }

  rclcpp::shutdown();
  return rc;
}
