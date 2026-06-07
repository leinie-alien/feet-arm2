#ifndef FTSERVO_HLS3625_TEACH__SERVO_BUS_HPP_
#define FTSERVO_HLS3625_TEACH__SERVO_BUS_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace ftservo_hls3625_teach
{

using ServoId = std::uint8_t;

struct BusConfig
{
  std::string port;
  int baud;
  double publish_hz;
  std::vector<ServoId> ids;
  std::vector<std::string> joint_names;
  std::vector<int> zero_offsets_raw;
  std::vector<int> directions;
  int move_speed_raw;
  int move_acc_raw;
};

struct ServoState
{
  ServoId id;
  int position_raw;
  int speed_raw;
  double position_rad;
  double speed_rad_s;
};

BusConfig declare_bus_config(rclcpp::Node & node);
std::string join_integers(const std::vector<int> & values);
std::string join_ids(const std::vector<ServoId> & values);
void write_bus_config_file(const std::string & output_path, const BusConfig & config);

class ServoBus
{
public:
  explicit ServoBus(const BusConfig & config);
  ~ServoBus();

  void ping_all(const rclcpp::Logger & logger);
  std::vector<ServoState> read_states();
  std::vector<int> read_positions_raw();
  void set_torque_enabled(bool enabled);
  void move_to_raw_positions(const std::vector<int> & positions_raw, int speed_raw, int acc_raw);

private:
  int read_word_signed(ServoId id, std::uint8_t address);
  BusConfig config_;
  class Impl;
  Impl * impl_;
};

}  // namespace ftservo_hls3625_teach

#endif  // FTSERVO_HLS3625_TEACH__SERVO_BUS_HPP_
