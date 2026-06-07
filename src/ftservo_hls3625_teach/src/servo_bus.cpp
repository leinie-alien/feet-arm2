#include "ftservo_hls3625_teach/servo_bus.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>

#include "SMSCL.h"

namespace ftservo_hls3625_teach
{

namespace
{

constexpr int kDefaultBaud = 1000000;
constexpr double kPi = 3.14159265358979323846;
constexpr double kPositionScaleRad = 0.087 * kPi / 180.0;
constexpr double kSpeedScaleRadPerSec = 0.732 * 2.0 * kPi / 60.0;
constexpr std::uint8_t kPresentPosition = 56;
constexpr std::uint8_t kPresentSpeed = 58;
constexpr int kSignedBit = 15;

int decode_signed_15_bit(int value)
{
  if (value < 0) {
    return value;
  }
  if ((value & (1 << kSignedBit)) != 0) {
    return -(value & ~(1 << kSignedBit));
  }
  return value;
}

double normalize_angle_rad(double angle_rad)
{
  return std::remainder(angle_rad, 2.0 * kPi);
}

std::vector<std::string> default_joint_names(const std::vector<int64_t> & ids)
{
  std::vector<std::string> names;
  names.reserve(ids.size());
  for (const auto id : ids) {
    names.push_back("servo_" + std::to_string(id));
  }
  return names;
}

std::string join_strings(const std::vector<std::string> & values)
{
  std::ostringstream stream;
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i != 0) {
      stream << ", ";
    }
    stream << "'" << values[i] << "'";
  }
  return stream.str();
}

void ensure_parent_directory(const std::string & output_path)
{
  const std::filesystem::path path(output_path);
  const auto parent = path.parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent);
  }
}

}  // namespace

class ServoBus::Impl
{
public:
  std::unique_ptr<SMSCL> servo;
};

BusConfig declare_bus_config(rclcpp::Node & node)
{
  BusConfig config;
  config.port = node.declare_parameter<std::string>("port", "/dev/ftservo_hls3625");
  config.baud = node.declare_parameter<int>("baud", kDefaultBaud);
  config.publish_hz = node.declare_parameter<double>("publish_hz", 30.0);
  config.move_speed_raw = node.declare_parameter<int>("move_speed_raw", 1200);
  config.move_acc_raw = node.declare_parameter<int>("move_acc_raw", 20);

  const auto ids_param =
    node.declare_parameter<std::vector<int64_t>>("ids", std::vector<int64_t>{1, 2, 3, 4, 5});
  const auto zero_offsets_param = node.declare_parameter<std::vector<int64_t>>(
    "zero_offsets_raw", std::vector<int64_t>(ids_param.size(), 0));
  const auto directions_param = node.declare_parameter<std::vector<int64_t>>(
    "directions", std::vector<int64_t>(ids_param.size(), -1));
  const auto joint_names_param = node.declare_parameter<std::vector<std::string>>(
    "joint_names", default_joint_names(ids_param));

  if (ids_param.empty()) {
    throw std::runtime_error("ids must not be empty");
  }
  if (zero_offsets_param.size() != ids_param.size()) {
    throw std::runtime_error("zero_offsets_raw length must match ids length");
  }
  if (directions_param.size() != ids_param.size()) {
    throw std::runtime_error("directions length must match ids length");
  }
  if (joint_names_param.size() != ids_param.size()) {
    throw std::runtime_error("joint_names length must match ids length");
  }
  if (config.publish_hz <= 0.0) {
    throw std::runtime_error("publish_hz must be positive");
  }
  if (config.move_speed_raw < 0) {
    throw std::runtime_error("move_speed_raw must be non-negative");
  }
  if (config.move_acc_raw < 0 || config.move_acc_raw > 255) {
    throw std::runtime_error("move_acc_raw must be in [0, 255]");
  }

  config.ids.reserve(ids_param.size());
  config.zero_offsets_raw.reserve(ids_param.size());
  config.directions.reserve(ids_param.size());
  config.joint_names = joint_names_param;

  for (std::size_t i = 0; i < ids_param.size(); ++i) {
    config.ids.push_back(static_cast<ServoId>(ids_param[i]));
    config.zero_offsets_raw.push_back(static_cast<int>(zero_offsets_param[i]));

    const int direction = static_cast<int>(directions_param[i]);
    if (direction != 1 && direction != -1) {
      throw std::runtime_error("directions entries must be either 1 or -1");
    }
    config.directions.push_back(direction);
  }

  return config;
}

std::string join_integers(const std::vector<int> & values)
{
  std::ostringstream stream;
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i != 0) {
      stream << ", ";
    }
    stream << values[i];
  }
  return stream.str();
}

std::string join_ids(const std::vector<ServoId> & values)
{
  std::ostringstream stream;
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i != 0) {
      stream << ", ";
    }
    stream << static_cast<int>(values[i]);
  }
  return stream.str();
}

void write_bus_config_file(const std::string & output_path, const BusConfig & config)
{
  ensure_parent_directory(output_path);

  std::ofstream output(output_path, std::ios::trunc);
  if (!output) {
    throw std::runtime_error("failed to open config output: " + output_path);
  }

  output
    << "/**:\n"
    << "  ros__parameters:\n"
    << "    port: '" << config.port << "'\n"
    << "    baud: " << config.baud << "\n"
    << "    publish_hz: " << config.publish_hz << "\n"
    << "    ids: [" << join_ids(config.ids) << "]\n"
    << "    joint_names: [" << join_strings(config.joint_names) << "]\n"
    << "    zero_offsets_raw: [" << join_integers(config.zero_offsets_raw) << "]\n"
    << "    directions: [" << join_integers(config.directions) << "]\n"
    << "    move_speed_raw: " << config.move_speed_raw << "\n"
    << "    move_acc_raw: " << config.move_acc_raw << "\n";
}

ServoBus::ServoBus(const BusConfig & config)
: config_(config), impl_(new Impl())
{
  impl_->servo = std::make_unique<SMSCL>();
  impl_->servo->IOTimeOut = 80;
  if (!impl_->servo->begin(config_.baud, config_.port.c_str())) {
    throw std::runtime_error("failed to open serial port " + config_.port);
  }
}

ServoBus::~ServoBus()
{
  delete impl_;
}

void ServoBus::ping_all(const rclcpp::Logger & logger)
{
  for (const auto id : config_.ids) {
    if (impl_->servo->Ping(id) == -1) {
      RCLCPP_WARN(logger, "Ping failed for servo ID %u", id);
    } else {
      RCLCPP_INFO(logger, "Servo ID %u online", id);
    }
  }
}

std::vector<ServoState> ServoBus::read_states()
{
  std::vector<ServoState> states;
  states.reserve(config_.ids.size());

  for (std::size_t i = 0; i < config_.ids.size(); ++i) {
    const auto id = config_.ids[i];
    const int position_raw = read_word_signed(id, kPresentPosition);
    const int speed_raw = read_word_signed(id, kPresentSpeed);
    const int calibrated_position = config_.directions[i] * (position_raw - config_.zero_offsets_raw[i]);
    const int calibrated_speed = config_.directions[i] * speed_raw;

    states.push_back(ServoState{
      id,
      position_raw,
      speed_raw,
      normalize_angle_rad(calibrated_position * kPositionScaleRad),
      calibrated_speed * kSpeedScaleRadPerSec,
    });
  }

  return states;
}

std::vector<int> ServoBus::read_positions_raw()
{
  std::vector<int> positions;
  positions.reserve(config_.ids.size());
  for (const auto id : config_.ids) {
    positions.push_back(read_word_signed(id, kPresentPosition));
  }
  return positions;
}

void ServoBus::set_torque_enabled(bool enabled)
{
  for (const auto id : config_.ids) {
    if (impl_->servo->EnableTorque(id, enabled ? 1 : 0) != 1) {
      throw std::runtime_error("failed to set torque state for servo ID " + std::to_string(id));
    }
  }
}

void ServoBus::move_to_raw_positions(
  const std::vector<int> & positions_raw,
  int speed_raw,
  int acc_raw)
{
  if (positions_raw.size() != config_.ids.size()) {
    throw std::runtime_error("pose length must match ids length");
  }

  for (std::size_t i = 0; i < config_.ids.size(); ++i) {
    const int raw = positions_raw[i];
    if (raw < -32767 || raw > 32767) {
      throw std::runtime_error("pose raw value out of range for servo ID " + std::to_string(config_.ids[i]));
    }
    if (impl_->servo->WritePosEx(
        config_.ids[i],
        static_cast<s16>(raw),
        static_cast<u16>(speed_raw),
        static_cast<u8>(acc_raw)) != 1)
    {
      throw std::runtime_error("failed to send position command for servo ID " + std::to_string(config_.ids[i]));
    }
  }
}

int ServoBus::read_word_signed(ServoId id, std::uint8_t address)
{
  const int raw = impl_->servo->readWord(id, address);
  if (raw == -1) {
    throw std::runtime_error("failed to read register " + std::to_string(address) + " from servo ID " +
      std::to_string(id));
  }
  return decode_signed_15_bit(raw);
}

}  // namespace ftservo_hls3625_teach
