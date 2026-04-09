#include "ftservo_hls3625_teach/pose_file.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace ftservo_hls3625_teach
{

namespace
{

std::string trim(const std::string & value)
{
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1);
}

std::vector<int> parse_integer_list(const std::string & line)
{
  const auto left = line.find('[');
  const auto right = line.find(']');
  if (left == std::string::npos || right == std::string::npos || right <= left) {
    throw std::runtime_error("invalid list syntax: " + line);
  }

  std::vector<int> values;
  std::stringstream stream(line.substr(left + 1, right - left - 1));
  std::string token;
  while (std::getline(stream, token, ',')) {
    const auto stripped = trim(token);
    if (!stripped.empty()) {
      values.push_back(std::stoi(stripped));
    }
  }
  return values;
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

RawPose read_pose_file(const std::string & pose_file)
{
  std::ifstream input(pose_file);
  if (!input) {
    throw std::runtime_error("failed to open pose file: " + pose_file);
  }

  std::vector<int> ids_as_int;
  std::vector<int> positions_raw;
  std::string line;
  while (std::getline(input, line)) {
    const auto stripped = trim(line);
    if (stripped.rfind("ids:", 0) == 0) {
      ids_as_int = parse_integer_list(stripped);
    } else if (stripped.rfind("positions_raw:", 0) == 0) {
      positions_raw = parse_integer_list(stripped);
    }
  }

  if (ids_as_int.empty()) {
    throw std::runtime_error("pose file is missing ids: " + pose_file);
  }
  if (positions_raw.empty()) {
    throw std::runtime_error("pose file is missing positions_raw: " + pose_file);
  }
  if (ids_as_int.size() != positions_raw.size()) {
    throw std::runtime_error("pose ids length must match positions_raw length");
  }

  RawPose pose;
  pose.positions_raw = positions_raw;
  pose.ids.reserve(ids_as_int.size());
  for (const auto value : ids_as_int) {
    pose.ids.push_back(static_cast<ServoId>(value));
  }
  return pose;
}

void write_pose_file(const std::string & pose_file, const RawPose & pose)
{
  if (pose.ids.empty()) {
    throw std::runtime_error("pose ids must not be empty");
  }
  if (pose.ids.size() != pose.positions_raw.size()) {
    throw std::runtime_error("pose ids length must match positions_raw length");
  }

  ensure_parent_directory(pose_file);

  std::ofstream output(pose_file, std::ios::trunc);
  if (!output) {
    throw std::runtime_error("failed to open pose output: " + pose_file);
  }

  output
    << "pose:\n"
    << "  ids: [" << join_ids(pose.ids) << "]\n"
    << "  positions_raw: [" << join_integers(pose.positions_raw) << "]\n";
}

}  // namespace ftservo_hls3625_teach
