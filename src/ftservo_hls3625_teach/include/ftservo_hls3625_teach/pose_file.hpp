#ifndef FTSERVO_HLS3625_TEACH__POSE_FILE_HPP_
#define FTSERVO_HLS3625_TEACH__POSE_FILE_HPP_

#include <string>
#include <vector>

#include "ftservo_hls3625_teach/servo_bus.hpp"

namespace ftservo_hls3625_teach
{

struct RawPose
{
  std::vector<ServoId> ids;
  std::vector<int> positions_raw;
};

RawPose read_pose_file(const std::string & pose_file);
void write_pose_file(const std::string & pose_file, const RawPose & pose);

}  // namespace ftservo_hls3625_teach

#endif  // FTSERVO_HLS3625_TEACH__POSE_FILE_HPP_
