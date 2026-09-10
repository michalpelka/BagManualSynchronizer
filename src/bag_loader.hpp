#pragma once

#include <atomic>
#include <cstddef>
#include <string>
#include <vector>

#include "imu_series.hpp"

namespace bms
{

/// ROS 2 type name this tool plots.
inline constexpr const char * kImuTypeName = "sensor_msgs/msg/Imu";

/// Open `uri` and return the names of every `sensor_msgs/msg/Imu` topic in it.
/// Throws std::runtime_error if the bag cannot be opened.
std::vector<std::string> find_imu_topics(const std::string & uri);

/// Read every message of `topic` from the bag at `uri` into an ImuSeries.
///
/// `progress` (optional) is updated with the number of messages read so far so
/// a UI thread can show progress while this runs on a worker.
/// Throws std::runtime_error if the bag or topic cannot be read.
ImuSeries load_imu_series(
  const std::string & uri,
  const std::string & topic,
  TimestampSource source,
  std::atomic<std::size_t> * progress = nullptr);

}  // namespace bms
