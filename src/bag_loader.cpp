#include "bag_loader.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace bms
{
namespace
{

/// Nanoseconds -> seconds, done in double only after the integer subtraction
/// would be lossy anyway; the absolute epoch value costs ~1 us of resolution
/// at double precision, which is far below IMU jitter.
constexpr double kNanoToSec = 1e-9;

double header_stamp_seconds(const sensor_msgs::msg::Imu & msg)
{
  return static_cast<double>(msg.header.stamp.sec) +
         static_cast<double>(msg.header.stamp.nanosec) * kNanoToSec;
}

struct Sample
{
  double t;
  double gyro[3];
  double accel[3];
};

}  // namespace

const char * to_string(TimestampSource src)
{
  switch (src) {
    case TimestampSource::HeaderStamp:
      return "header stamp";
    case TimestampSource::BagReceive:
      return "bag receive time";
  }
  return "unknown";
}

std::vector<std::string> find_imu_topics(const std::string & uri)
{
  rosbag2_cpp::Reader reader;
  reader.open(uri);

  std::vector<std::string> topics;
  for (const auto & meta : reader.get_all_topics_and_types()) {
    if (meta.type == kImuTypeName) {
      topics.push_back(meta.name);
    }
  }
  reader.close();
  return topics;
}

ImuSeries load_imu_series(
  const std::string & uri,
  const std::string & topic,
  TimestampSource source,
  std::atomic<std::size_t> * progress)
{
  ImuSeries series;
  series.topic = topic;
  series.source = source;

  rosbag2_cpp::Reader reader;
  reader.open(uri);

  rosbag2_storage::StorageFilter filter;
  filter.topics = {topic};
  reader.set_filter(filter);

  rclcpp::Serialization<sensor_msgs::msg::Imu> serialization;
  sensor_msgs::msg::Imu msg;

  std::vector<Sample> samples;
  std::size_t count = 0;

  while (reader.has_next()) {
    const auto bag_msg = reader.read_next();
    // The filter is applied by the storage plugin, but a plugin that ignores
    // it would otherwise feed the wrong type into the deserializer.
    if (bag_msg->topic_name != topic) {
      continue;
    }

    rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
    serialization.deserialize_message(&serialized, &msg);

    double t;
    if (source == TimestampSource::HeaderStamp) {
      t = header_stamp_seconds(msg);
      if (t <= 0.0) {
        ++series.invalid_stamps;
        continue;
      }
    } else {
      t = static_cast<double>(bag_msg->recv_timestamp) * kNanoToSec;
    }

    Sample s;
    s.t = t;
    s.gyro[0] = msg.angular_velocity.x;
    s.gyro[1] = msg.angular_velocity.y;
    s.gyro[2] = msg.angular_velocity.z;
    s.accel[0] = msg.linear_acceleration.x;
    s.accel[1] = msg.linear_acceleration.y;
    s.accel[2] = msg.linear_acceleration.z;
    samples.push_back(s);

    if (progress != nullptr && (++count % 512) == 0) {
      progress->store(count, std::memory_order_relaxed);
    }
  }
  reader.close();

  // A bag can interleave messages out of order (multi-file bags, or a sensor
  // clock that stepped). Sorting keeps interpolation and plotting well defined;
  // the count is reported so the user knows the input was not clean.
  for (std::size_t i = 1; i < samples.size(); ++i) {
    if (samples[i].t < samples[i - 1].t) {
      ++series.out_of_order;
    }
  }
  if (series.out_of_order > 0) {
    std::stable_sort(
      samples.begin(), samples.end(),
      [](const Sample & a, const Sample & b) {return a.t < b.t;});
  }

  series.t.reserve(samples.size());
  for (int axis = 0; axis < 4; ++axis) {
    series.gyro[axis].reserve(samples.size());
    series.accel[axis].reserve(samples.size());
  }

  for (const auto & s : samples) {
    series.t.push_back(s.t);
    for (int axis = 0; axis < 3; ++axis) {
      series.gyro[axis].push_back(s.gyro[axis]);
      series.accel[axis].push_back(s.accel[axis]);
    }
    series.gyro[3].push_back(
      std::sqrt(s.gyro[0] * s.gyro[0] + s.gyro[1] * s.gyro[1] + s.gyro[2] * s.gyro[2]));
    series.accel[3].push_back(
      std::sqrt(s.accel[0] * s.accel[0] + s.accel[1] * s.accel[1] + s.accel[2] * s.accel[2]));
  }

  if (progress != nullptr) {
    progress->store(count, std::memory_order_relaxed);
  }
  return series;
}

}  // namespace bms
