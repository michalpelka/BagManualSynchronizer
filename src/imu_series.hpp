#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace bms
{

/// Which clock the sample timestamps are taken from.
enum class TimestampSource
{
  HeaderStamp,   ///< sensor_msgs::msg::Imu::header::stamp (sensor clock)
  BagReceive,    ///< rosbag2 recv_timestamp (recording clock)
};

const char * to_string(TimestampSource src);

/// One IMU topic read out of one bag, stored as structure-of-arrays so the
/// component vectors can be handed to ImPlot without a copy.
struct ImuSeries
{
  std::string topic;
  TimestampSource source = TimestampSource::HeaderStamp;

  /// Absolute time in seconds since the epoch, monotonically increasing.
  std::vector<double> t;
  /// Angular velocity [rad/s], indexed [axis][sample]; axis 3 is the magnitude.
  std::array<std::vector<double>, 4> gyro;
  /// Linear acceleration [m/s^2], indexed [axis][sample]; axis 3 is the magnitude.
  std::array<std::vector<double>, 4> accel;

  /// Number of messages whose header stamp was zero/unset (header source only).
  std::size_t invalid_stamps = 0;
  /// Number of messages that had to be dropped to keep `t` sorted.
  std::size_t out_of_order = 0;

  bool empty() const {return t.empty();}
  std::size_t size() const {return t.size();}

  double t_begin() const {return t.empty() ? 0.0 : t.front();}
  double t_end() const {return t.empty() ? 0.0 : t.back();}
  double duration() const {return t.empty() ? 0.0 : t.back() - t.front();}

  /// Messages per second averaged over the whole series, 0 if undefined.
  double mean_rate() const
  {
    const double d = duration();
    return d > 0.0 ? static_cast<double>(t.size() - 1) / d : 0.0;
  }
};

/// Everything known about one of the two bags under comparison.
struct BagData
{
  std::string uri;
  /// All `sensor_msgs/msg/Imu` topics found in the bag, in bag order.
  std::vector<std::string> imu_topics;
  /// The topic currently loaded into `series`.
  ImuSeries series;
};

/// Axis labels used by both plots; index 3 is the magnitude channel.
inline constexpr std::array<const char *, 4> kAxisNames{"x", "y", "z", "mag"};

}  // namespace bms
