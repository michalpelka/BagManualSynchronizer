#pragma once

#include <string>

#include "imu_series.hpp"

namespace bms
{

/// The result of a synchronization session, as written to / read from disk.
///
/// Offset convention: a timestamp `t_b` from bag B lines up with bag A's clock
/// at `t_b + time_offset_seconds`.
struct SyncConfig
{
  std::string bag_a_uri;
  std::string bag_a_topic;
  std::string bag_b_uri;
  std::string bag_b_topic;
  TimestampSource timestamp_source = TimestampSource::HeaderStamp;
  double time_offset_seconds = 0.0;
};

/// Write `cfg` to `path` as YAML. Throws std::runtime_error on failure.
void save_sync_config(const std::string & path, const SyncConfig & cfg);

/// Read a config previously written by save_sync_config().
/// Throws std::runtime_error if the file is missing or malformed.
SyncConfig load_sync_config(const std::string & path);

}  // namespace bms
