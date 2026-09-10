#pragma once

#include <atomic>
#include <cstddef>
#include <string>
#include <vector>

namespace bms
{

struct ExportOptions
{
  /// Also rewrite the `header.stamp` inside every message that has one.
  /// Without this only the bag's own timestamps move, and any consumer that
  /// works off header stamps (most of them) still sees the original skew.
  bool shift_header_stamps = true;
  /// Storage plugin for the output bag.
  std::string storage_id = "mcap";
};

/// Live counters a UI thread can read while the export runs on a worker.
struct ExportProgress
{
  std::atomic<std::size_t> written{0};
  std::atomic<std::size_t> total{0};
  std::atomic<bool> cancel{false};
};

struct ExportResult
{
  std::string destination;
  std::size_t messages_written = 0;
  std::size_t header_stamps_shifted = 0;
  /// Stamps left alone because they were exactly zero, which by ROS convention
  /// means "unset" rather than "the epoch".
  std::size_t zero_stamps_kept = 0;
  /// Stamps that would have gone negative and were clamped to zero.
  std::size_t clamped_stamps = 0;
  /// Topics whose type begins with a std_msgs/Header, and those that do not.
  std::vector<std::string> header_topics;
  std::vector<std::string> stampless_topics;
  bool cancelled = false;
};

/// Copy every message of the bag at `src_uri` into a new bag at `dst_uri`,
/// shifting all timestamps by `offset_seconds`.
///
/// Throws std::runtime_error if either bag cannot be opened or written.
ExportResult export_shifted_bag(
  const std::string & src_uri,
  const std::string & dst_uri,
  double offset_seconds,
  const ExportOptions & opts,
  ExportProgress * progress = nullptr);

}  // namespace bms
