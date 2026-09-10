#pragma once

#include <string>

#include "imu_series.hpp"

namespace bms
{

struct AlignOptions
{
  /// Lags searched around the current offset, in seconds (both directions).
  double search_radius_s = 5.0;
  /// Upper bound on the resampling grid rate; keeps the search affordable.
  double max_rate_hz = 200.0;
  /// Use linear acceleration instead of angular velocity as the sync signal.
  bool use_accel = false;
  /// Channel index: 0..2 for x/y/z, 3 for the magnitude.
  int channel = 3;
};

struct AlignResult
{
  bool ok = false;
  /// Human-readable reason when !ok, or a short summary when ok.
  std::string message;
  /// The estimated offset, replacing (not added to) the previous one.
  double offset_seconds = 0.0;
  /// Peak normalized cross-correlation in [-1, 1]; low values mean the
  /// estimate is not trustworthy (e.g. both sensors were stationary).
  double correlation = 0.0;
  /// Spacing of the resampling grid the search ran on.
  double grid_dt = 0.0;
};

/// Estimate the offset that best aligns `b` onto `a` by normalized cross-
/// correlation of one IMU channel, searching around `initial_offset`.
AlignResult estimate_offset(
  const ImuSeries & a,
  const ImuSeries & b,
  double initial_offset,
  const AlignOptions & opts);

}  // namespace bms
