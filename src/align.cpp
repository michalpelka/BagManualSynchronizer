#include "align.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace bms
{
namespace
{

/// Resample an irregularly sampled channel onto `n` points spaced `dt` apart
/// starting at `t0`, by linear interpolation. Assumes `t` is sorted.
std::vector<double> resample(
  const std::vector<double> & t,
  const std::vector<double> & v,
  double t0,
  double dt,
  std::size_t n)
{
  std::vector<double> out(n, 0.0);
  std::size_t k = 0;
  for (std::size_t i = 0; i < n; ++i) {
    const double ti = t0 + static_cast<double>(i) * dt;
    while (k + 2 < t.size() && t[k + 1] < ti) {
      ++k;
    }
    const double t_lo = t[k];
    const double t_hi = t[k + 1];
    const double span = t_hi - t_lo;
    // Duplicate stamps would make the ratio infinite; fall back to the left
    // sample, which is what a zero-width interval means anyway.
    const double u = span > 0.0 ? (ti - t_lo) / span : 0.0;
    out[i] = v[k] + (v[k + 1] - v[k]) * std::clamp(u, 0.0, 1.0);
  }
  return out;
}

/// Sub-sample peak position from three correlation values around the maximum.
double parabolic_peak(double y_prev, double y_peak, double y_next)
{
  const double denom = y_prev - 2.0 * y_peak + y_next;
  if (std::abs(denom) < 1e-12) {
    return 0.0;
  }
  return std::clamp(0.5 * (y_prev - y_next) / denom, -1.0, 1.0);
}

}  // namespace

AlignResult estimate_offset(
  const ImuSeries & a,
  const ImuSeries & b,
  double initial_offset,
  const AlignOptions & opts)
{
  AlignResult result;
  result.offset_seconds = initial_offset;

  if (a.size() < 2 || b.size() < 2) {
    result.message = "both bags need at least two IMU samples";
    return result;
  }

  const int ch = std::clamp(opts.channel, 0, 3);
  const auto & va = opts.use_accel ? a.accel[ch] : a.gyro[ch];
  const auto & vb = opts.use_accel ? b.accel[ch] : b.gyro[ch];

  const double rate = std::min({a.mean_rate(), b.mean_rate(), opts.max_rate_hz});
  if (!(rate > 0.0)) {
    result.message = "cannot determine a sample rate for the bags";
    return result;
  }
  const double dt = 1.0 / rate;
  result.grid_dt = dt;

  // Grid A over its own span; grid B over its span already shifted by the
  // offset the user has dialled in, so the search is relative to that.
  const double a0 = a.t_begin();
  const double b0 = b.t_begin() + initial_offset;
  const auto na = static_cast<std::size_t>(a.duration() / dt) + 1;
  const auto nb = static_cast<std::size_t>(b.duration() / dt) + 1;

  const std::vector<double> ga = resample(a.t, va, a0, dt, na);
  std::vector<double> tb_shifted = b.t;
  for (double & x : tb_shifted) {
    x += initial_offset;
  }
  const std::vector<double> gb = resample(tb_shifted, vb, b0, dt, nb);

  // b's grid point j sits at the same time as a's grid point i = j + d + lag.
  // The two grids rarely line up exactly, and rounding d to an integer hides a
  // sub-sample shift of (d - d_exact) * dt that has to be added back to the
  // result -- otherwise every estimate is biased by up to half a grid step.
  const double d_exact = (b0 - a0) / dt;
  const auto d = static_cast<long>(std::llround(d_exact));
  const double grid_residual = static_cast<double>(d) - d_exact;
  const auto lag_max = static_cast<long>(std::llround(opts.search_radius_s / dt));
  if (lag_max < 1) {
    result.message = "search radius is smaller than one sample";
    return result;
  }

  double best_corr = -2.0;
  long best_lag = 0;
  std::vector<double> corr(static_cast<std::size_t>(2 * lag_max + 1), -2.0);

  for (long lag = -lag_max; lag <= lag_max; ++lag) {
    // Valid i satisfies 0 <= i < na and 0 <= i - lag - d < nb.
    const long i_begin = std::max<long>(0, lag + d);
    const long i_end = std::min<long>(static_cast<long>(na), static_cast<long>(nb) + lag + d);
    const long n = i_end - i_begin;
    if (n < 16) {
      continue;   // too little overlap to mean anything
    }

    double sa = 0.0, sb = 0.0, saa = 0.0, sbb = 0.0, sab = 0.0;
    for (long i = i_begin; i < i_end; ++i) {
      const double x = ga[static_cast<std::size_t>(i)];
      const double y = gb[static_cast<std::size_t>(i - lag - d)];
      sa += x;
      sb += y;
      saa += x * x;
      sbb += y * y;
      sab += x * y;
    }
    const double dn = static_cast<double>(n);
    const double cov = sab - sa * sb / dn;
    const double var_a = saa - sa * sa / dn;
    const double var_b = sbb - sb * sb / dn;
    if (var_a <= 0.0 || var_b <= 0.0) {
      continue;   // a constant window carries no timing information
    }
    const double c = cov / std::sqrt(var_a * var_b);

    corr[static_cast<std::size_t>(lag + lag_max)] = c;
    if (c > best_corr) {
      best_corr = c;
      best_lag = lag;
    }
  }

  if (best_corr < -1.5) {
    result.message = "bags do not overlap anywhere in the search range";
    return result;
  }

  // Refine to sub-sample resolution when both neighbours were evaluated.
  double refined = static_cast<double>(best_lag);
  const auto idx = static_cast<std::size_t>(best_lag + lag_max);
  if (best_lag > -lag_max && best_lag < lag_max &&
    corr[idx - 1] > -1.5 && corr[idx + 1] > -1.5)
  {
    refined += parabolic_peak(corr[idx - 1], corr[idx], corr[idx + 1]);
  }

  // A positive lag means b's grid had to move later to match a, and b's grid
  // was already shifted by initial_offset.
  result.ok = true;
  result.offset_seconds = initial_offset + (refined + grid_residual) * dt;
  result.correlation = best_corr;
  result.message = "peak correlation " + std::to_string(best_corr);
  return result;
}

}  // namespace bms
