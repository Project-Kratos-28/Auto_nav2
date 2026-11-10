#pragma once

#include <algorithm>
#include <cmath>

namespace lidar_angle_filter
{

constexpr double kPi = 3.14159265358979323846;

inline double degrees_to_radians(const double degrees)
{
  return degrees * kPi / 180.0;
}

inline double angular_distance(const double first, const double second)
{
  return std::abs(std::remainder(first - second, 2.0 * kPi));
}

inline bool point_is_masked(
  const double x, const double y, const double front_center_deg,
  const double front_sector_deg, const double back_sector_deg)
{
  if (!std::isfinite(x) || !std::isfinite(y) || (x == 0.0 && y == 0.0)) {
    return false;
  }

  const double azimuth = std::atan2(y, x);
  const double front_center = degrees_to_radians(front_center_deg);
  const double back_center = front_center + kPi;
  constexpr double boundary_tolerance = 1e-12;

  const bool in_front_sector =
    front_sector_deg > 0.0 &&
    angular_distance(azimuth, front_center) <=
    degrees_to_radians(front_sector_deg) * 0.5 + boundary_tolerance;
  const bool in_back_sector =
    back_sector_deg > 0.0 &&
    angular_distance(azimuth, back_center) <=
    degrees_to_radians(back_sector_deg) * 0.5 + boundary_tolerance;
  return in_front_sector || in_back_sector;
}

// Keep only points whose elevation above the sensor's horizontal plane is within
// [min_elevation_deg, max_elevation_deg]. Use it when the sensor is held high and
// everything below its horizontal plane (the person carrying it) must not reach GLIM.
// Non-finite points are left alone: the scan gate already counts them as unusable.
inline bool point_in_elevation_range(
  const double x, const double y, const double z,
  const double min_elevation_deg, const double max_elevation_deg)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    return true;
  }
  const double elevation_deg = std::atan2(z, std::hypot(x, y)) * 180.0 / kPi;
  return elevation_deg >= min_elevation_deg && elevation_deg <= max_elevation_deg;
}

// GLIM aborts the whole process when a scan has no usable points left after its
// own preprocessing (a covered or blocked LiDAR). A scan is only worth passing on
// if enough points are finite and far enough from the sensor to survive that.
inline bool point_is_usable(
  const double x, const double y, const double z, const double min_range)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    return false;
  }
  return x * x + y * y + z * z >= min_range * min_range;
}

inline bool scan_has_enough_points(
  const unsigned long usable_points, const unsigned long min_usable_points)
{
  return usable_points >= min_usable_points;
}

}  // namespace lidar_angle_filter
