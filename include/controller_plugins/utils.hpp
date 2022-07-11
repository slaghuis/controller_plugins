#ifndef NAVIGATION_LITE_UTILS_HPP_
#define NAVIGATION_LITE_UTILS_HPP_

#include <cmath> // fabs, sqrt, pow
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/path.hpp"


const double PI = 3.14159;

// In C the modulo operation returns a value with the same sign as the dividend.
// Hence a custom modulo function
inline double modulo(const double a, const double n) {
  return a - floor(a/n) * n;
}

// Returns the difference between two angles x and y as a number 
// between -180 and 180.  c can be PI for radians, or 180 for degrees. 
inline double getDiff2Angles(const double x, const double y, const double c)
{
  double a = x-y;
  return modulo( a+c, 2*c) - c;
}

// Calculate the angle between a line defined by two points and the coordinate axes.
// result is in RADIANS
double angle(const double x1, const double y1, const double x2, const double y2);

/**
 * @brief Get the L2 distance between 2 geometry_msgs::Poses
 * @param pos1 First pose
 * @param pos1 Second pose
 * @param is_3d True if a true L2 distance is desired (default false)
 * @return double euclidean distance
 */
inline double euclidean_distance(
  const geometry_msgs::msg::Pose & pos1,
  const geometry_msgs::msg::Pose & pos2,
  const bool is_3d = true)
{
  double dx = pos1.position.x - pos2.position.x;
  double dy = pos1.position.y - pos2.position.y;

  if (is_3d) {
    double dz = pos1.position.z - pos2.position.z;
    return std::hypot(dx, dy, dz);
  }

  return std::hypot(dx, dy);
}

/**
 * @brief Get the L2 distance between 2 geometry_msgs::PoseStamped
 * @param pos1 First pose
 * @param pos1 Second pose
 * @param is_3d True if a true L2 distance is desired (default false)
 * @return double L2 distance
 */
inline double euclidean_distance(
  const geometry_msgs::msg::PoseStamped & pos1,
  const geometry_msgs::msg::PoseStamped & pos2,
  const bool is_3d = true)
{
  return euclidean_distance(pos1.pose, pos2.pose, is_3d);
}

// Returns the yaw from a quaternion
double getYaw(const geometry_msgs::msg::Quaternion & orientation);
double getYaw(const geometry_msgs::msg::PoseStamped & pose);
double getYaw(const geometry_msgs::msg::Pose & pose);

inline double rad_to_deg(const double rad) { return (rad * 180.0) / PI; }
inline double deg_to_rad(const double deg) { return (deg * PI) / 180.0; }

template<typename NodeT>
void declare_parameter_if_not_declared(
  NodeT node,
  const std::string & param_name,
  const rclcpp::ParameterValue & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
  rcl_interfaces::msg::ParameterDescriptor())
{
  if (!node->has_parameter(param_name)) {
    node->declare_parameter(param_name, default_value, parameter_descriptor);
  }
}

/**
 * Find first element in iterator that is greater integrated distance than comparevalue
 */
template<typename Iter, typename Getter>
inline Iter first_after_integrated_distance(Iter begin, Iter end, Getter getCompareVal)
{
  if (begin == end) {
    return end;
  }
  Getter dist = 0.0;
  for (Iter it = begin; it != end - 1; it++) {
    dist += euclidean_distance(*it, *(it + 1));
    if (dist > getCompareVal) {
      return it + 1;
    }
  }
  return end;
}

/**
 * @brief Calculate the length of the provided path, starting at the provided index
 * @param path Path containing the poses that are planned
 * @param start_index Optional argument specifying the starting index for
 * the calculation of path length. Provide this if you want to calculate length of a
 * subset of the path.
 * @return double Path length
 */
inline double calculate_path_length(const nav_msgs::msg::Path & path, size_t start_index = 0)
{
  if (start_index + 1 >= path.poses.size()) {
    return 0.0;
  }
  double path_length = 0.0;
  for (size_t idx = start_index; idx < path.poses.size() - 1; ++idx) {
    path_length += euclidean_distance(path.poses[idx].pose, path.poses[idx + 1].pose);
  }
  return path_length;
}

/**
 * Find element in iterator with the minimum calculated value
 */
template<typename Iter, typename Getter>
inline Iter min_by(Iter begin, Iter end, Getter getCompareVal)
{
  if (begin == end) {
    return end;
  }
  auto lowest = getCompareVal(*begin);
  Iter lowest_it = begin;
  for (Iter it = ++begin; it != end; ++it) {
    auto comp = getCompareVal(*it);
    if (comp < lowest) {
      lowest = comp;
      lowest_it = it;
    }
  }
  return lowest_it;
}

#endif // NAVIGATION_LITE__UTILS_HPP_