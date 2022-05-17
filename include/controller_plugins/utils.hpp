#ifndef NAVIGATION_LITE__UTILS_HPP_
#define NAVIGATION_LITE__UTILS_HPP_

const double PI = 3.14159;
  
// Calculate the difference between two angles   
inline double getDiff2Angles(const double x, const double y, const double c)
{
  // c can be PI (for radians) or 180.0 (for degrees);
  double d =  fabs(fmod(fabs(x - y), 2*c));
  double r = d > c ? c*2 - d : d;
  
  double sign = ((x-y >= 0.0) && (x-y <= c)) || ((x-y <= -c) && (x-y> -2*c)) ? 1.0 : -1.0;
  return sign * r;                                           

}

// Calculate the angle between a line defined by two points and the coordinate axes.
inline double angle(const double x1, double y1, double x2, double y2)
{
  if (x2 == x1) 
    return PI / 2;
  
  return atan( (y2-y1) /(x2-x1));
}

double euclidean_distance( const geometry_msgs::msg::PoseStamped & start,
                           const geometry_msgs::msg::PoseStamped & goal)
{
//  return sqrt( pow(goal.pose.position.x - start.pose.position.x, 2) +
//               pow(goal.pose.position.y - start.pose.position.y, 2) +
//               pow(goal.pose.position.z - start.pose.position.z, 2));
  return hypot( goal.pose.position.x - start.pose.position.x,
                goal.pose.position.y - start.pose.position.y,
                goal.pose.position.z - start.pose.position.z );

}

// Rather use tf2::getYaw(poses.pose.orientation);
double getYaw(const geometry_msgs::msg::PoseStamped & pose)
{
  double roll, pitch, yaw;
  
  tf2::Quaternion q(
    pose.pose.orientation.x,
    pose.pose.orientation.y,
    pose.pose.orientation.z,
    pose.pose.orientation.w );
  
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  
  return yaw;
}

double getYaw(const geometry_msgs::msg::Pose & pose)
{
  double roll, pitch, yaw;
  
  tf2::Quaternion q(
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w );
  
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  
  return yaw;
}

inline double rad_to_deg(float rad) { return (rad * 180.0) / PI; }
inline double deg_to_rad(float deg) { return (deg * PI) / 180.0; }

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

#endif // NAVIGATION_LITE__UTILS_HPP_