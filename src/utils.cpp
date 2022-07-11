#include "controller_plugins/utils.hpp"


// Calculate the angle between a line defined by two points and the coordinate axes.
// result is in RADIANS
double angle(const double x1, const double y1, const double x2, const double y2)
{
  if (x2 == x1) 
    return PI / 2;
  
  return atan( (y2-y1) /(x2-x1));
}


// Returns the yaw from a quaternion
// Rather use tf2::getYaw(poses.pose.orientation);
double getYaw(const geometry_msgs::msg::Quaternion & orientation)
{
  double roll, pitch, yaw;
  
  tf2::Quaternion q(
    orientation.x,
    orientation.y,
    orientation.z,
    orientation.w );
  
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  
  return yaw;
}

double getYaw(const geometry_msgs::msg::PoseStamped & pose) {   return getYaw(pose.pose.orientation); }
double getYaw(const geometry_msgs::msg::Pose & pose) {  return getYaw(pose.orientation); }
