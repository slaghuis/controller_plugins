// Copyright (c) 2022 Eric Slaghuis
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* *******************************************************
 * Plugin to the navigation_lite controller server
 * Planning algorith: Pure Pursuit 
 *
 * **********************  WARNING  **********************
 * This planner ignores the octomap, and will fly straight
 * into an obstacle.  It assumes that the planner server
 * did a great job!
 * ******************************************************* */
#include <math.h>     // fabs
#include <algorithm>  // std::clamp (C++ 17), find_if

#include "controller_plugins/pure_pursuit_controller.hpp"
#include "controller_plugins/utils.hpp"
#include "navigation_lite/exceptions.hpp"   // nvigation_lite::ControllerException

namespace controller_plugins
{

  void PurePursuitController::configure(const rclcpp::Node::SharedPtr parent, 
                         std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                         std::shared_ptr<octomap::OcTree> costmap )
  {
    node_ = parent;
    plugin_name_ = name;
    tf_ = tf;
    costmap_ = costmap;    
    
    logger_ = node_->get_logger();
    clock_ = node_->get_clock();
    
    double control_frequency = 20.0;
    
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(2.0));     // Maximum horizonral speed in m/s
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(3.5));
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(1.5));
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(4.0));
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.5));
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".rotate_to_heading_angular_vel", rclcpp::ParameterValue(1.8));    // Maximum yaw speed i radians/s
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".use_velocity_scaled_lookahead_dist", rclcpp::ParameterValue(false));
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".goal_dist_tol", rclcpp::ParameterValue(0.20));       // Distance (m) considered close enough to a waypoint
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".yaw_control_limit", rclcpp::ParameterValue(0.40));   // Distance (m) where yaw control and velocity is changes to x and y velocity
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".goal_yaw_tol", rclcpp::ParameterValue(0.020));       // Angle (radians) when yaw control is close enough
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".max_linear_accel", rclcpp::ParameterValue(0.3));     // Maximum acceleration in m/s/s
    
    node_->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
    node_->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
    node_->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
    node_->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
    node_->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
    node_->get_parameter(
      plugin_name_ + ".rotate_to_heading_angular_vel",
      rotate_to_heading_angular_vel_);
    node_->get_parameter(
      plugin_name_ + ".use_velocity_scaled_lookahead_dist",
      use_velocity_scaled_lookahead_dist_);
    node_->get_parameter("controller_frequency", control_frequency);
    
    node_->get_parameter(plugin_name_ + ".goal_dist_tol", goal_dist_tol_);
    node_->get_parameter(plugin_name_ + ".yaw_control_limit", yaw_control_limit_);
    node_->get_parameter(plugin_name_ + ".goal_yaw_tol", goal_yaw_tol_);
    node_->get_parameter(plugin_name_ + ".max_linear_accel", max_linear_accel_);
        
    rotate_to_heading_angular_vel_ = 0.5;
    control_duration_ = 1.0 / control_frequency;
    
    pid_x   = std::make_shared<PID>(control_duration_ , desired_linear_vel_, -desired_linear_vel_, 0.7, 0.0, 0.0);
    pid_y   = std::make_shared<PID>(control_duration_ , desired_linear_vel_, -desired_linear_vel_, 0.7, 0.0, 0.0);
    pid_z   = std::make_shared<PID>(control_duration_ , desired_linear_vel_, -desired_linear_vel_, 0.7, 0.0, 0.0);
    pid_yaw = std::make_shared<PID>(control_duration_ , desired_linear_vel_, -desired_linear_vel_, 0.7, 0.0, 0.0);

    last_v_x = 0.0;
    last_v_y = 0.0;
  }
                       
  void PurePursuitController::setPath(const nav_msgs::msg::Path & path)
  {
    global_plan_ = path;    
  }
    
  geometry_msgs::msg::TwistStamped PurePursuitController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed)
  {
    double lookahead_dist = getLookAheadDistance(speed);
    RCLCPP_DEBUG(logger_, "Lookahead distance %.2f", lookahead_dist);
    auto goal_pose = getLookAheadPoint(lookahead_dist, pose);
    RCLCPP_DEBUG(logger_, "Current pose [%.2f,%.2f,%.2f]", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    RCLCPP_DEBUG(logger_, "Goal pose [%.2f,%.2f,%.2f]", goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z);
        
    geometry_msgs::msg::TwistStamped setpoint = geometry_msgs::msg::TwistStamped();
    setpoint.header.frame_id = "base_link";
    setpoint.header.stamp = clock_->now();
    setpoint.twist.linear.x = 0.0;
    setpoint.twist.linear.y = 0.0;
    setpoint.twist.linear.z = 0.0;
    setpoint.twist.angular.x = 0.0;
    setpoint.twist.angular.y = 0.0;
    setpoint.twist.angular.z = 0.0;
    
    double err_x, err_y, err_z;
    err_x = goal_pose.pose.position.x - pose.pose.position.x;
    err_y = goal_pose.pose.position.y - pose.pose.position.y;
    err_z = goal_pose.pose.position.z - pose.pose.position.z;
    double err_dist = std::hypot(err_x, err_y);
    double yaw_to_target = atan2(err_y, err_x);
    double yaw_error = getDiff2Angles(yaw_to_target, getYaw(pose), PI);
    
    // Here we go:                                                          THE FLIGHT STRATEGY:
    if (err_dist < goal_dist_tol_) {                                        // If we are on the x, y mark of the last waypoint, then
      RCLCPP_DEBUG(logger_, "Mode 1 - Err Z = %.2f", err_z);
      setpoint.twist.linear.z = pid_z->calculate(0, -err_z);                  //   adjust the altitude
      double yaw_error = getDiff2Angles(getYaw(goal_pose), getYaw(pose), PI);
      setpoint.twist.angular.z = pid_yaw->calculate(0.0, - yaw_error);        //   and adjust yaw to point as per the last waypoint
    } else if (err_dist < yaw_control_limit_) {                            // If we are close in the XY plane, but not quite there, then
      RCLCPP_DEBUG(logger_, "Mode 2");
      setpoint.twist.linear.z = pid_z->calculate(0, -err_z);                  //   adjust the altitude
      setpoint.twist.linear.x = pid_x->calculate(0, -err_x);                  //   and tune the position using X and Y PID controllers.
      setpoint.twist.linear.y = pid_y->calculate(0, -err_y); 
    } else if (fabs(yaw_error) > goal_yaw_tol_) {                      // If we are far far away, and pointing wrong, then    
      RCLCPP_DEBUG(logger_, "Mode 3: ytt=%.4f > goal_yaw_tol_ =%.4f", fabs(yaw_to_target), goal_yaw_tol_);
      setpoint.twist.angular.z = pid_yaw->calculate(0.0, -yaw_error);       //   adjust yaw to point to the next waypoint
    } else {
      RCLCPP_DEBUG(logger_, "Mode 4 - Actual flight");                     // Otherwise the yaw is good,
      setpoint.twist.angular.z = pid_yaw->calculate(0.0, - yaw_error);      //   correct the yaw (fine tuning)
      setpoint.twist.linear.x = pid_x->calculate(0, -err_dist);             //   increase foreward thrust to get closer to the target  
      setpoint.twist.linear.z = pid_z->calculate(0, -err_z);                //   adjust the altitude to get closer to the target
    }

    // Govern acceleration, and decellaration.  The latter should be governed by a well 
    // tuned PID but then not all control is done via a PID.  Often velocity is forced
    // to 0 which is an abrupt stop.
    // Could use the speed that was passed as a parameter, but that seems no to work that well
    if (setpoint.twist.linear.x > last_v_x ) {  // Acceleration
      setpoint.twist.linear.x = min(setpoint.twist.linear.x, last_v_x + (max_linear_accel_ * control_duration_));
    } else {                              // Decelleration
      setpoint.twist.linear.x = max(setpoint.twist.linear.x, last_v_x - (max_linear_accel_ * control_duration_));
    }
    if (setpoint.twist.linear.y > last_v_y ) {
      setpoint.twist.linear.y = min(setpoint.twist.linear.y, last_v_y + (max_linear_accel_ * control_duration_));
    } else {
      setpoint.twist.linear.y = max(setpoint.twist.linear.y, last_v_y - (max_linear_accel_ * control_duration_));
    }
    last_v_x = setpoint.twist.linear.x;
    last_v_y = setpoint.twist.linear.y;
    
    return setpoint;
  }
  
  double PurePursuitController::getLookAheadDistance(
    const geometry_msgs::msg::Twist & speed)
  {
    // If using velocity-scaled look ahead distances, find and clamp the dist
    // Else, use the static look ahead distance
    double lookahead_dist = lookahead_dist_;
    if (use_velocity_scaled_lookahead_dist_) {
      lookahead_dist = max(fabs(speed.linear.x), fabs(speed.linear.z)) * lookahead_time_;
      lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
    }

    return lookahead_dist;
  }
  
  geometry_msgs::msg::PoseStamped PurePursuitController::getLookAheadPoint(
    const double & lookahead_dist,
    const geometry_msgs::msg::PoseStamped current_pose)
  {
    // Find the first pose which is at a distance greater than the lookahead distance
    auto goal_pose_it = std::find_if(
      global_plan_.poses.begin(), global_plan_.poses.end(), [&](const auto & ps) {
        return (euclidean_distance(ps, current_pose) >= lookahead_dist);
      });

    // If the no pose is not far enough, take the last pose
    if (goal_pose_it == global_plan_.poses.end()) {
      goal_pose_it = std::prev(global_plan_.poses.end());
    } 
    // RCLCPP_INFO(logger_, "Goal pose it [%.2f,%.2f,%.2f]", goal_pose_it->pose.position.x, goal_pose_it->pose.position.y,goal_pose_it->pose.position.z);
    
    /*  use interpolation to calculate a point in 3D space that is on the way to the goal psoe
      else if (use_interpolation_ && goal_pose_it != transformed_plan.poses.begin()) {
      // Find the point on the line segment between the two poses
      // that is exactly the lookahead distance away from the robot pose (the origin)
      // This can be found with a closed form for the intersection of a segment and a circle
      // Because of the way we did the std::find_if, prev_pose is guaranteed to be inside the circle,
      // and goal_pose is guaranteed to be outside the circle.
      auto prev_pose_it = std::prev(goal_pose_it);
      auto point = circleSegmentIntersection(
        prev_pose_it->pose.position,
        goal_pose_it->pose.position, lookahead_dist);
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = prev_pose_it->header.frame_id;
      pose.header.stamp = goal_pose_it->header.stamp;
      pose.pose.position = point;
      return pose;
    } */  

    // Prune the global path, from waypoints that we have already passed.
    global_plan_.poses.erase(begin(global_plan_.poses), goal_pose_it);
    
    return *goal_pose_it;
  }

}  // namespace controller_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(controller_plugins::PurePursuitController, navigation_lite::Controller)
