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
 * Planning algorith: 3DVHF+ 
 *
 * ******************************************************* */
#include <math.h>     // fabs
#include <algorithm>  // std::clamp (C++ 17), find_if
#include <iomanip>
#include <sstream>
#include <iostream>

#include "controller_plugins/vhf_plus_controller.hpp"
#include "controller_plugins/utils.hpp"
#include "controller_plugins/histogram.hpp"
#include "navigation_lite/exceptions.hpp"   // navigation_lite::ControllerException
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// Costants for path weights as per Ulrich et al.
const double MU1 = 5;   // Target angle vs Candidate direction
const double MU2 = 2;   // Rotation Theta of the robot and the candidate direction
const double MU3 = 2;   // Previous delected direction and candidate direction

namespace controller_plugins
{

  void VhfPlusController::configure(const rclcpp::Node::SharedPtr parent, 
                         std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                         std::shared_ptr<octomap::OcTree> costmap )
  {
    node_ = parent;
    plugin_name_ = name;
    tf_ = tf;
    costmap_ = costmap;    
    
    logger_ = node_->get_logger();
    clock_ = node_->get_clock();
    
    double transform_tolerance = 0.1;
    double control_frequency = 20.0;
    
    goal_dist_tol_ = 0.25;  // reasonable default before first update

    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(2.0));              // Maximum horisontal speed, in m/s
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(2.5));
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.3));
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(5.5));
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.5));
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".rotate_to_heading_angular_vel", rclcpp::ParameterValue(0.33));  // Maximum horisontal speed, in m/s
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.2));  // 0.1
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".use_velocity_scaled_lookahead_dist",
      rclcpp::ParameterValue(false));
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".min_approach_linear_velocity", rclcpp::ParameterValue(0.05));
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".min_approach_linear_distance", rclcpp::ParameterValue(0.05));
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".max_allowed_time_to_collision_up_to_carrot",
      rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".use_regulated_linear_velocity_scaling", rclcpp::ParameterValue(true));
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".use_cost_regulated_linear_velocity_scaling",
      rclcpp::ParameterValue(true));
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".cost_scaling_dist", rclcpp::ParameterValue(0.6));
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".cost_scaling_gain", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".inflation_cost_scaling_factor", rclcpp::ParameterValue(3.0));
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".regulated_linear_scaling_min_radius", rclcpp::ParameterValue(0.90));
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".regulated_linear_scaling_min_speed", rclcpp::ParameterValue(0.25));
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".use_rotate_to_heading", rclcpp::ParameterValue(true));
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".rotate_to_heading_min_angle", rclcpp::ParameterValue(0.025));   // Acceptible YAW to start foreward acceleration (radians)
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".rotate_to_heading_min_distance", rclcpp::ParameterValue(1.0));  // Minimum distance from target that rotate to heading will be used.  Start XY positioning
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".max_angular_accel", rclcpp::ParameterValue(3.2));
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".allow_reversing", rclcpp::ParameterValue(false));
//    declare_parameter_if_not_declared(
//      node_, plugin_name_ + ".max_robot_pose_search_dist",
//      rclcpp::ParameterValue(getCostmapMaxExtent()));
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".use_interpolation",
      rclcpp::ParameterValue(true));    
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".robot_radius", rclcpp::ParameterValue(0.5));         // Radius (m) of the sphere the robot will fit in
    declare_parameter_if_not_declared(
      node_, plugin_name_ + ".safety_radius", rclcpp::ParameterValue(0.5));        // Safety margin (m) around obstacles
    
    node_->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
      base_desired_linear_vel_ = desired_linear_vel_;
    
    node_->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
    node_->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
    node_->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
    node_->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
    node_->get_parameter(
      plugin_name_ + ".rotate_to_heading_angular_vel",
      rotate_to_heading_angular_vel_);
    node_->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
    node_->get_parameter(
      plugin_name_ + ".use_velocity_scaled_lookahead_dist",
      use_velocity_scaled_lookahead_dist_);
    node_->get_parameter(
      plugin_name_ + ".min_approach_linear_velocity",
      min_approach_linear_velocity_);
    node_->get_parameter(
      plugin_name_ + ".min_approach_linear_distance",
      min_approach_linear_distance_);
    node_->get_parameter(
      plugin_name_ + ".max_allowed_time_to_collision_up_to_carrot",
      max_allowed_time_to_collision_up_to_carrot_);
    node_->get_parameter(
      plugin_name_ + ".use_regulated_linear_velocity_scaling",
      use_regulated_linear_velocity_scaling_);
    node_->get_parameter(
      plugin_name_ + ".use_cost_regulated_linear_velocity_scaling",
      use_cost_regulated_linear_velocity_scaling_);
    node_->get_parameter(plugin_name_ + ".cost_scaling_dist", cost_scaling_dist_);
    node_->get_parameter(plugin_name_ + ".cost_scaling_gain", cost_scaling_gain_);
    node_->get_parameter(
      plugin_name_ + ".inflation_cost_scaling_factor",
      inflation_cost_scaling_factor_);
    node_->get_parameter(
      plugin_name_ + ".regulated_linear_scaling_min_radius",
      regulated_linear_scaling_min_radius_);
    node_->get_parameter(
      plugin_name_ + ".regulated_linear_scaling_min_speed",
      regulated_linear_scaling_min_speed_);
    node_->get_parameter(plugin_name_ + ".use_rotate_to_heading", use_rotate_to_heading_);
    node_->get_parameter(plugin_name_ + ".rotate_to_heading_min_angle", rotate_to_heading_min_angle_);
    node_->get_parameter(plugin_name_ + ".rotate_to_heading_min_distance", rotate_to_heading_min_distance_);
    node_->get_parameter(plugin_name_ + ".max_angular_accel", max_angular_accel_);
    node_->get_parameter(plugin_name_ + ".allow_reversing", allow_reversing_);
    node_->get_parameter("controller_frequency", control_frequency);
//    node_->get_parameter(
//      plugin_name_ + ".max_robot_pose_search_dist",
//      max_robot_pose_search_dist_); 
    node_->get_parameter(
      plugin_name_ + ".use_interpolation",
      use_interpolation_);
    
    node_->get_parameter(plugin_name_ + ".robot_radius", robot_radius_);
    node_->get_parameter(plugin_name_ + ".safety_radius", safety_radius_);
    
    base_frame_id_ = "base_link";
    
    transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
    control_duration_ = 1.0 / control_frequency;

    node_->declare_parameter(plugin_name_ + ".pid_xy", std::vector<double>{0.7, 0.0, 0.0});
    rclcpp::Parameter pid_xy_settings_param = node_->get_parameter(plugin_name_ + ".pid_xy");
    std::vector<double> pid_xy_settings = pid_xy_settings_param.as_double_array(); 
    pid_x   = std::make_shared<PID>(control_duration_ , base_desired_linear_vel_, -base_desired_linear_vel_, (float)pid_xy_settings[0], (float)pid_xy_settings[1], (float)pid_xy_settings[2]);
    pid_y   = std::make_shared<PID>(control_duration_ , base_desired_linear_vel_, -base_desired_linear_vel_, (float)pid_xy_settings[0], (float)pid_xy_settings[1], (float)pid_xy_settings[2]);
    
    node_->declare_parameter(plugin_name_ + ".pid_z", std::vector<double>{0.7, 0.0, 0.0});
    rclcpp::Parameter pid_z_settings_param = node_->get_parameter(plugin_name_ + ".pid_z");
    std::vector<double> pid_z_settings = pid_z_settings_param.as_double_array(); 
    pid_z   = std::make_shared<PID>(control_duration_, base_desired_linear_vel_, -base_desired_linear_vel_, (float)pid_z_settings[0], (float)pid_z_settings[1], (float)pid_z_settings[2]);
    
    node_->declare_parameter(plugin_name_ + ".pid_yaw", std::vector<double>{0.7, 0.0, 0.0});  
    rclcpp::Parameter pid_yaw_settings_param = node_->get_parameter(plugin_name_ + ".pid_yaw");
    std::vector<double> pid_yaw_settings = pid_yaw_settings_param.as_double_array(); 
    pid_yaw   = std::make_shared<PID>(control_duration_, rotate_to_heading_angular_vel_, -rotate_to_heading_angular_vel_, (float)pid_yaw_settings[0], (float)pid_yaw_settings[1], (float)pid_yaw_settings[2]);

    last_v_x = 0.0;
    last_v_y = 0.0;
    
    last_e_angle_ = 0.0;
    last_z_angle_ = 0.0;
  }

  void VhfPlusController::setPath(const nav_msgs::msg::Path & path)
  {
    global_plan_ = path;    
  }
    
  geometry_msgs::msg::TwistStamped VhfPlusController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::Twist & speed)
  {

   RCLCPP_DEBUG(logger_, "CURRENT Pose is %.2f, %.2f, %.2f with yaw %.3f", 
               current_pose.pose.position.x,
               current_pose.pose.position.y,
               current_pose.pose.position.z, 
               getYaw(current_pose));
    
    // Set a logical lookahead distance
    double lookahead_dist = getLookAheadDistance(speed);
    RCLCPP_DEBUG(logger_, "Lookahead distance is %.2f", lookahead_dist);
           
    // This applies the 3DVFH+ algorithm for local planning
    // The result is in the map frame
    auto goal_pose = getLookAheadPoint(lookahead_dist, current_pose);   
           
    RCLCPP_DEBUG(logger_, "GOAL___ Pose is %.2f, %.2f, %.2f with yaw %.3f", 
               goal_pose.pose.position.x,
               goal_pose.pose.position.y,
               goal_pose.pose.position.z, 
               getYaw(goal_pose));

    // Transform the goal_pose to the base_link frame 
    geometry_msgs::msg::PoseStamped carrot_pose;
    transformPose("base_link", goal_pose, carrot_pose);
    
    double linear_vel, angular_vel;

    // Find distance^2 to look ahead point (carrot) in robot base frame
    // This is the chord length of the circle
    double carrot_dist2 =
      (carrot_pose.pose.position.x * carrot_pose.pose.position.x) +
      (carrot_pose.pose.position.y * carrot_pose.pose.position.y);

    // Find curvature of circle (k = 1 / R)
    double curvature = 0.0;
    if (carrot_dist2 > 0.001) {
      curvature = 2.0 * carrot_pose.pose.position.y / carrot_dist2;
    }

    // Setting the velocity direction
    double sign = 1.0;
    if (allow_reversing_) {
      sign = carrot_pose.pose.position.x >= 0.0 ? 1.0 : -1.0;
    }

    linear_vel = desired_linear_vel_;

    // Make sure we're in compliance with basic constraints
    double angle_to_heading;
    if (shouldRotateToGoalHeading(carrot_pose)) {
      geometry_msgs::msg::PoseStamped last_pose;
      transformPose("base_link", global_plan_.poses.back(), last_pose);
      double angle_to_goal = getYaw(last_pose);      
      rotateToHeading(linear_vel, angular_vel, angle_to_goal, speed);
    } else if (shouldRotateToPath(carrot_pose, angle_to_heading)) {
      rotateToHeading(linear_vel, angular_vel, angle_to_heading, speed);
    } else {
      applyConstraints(
        fabs(lookahead_dist - sqrt(carrot_dist2)),
        lookahead_dist, curvature, speed,
        linear_vel, sign);

      // Apply curvature to angular velocity after constraining linear velocity
      angular_vel = sign * linear_vel * curvature;
    }

    
    // Now do the same for the vertical position
    double vertical_vel;

    // Find distance^2 to look ahead point (carrot) in robot base frame
    // This is the chord length of the circle
    carrot_dist2 =  (carrot_pose.pose.position.x * carrot_pose.pose.position.x) +
                    (carrot_pose.pose.position.z * carrot_pose.pose.position.z);

    // Find curvature of circle (k = 1 / R)
    curvature = 0.0;
    if (carrot_dist2 > 0.001) {
      curvature = 2.0 * carrot_pose.pose.position.z / carrot_dist2;
    }

    // Setting the velocity direction
    sign = carrot_pose.pose.position.z >= 0.0 ? 1.0 : -1.0;

    vertical_vel = desired_linear_vel_;

    // Make sure we're in compliance with basic constraints
    applyVerticalConstraints(
      fabs(lookahead_dist - sqrt(carrot_dist2)),
      lookahead_dist, curvature, speed,
      vertical_vel, sign);

    // Apply curvature to angular velocity after constraining linear velocity
    vertical_vel = sign * vertical_vel * curvature;

    ////////////////////////////////////////////////////////////////////////
    // populate and return message
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header = current_pose.header;
    cmd_vel.twist.linear.x = linear_vel;
    cmd_vel.twist.linear.z = vertical_vel;
    cmd_vel.twist.angular.z = angular_vel;
    return cmd_vel;
    
    
/*    
    // Prepare an empty data structure
    geometry_msgs::msg::TwistStamped setpoint = geometry_msgs::msg::TwistStamped();
    setpoint.header.frame_id = "base_link";
    setpoint.header.stamp = clock_->now();
    setpoint.twist.linear.x = 0.0;
    setpoint.twist.linear.y = 0.0;
    setpoint.twist.linear.z = 0.0;
    setpoint.twist.angular.x = 0.0;
    setpoint.twist.angular.y = 0.0;
    setpoint.twist.angular.z = 0.0;
    
    // Flight is determined by distance to the target in the XY,
    // distance to the target in the Z and the heading to the target.
    double err_x = goal_pose.pose.position.x - current_pose.pose.position.x;
    double err_y = goal_pose.pose.position.y - current_pose.pose.position.y;
    double err_z = goal_pose.pose.position.z - current_pose.pose.position.z;
    double err_dist = std::hypot(err_x, err_y);
    double yaw_to_target = atan2(err_y, err_x);
    double yaw_error = getDiff2Angles(yaw_to_target, getYaw(current_pose), PI);
    

    
    
    // Here we go:                                                          THE FLIGHT STRATEGY:
    if (err_dist < min_approach_linear_distance_) {                        // If we are on the x, y mark of the last waypoint, then
      RCLCPP_INFO(logger_, "Mode 1 - We are there, just tune ALTITUDE and YAW Err Z = %.2f", err_z);
      setpoint.twist.linear.z = pid_z->calculate(0, - err_z);                  //   adjust the altitude
      setpoint.twist.angular.z = pid_yaw->calculate(0.0, yaw_error);        //   and adjust yaw to point as per the last waypoint
    } else if (err_dist < rotate_to_heading_min_distance_) {               // If we are close in the XY plane, but not quite there, then
      RCLCPP_INFO(logger_, "Mode 2 - We are close, Fix XYZ");
      setpoint.twist.linear.z = pid_z->calculate(0, -err_z);                  //   adjust the altitude
      setpoint.twist.linear.x = pid_x->calculate(0, err_x);                  //   and tune the position using X and Y PID controllers.
      setpoint.twist.linear.y = pid_y->calculate(0, err_y); 
    } else if (fabs(yaw_error) > rotate_to_heading_min_angle_) {           // If we are far far away, and pointing wrong, then    
      RCLCPP_INFO(logger_, "Mode 3: Far away, pointing wrong yaw_error=%.4f > goal_yaw_tol_ =%.4f", fabs(yaw_error), rotate_to_heading_min_angle_);
      setpoint.twist.angular.z = pid_yaw->calculate(0.0, yaw_error);        //   adjust yaw to point to the next waypoint
    } else {
      RCLCPP_INFO(logger_, "Mode 4 - Actual flight");                      // Otherwise the yaw is good,
      setpoint.twist.angular.z = pid_yaw->calculate(0.0, yaw_error);        //   correct the yaw (fine tuning)
      setpoint.twist.linear.x = pid_x->calculate(0, err_dist);              //   increase forward thrust to get closer to the target  
      setpoint.twist.linear.z = pid_z->calculate(0, -err_z);                 //   adjust the altitude to get closer to the target
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
*/    
    /*
    // Orientation quaternion
    tf2::Quaternion q(
        goal_pose.pose.orientation.x,
        goal_pose.pose.orientation.y,
        goal_pose.pose.orientation.z,
        goal_pose.pose.orientation.w);
        
    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m(q);

    // Roll Pitch and Yaw from rotation matrix
    double roll, pitch, yaw_to_target; 
    m.getRPY(roll, pitch, yaw_to_target);
    
    RCLCPP_INFO(logger_, "Goal Pose is %.2f, %.2f, %.2f with yaw %.3f", 
               goal_pose.pose.position.x,
               goal_pose.pose.position.y,
               goal_pose.pose.position.z, 
               yaw_to_target);
    
    // The goal_pose is in the robot base frame "base_link"
    // Calculate velocities to move the base in this direction
    
    geometry_msgs::msg::TwistStamped setpoint = geometry_msgs::msg::TwistStamped();
    setpoint.header.frame_id = "base_link";
    setpoint.header.stamp = clock_->now();
    // Should I even initialise these?
    setpoint.twist.linear.x = 0.0;
    setpoint.twist.linear.y = 0.0;
    setpoint.twist.linear.z = 0.0;
    setpoint.twist.angular.x = 0.0;
    setpoint.twist.angular.y = 0.0;
    setpoint.twist.angular.z = 0.0;
    
    double err_x, err_y, err_z;
    err_x = goal_pose.pose.position.x; // - current_pose.pose.position.x;
    err_y = goal_pose.pose.position.y; // - current_pose.pose.position.y;
    err_z = goal_pose.pose.position.z; // - current_pose.pose.position.z;
    double err_dist = std::hypot(err_x, err_y);
    //double yaw_to_target = atan2(err_y, err_x);
    //double yaw_error = getDiff2Angles(0, yaw_to_target, PI);
    //double yaw_error = getDiff2Angles(yaw_to_target, getYaw(current_pose), PI);
    double yaw_error = yaw_to_target;
    
    // Here we go:                                                          THE FLIGHT STRATEGY:
    if (err_dist < min_approach_linear_distance_) {                         // If we are on the x, y mark of the last waypoint, then
      RCLCPP_INFO(logger_, "Mode 1 - Err Z = %.2f", err_z);
      setpoint.twist.linear.z = pid_z->calculate(0, -err_z);                  //   adjust the altitude
//      double yaw_error = getDiff2Angles(getYaw(goal_pose), getYaw(current_pose), PI);
      setpoint.twist.angular.z = pid_yaw->calculate(0.0, - yaw_error);        //   and adjust yaw to point as per the last waypoint
    } else if (err_dist < rotate_to_heading_min_distance_) {               // If we are close in the XY plane, but not quite there, then
      RCLCPP_INFO(logger_, "Mode 2");
      setpoint.twist.linear.x = pid_x->calculate(0, -err_x);                  //   and tune the position using X and Y PID controllers.
      setpoint.twist.linear.y = pid_y->calculate(0, -err_y); 
      setpoint.twist.linear.z = pid_z->calculate(0, -err_z);                  //   adjust the altitude
    } else if (fabs(yaw_error) > rotate_to_heading_min_angle_) {                          // If we are far far away, and pointing wrong, then    
      RCLCPP_INFO(logger_, "Mode 3: ytt=%.4f > goal_yaw_tol =%.4f", fabs(yaw_to_target), rotate_to_heading_min_angle_);
      setpoint.twist.linear.z = pid_z->calculate(0, -err_z);                  //   adjust the altitude
      setpoint.twist.angular.z = pid_yaw->calculate(0.0, -yaw_error);         //   adjust yaw to point to the next waypoint     
    } else {
      RCLCPP_INFO(logger_, "Mode 4 - Actual flight");                      // Otherwise the yaw is good,
      setpoint.twist.angular.z = pid_yaw->calculate(0.0, -yaw_error);        //   correct the yaw (fine tuning)
      setpoint.twist.linear.x = pid_x->calculate(0, -err_dist);               //   increase foreward thrust to get closer to the target  
      setpoint.twist.linear.z = pid_z->calculate(0, -err_z);                  //   adjust the altitude to get closer to the target
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
    */
   }
      
  bool VhfPlusController::shouldRotateToPath(
  const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path)
{
  // Whether we should rotate robot to rough path heading
  angle_to_path = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
  return use_rotate_to_heading_ && fabs(angle_to_path) > rotate_to_heading_min_angle_;
}

bool VhfPlusController::shouldRotateToGoalHeading(
  const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  // Whether we should rotate robot to goal heading
  double dist_to_goal = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  return use_rotate_to_heading_ && dist_to_goal < goal_dist_tol_;
}

void VhfPlusController::rotateToHeading(
  double & linear_vel, double & angular_vel,
  const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed)
{
  // Rotate in place using max angular velocity / acceleration possible
  linear_vel = 0.0;
  const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
  angular_vel = sign * rotate_to_heading_angular_vel_;

  const double & dt = control_duration_;
  const double min_feasible_angular_speed = curr_speed.angular.z - max_angular_accel_ * dt;
  const double max_feasible_angular_speed = curr_speed.angular.z + max_angular_accel_ * dt;
  angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);
}
  
void VhfPlusController::applyConstraints(
  const double & dist_error, const double & lookahead_dist,
  const double & curvature, const geometry_msgs::msg::Twist & /*curr_speed*/,
  double & linear_vel, double & sign)
{
  double curvature_vel = linear_vel;
  double approach_vel = linear_vel;

  // limit the linear velocity by curvature
  const double radius = fabs(1.0 / curvature);
  const double & min_rad = regulated_linear_scaling_min_radius_;
  if (use_regulated_linear_velocity_scaling_ && radius < min_rad) {
    curvature_vel *= 1.0 - (fabs(radius - min_rad) / min_rad);
  }

  // Use the lowest of the constraint heuristic, but above the minimum translational speed
  linear_vel = std::max(linear_vel, regulated_linear_scaling_min_speed_);

  // if the actual lookahead distance is shorter than requested, that means we're at the
  // end of the path. We'll scale linear velocity by error to slow to a smooth stop.
  // This expression is eq. to (1) holding time to goal, t, constant using the theoretical
  // lookahead distance and proposed velocity and (2) using t with the actual lookahead
  // distance to scale the velocity (e.g. t = lookahead / velocity, v = carrot / t).
  if (dist_error > 2.0 * costmap_->getResolution()) {
    double velocity_scaling = 1.0 - (dist_error / lookahead_dist);
    double unbounded_vel = approach_vel * velocity_scaling;
    if (unbounded_vel < min_approach_linear_velocity_) {
      approach_vel = min_approach_linear_velocity_;
    } else {
      approach_vel *= velocity_scaling;
    }

    // Use the lowest velocity between approach and other constraints, if all overlapping
    linear_vel = std::min(linear_vel, approach_vel);
  }

  // Limit linear velocities to be valid
  linear_vel = std::clamp(fabs(linear_vel), 0.0, desired_linear_vel_);
  linear_vel = sign * linear_vel;
}  

void VhfPlusController::applyVerticalConstraints(
  const double & dist_error, const double & lookahead_dist,
  const double & curvature, const geometry_msgs::msg::Twist & /*curr_speed*/,
  double & vertical_vel, double & sign)
{
  double curvature_vel = vertical_vel;
  double approach_vel = vertical_vel;

  // limit the linear velocity by curvature
  const double radius = fabs(1.0 / curvature);
  const double & min_rad = regulated_linear_scaling_min_radius_;
  if (use_regulated_linear_velocity_scaling_ && radius < min_rad) {
    curvature_vel *= 1.0 - (fabs(radius - min_rad) / min_rad);
  }

  // Use the lowest of the constraint heuristic, but above the minimum translational speed
  vertical_vel = std::max(vertical_vel, regulated_linear_scaling_min_speed_);

  // if the actual lookahead distance is shorter than requested, that means we're at the
  // end of the path. We'll scale linear velocity by error to slow to a smooth stop.
  // This expression is eq. to (1) holding time to goal, t, constant using the theoretical
  // lookahead distance and proposed velocity and (2) using t with the actual lookahead
  // distance to scale the velocity (e.g. t = lookahead / velocity, v = carrot / t).
  if (dist_error > 2.0 * costmap_->getResolution()) {
    double velocity_scaling = 1.0 - (dist_error / lookahead_dist);
    double unbounded_vel = approach_vel * velocity_scaling;
    if (unbounded_vel < min_approach_linear_velocity_) {
      approach_vel = min_approach_linear_velocity_;
    } else {
      approach_vel *= velocity_scaling;
    }

    // Use the lowest velocity between approach and other constraints, if all overlapping
    vertical_vel = std::min(vertical_vel, approach_vel);
  }

  // Limit linear velocities to be valid
  vertical_vel = std::clamp(fabs(vertical_vel), 0.0, desired_linear_vel_);
  vertical_vel = sign * vertical_vel;
}  


  double VhfPlusController::getLookAheadDistance(
    const geometry_msgs::msg::Twist & speed)
  {
    // If using velocity-scaled look ahead distances, find and clamp the dist
    // Else, use the static look ahead distance
    double lookahead_dist = lookahead_dist_;
    if (use_velocity_scaled_lookahead_dist_) {
      lookahead_dist = std::max(fabs(speed.linear.x), fabs(speed.linear.z)) * lookahead_time_;
      lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
    }

    return lookahead_dist;
  }
  
  std::pair<int, int> VhfPlusController::get_ez_grid_pos(const octomap:: point3d & goal)
  {
    // Now we want to work in the base_link frame to incorporate the yaw of the drone.  This is the trick in the algorithm I think.
    // The VCP will be at 0,0,0 and the target will be positioned apropriately.  Straight flight will take one to the middle of the
    // histogram.  This way section 4.3 of the algoritm becomes easy to impliment.
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = rclcpp::Time();
    goal_pose.pose.position.x = goal.x();
    goal_pose.pose.position.y = goal.y();
    goal_pose.pose.position.z = goal.z();
    goal_pose.pose.orientation.x = 0.0;    // Just specify a neutral yaw.  Not really relevant.
    goal_pose.pose.orientation.y = 0.0;
    goal_pose.pose.orientation.z = 0.0;
    goal_pose.pose.orientation.w = 1.0;
    
    geometry_msgs::msg::PoseStamped voxel;
    transformPose("base_link", goal_pose, voxel);
/*    
    RCLCPP_INFO(logger_, "Transform point from [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]", 
                goal.x(), goal.y(), goal.z(),
                voxel.pose.position.x,
                voxel.pose.position.y,
                voxel.pose.position.z);
*/ 
    
    geometry_msgs::msg::PoseStamped source_pose;
    goal_pose.header.frame_id = "map";
    goal_pose.pose.position.x = 0.0;
    goal_pose.pose.position.y = 0.0;
    goal_pose.pose.position.z = 0.0;
        
    auto ez = get_ez(source_pose, voxel);
    
    // Moving the values into positive whole numbers, scaled to fit into our matrix
    double e = floor( (90.0 + rad_to_deg( ez.first) ) / ALPHA_RES);
    double z = floor( (180.0 + rad_to_deg( ez.second ) ) / ALPHA_RES);
    
    return std::pair<int, int>(e,z);
  }
    
  std::pair<double, double> VhfPlusController::get_ez(const geometry_msgs::msg::PoseStamped & current_pose,
                                                      const geometry_msgs::msg::PoseStamped & target_pose)
  {
    
    // Using direction cosines as discussed
    // https://gis.stackexchange.com/questions/108547/how-to-calculate-distance-azimuth-and-dip-from-two-xyz-coordinates
    // by https://gis.stackexchange.com/users/2581/gene
    double distance = std::hypot(target_pose.pose.position.x - current_pose.pose.position.x, 
                                 target_pose.pose.position.y - current_pose.pose.position.y, 
                                 target_pose.pose.position.z - current_pose.pose.position.z);
    double cosalpha = (target_pose.pose.position.x - current_pose.pose.position.x) / distance;
    double cosbeta = (target_pose.pose.position.y - current_pose.pose.position.y) / distance;
    double cosgamma = (target_pose.pose.position.z - current_pose.pose.position.z) / distance;
    double plunge = asin(cosgamma);   // # the resulting dip_plunge is positive downward if z2 > z1
    
    // prevent division by zero
    if ( (cosalpha == 0.0) && (cosbeta == 0.0) ) {
      cosalpha = 0.000001;
    }
    double azimuth =  atan2(cosalpha, cosbeta); 
    
    return std::pair<double, double>(plunge, azimuth);
  }
  
  geometry_msgs::msg::PoseStamped VhfPlusController::getLookAheadPoint(
    const double & lookahead_dist,
    const geometry_msgs::msg::PoseStamped & current_pose)
  {    
    
    // Start off working in the MAP frame.  This allows us to search against the global path,
    // and to cater for the yaw of the drone.
    
    // Find the first pose which is at a distance greater than the lookahead distance
    auto goal_pose_it = std::find_if(
      global_plan_.poses.begin(), global_plan_.poses.end(), [&](const auto & ps) {
        return (euclidean_distance(ps, current_pose) >= lookahead_dist);
      });
    
    double bounding_box_radius = lookahead_dist;
    
    // If the pose is not far enough, take the last pose
    if (goal_pose_it == global_plan_.poses.end()) {
      goal_pose_it = std::prev(global_plan_.poses.end());
      bounding_box_radius = euclidean_distance(current_pose, *goal_pose_it);
    }
    
    RCLCPP_DEBUG(logger_, "Navigating from [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]",
                current_pose.pose.position.x,
                current_pose.pose.position.y,
                current_pose.pose.position.z,
                goal_pose_it->pose.position.x,
                goal_pose_it->pose.position.y,
                goal_pose_it->pose.position.z);               
    
    RCLCPP_DEBUG(logger_, "Boundig box radius is %.2f", bounding_box_radius);

                 
    if (bounding_box_radius < 0.5 ) {    // Too close to calculate anything usable
      return *goal_pose_it;
    }  

    // 4.1 Iterate through the Octomap and 4.2 populate the 2D Primary Historam        
    octomap:: point3d start_point(current_pose.pose.position.x,
                                  current_pose.pose.position.y,
                                  current_pose.pose.position.z);

    octomap::point3d min(current_pose.pose.position.x - bounding_box_radius,  
                         current_pose.pose.position.y - bounding_box_radius,
                         current_pose.pose.position.z - bounding_box_radius);  
    octomap::point3d max(current_pose.pose.position.x + bounding_box_radius,
                         current_pose.pose.position.y + bounding_box_radius,
                         current_pose.pose.position.z + bounding_box_radius);
                                
    double const_b = 5.0;    // Just a random number
    double const_a = 1.0 + const_b * pow((bounding_box_radius - 1.0) / 2, 2);
       
    Histogram histogram(ALPHA_RES);
    histogram.set_zero();
     
    for(octomap::OcTree::leaf_bbx_iterator it = costmap_->begin_leafs_bbx(min,max),
        end=costmap_->end_leafs_bbx(); it!= end; ++it) {
             
      octomap::point3d end_point(it.getCoordinate());
      double distance = start_point.distance(end_point);
      double l = distance - (robot_radius_ + safety_radius_ + costmap_->getResolution());
      if ( (distance <= bounding_box_radius) && (l > 0.001) )  {   // Work in the drone radius, to minimuse calculation.
        
        // The point is within a sphere around the drone, thus an active cell
        std::pair<int, int> coords = get_ez_grid_pos(end_point);   // NOTE this point is in the base_link frame.  Remember when translating to the lookahead point    
        int lamda = floor(rad_to_deg( asin((robot_radius_ + safety_radius_ + costmap_->getResolution()) / distance) / ALPHA_RES));
                
        double weight = pow( it->getOccupancy(), 2) * (const_a - (const_b * l));
             
        if( weight > 0.001 ) {    // Small optimization.  Don't iterate for zero weight or if the target is on top of itself.
          // Incorporate the size of the robot in the calculation.  All these points in the histogram are influenced
          for(int e = std::max(0, coords.first-lamda); e <= coords.first+lamda; e++) {
            for(int z = std::max(0, coords.second-lamda); z <= coords.second+lamda; z++) {
              
              // Trim all values of e & z that are out of bounds, because we add or subtract lamda
              if ((e < histogram.e_dim()) && ( z < histogram.z_dim() )) {  
                histogram.add_weight(e, z, weight);
              }  
            }             
          }
        }  
      };  // else discard the point.
    }  
    
/*
  // A nasty piece of code to print the histogram
    for(int e = 0; e < histogram.e_dim(); e++) {
      RCLCPP_INFO(logger_, "e %d | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | ", 
                  e,  
                  histogram.get_weight(e,0),
                  histogram.get_weight(e,3),
                  histogram.get_weight(e,6),
                  histogram.get_weight(e,9),
                  histogram.get_weight(e,12),
                  histogram.get_weight(e,15),
                  histogram.get_weight(e,18),
                  histogram.get_weight(e,21),
                  histogram.get_weight(e,24),
                  histogram.get_weight(e,27),
                  histogram.get_weight(e,30),
                  histogram.get_weight(e,32),
                  histogram.get_weight(e,36),
                  histogram.get_weight(e,39),
                  histogram.get_weight(e,42),
                  histogram.get_weight(e,45),
                  histogram.get_weight(e,48),
                  histogram.get_weight(e,51),
                  histogram.get_weight(e,54),
                  histogram.get_weight(e,57),
                  histogram.get_weight(e,60)
                 );    
    }
*/
    // 4.4 Fourth Stage: 2D Binary Polar Histogram
    /* IMPROVEMENT REQUIRED:  
       Because the 3DVFH+ algorithm uses a 2D polar histogram, the thresholds
       high and low need to change when using a different elevation angle Be. The cells
       of the 2D polar histogram do not have the same size (as shown in Figure 2) so
       to compensate for this, different thresholds are required for different elevation
       angles, see (18). The size of these thresholds depends on the robot, the robot's
       speed, the window size of stage  ve, the octomap resolution and the bounding
       sphere size.
    */

    histogram.go_binary(0.03, 0.3);      // See note above.  Using constants for now
    
    // 4.5 Fith Stage: Path detection and selection
    int best_e, best_z;
    double best_score = std::numeric_limits<double>::max();   // Default to the worst possible score
    
    // Calculate some numbers to optimise the algorithm
    auto target_ez = get_ez(current_pose, *goal_pose_it); 
    double target_elevation = rad_to_deg( target_ez.first );
    double target_azimuth = rad_to_deg( target_ez.second );
    
    //double target_azimuth = rad_to_deg(angle(current_pose.pose.position.x, current_pose.pose.position.y, 
    //                                         goal_pose_it->pose.position.x, goal_pose_it->pose.position.y));
    //double target_elevation =  rad_to_deg(angle(current_pose.pose.position.x, current_pose.pose.position.z, 
    //                                            goal_pose_it->pose.position.x, goal_pose_it->pose.position.z));
/*    RCLCPP_INFO(logger_, "Calculating for [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]",
                current_pose.pose.position.x,
                current_pose.pose.position.y,
                current_pose.pose.position.z,
                goal_pose_it->pose.position.x,
                goal_pose_it->pose.position.y,
                goal_pose_it->pose.position.z);    
    
    RCLCPP_INFO(logger_, "Target  elevation %.3f, azimuth %.3f", target_elevation, target_azimuth);    
    int target_e = floor( (90.0 + target_elevation ) / ALPHA_RES);
    int target_z = floor( (180.0 + target_azimuth ) / ALPHA_RES);
    RCLCPP_INFO(logger_, "Target is at elevation %d, azimuth %d", target_e, target_z);
  */  
    for(int e = 0; e < histogram.e_dim(); e++) {
      for(int z = 0; z < histogram.z_dim(); z++) {
        
        if( histogram.path_available(e, z) ) {

          // Keep in mind:
          // The histogram is in the base_link frame with the robot at 0,0,0.  
          // The histogram is in DEGREES
          // e and z are indexes relative to ALPHA_RES.  Don't forget to multiply out
          // histogram indexes have been manipulated to be positive (e += 90, z+=180 degrees)

          double z_angle = (z * ALPHA_RES) - 180; // z angle is the direction of azimuth (horisontal) 
          
          // "The first path weight is the difference between the target angle and this candidate direction"      
          double delta_vk = fabs(getDiff2Angles(target_azimuth, z_angle, 180));
          
          // "The second path weight is the difference between the rotation of the robot" (yaw in map frame) " and the candidate direction" 
          // But the matrix is in the base_link frame.  current Yaw is 0.  Thus the penalty is simply the azimuth to the target.  
          double delta_vtheta = fabs(z_angle);

          // "The last path weight is the difference between the previous selected direction and the candidate direction"
          double delta_vk1 = fabs(getDiff2Angles(last_z_angle_, z_angle, 180));
          
          double z_score = MU1*delta_vk + MU2*delta_vtheta + MU3*delta_vk1;           
          
          // Repeat these now for the elevation
          double e_angle = (e * ALPHA_RES) - 90;  // e_angle is the elevation in degrees  
          delta_vk = fabs(getDiff2Angles(e_angle, target_elevation, 180));          
          delta_vtheta = fabs(e_angle); 
          delta_vk1 = fabs(getDiff2Angles(last_e_angle_, e_angle, 180));               
                    
          double e_score = MU1*delta_vk + MU2*delta_vtheta + MU3*delta_vk1;
                     
          if ((z_score + e_score) < best_score) {
            best_z = z;
            best_e = e;
            best_score = z_score + e_score;
          }  
        }
      }             
    }
        
    RCLCPP_DEBUG(logger_, "Fly Azimuth (z 0-59) %d, Elevation (e 0-29) %d, Weight %.4f", best_z, best_e, histogram.get_weight(best_e, best_z));
    
    double plunge = best_e * ALPHA_RES - 90.0;
    double azimuth = best_z * ALPHA_RES - 180.0;

    geometry_msgs::msg::PoseStamped point;
    point.header.frame_id = "map";              
    point.pose.position.x = bounding_box_radius * cos(deg_to_rad(azimuth)) + current_pose.pose.position.x;
    point.pose.position.y = bounding_box_radius * sin(deg_to_rad(azimuth)) + current_pose.pose.position.y;
    point.pose.position.z = bounding_box_radius * sin(deg_to_rad(plunge)) + current_pose.pose.position.z;
    
//    RCLCPP_WARN(logger_, "Fly to point [%.2f, %.2f, %.2f] to avoid obstacle.", 
//                point.pose.position.x,
//                point.pose.position.y,
//                point.pose.position.z);
    
    return point;    
  }   
    
  bool VhfPlusController::transformPose(
    const std::string frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const
  {
    if (in_pose.header.frame_id == frame) {
      out_pose = in_pose;
      return true;
    }
    try {
      tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
      out_pose.header.frame_id = frame;
      return true;
//    } catch (tf2::TransformException & ex) {
//      RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
//    }
    } catch (tf2::LookupException & ex) {
      RCLCPP_ERROR(
        logger_,
        "No Transform available Error looking up target frame: %s\n", ex.what());
    } catch (tf2::ConnectivityException & ex) {
      RCLCPP_ERROR(
        logger_,
        "Connectivity Error looking up target frame: %s\n", ex.what());
    } catch (tf2::ExtrapolationException & ex) {
      RCLCPP_ERROR(
        logger_,
        "Extrapolation Error looking up target frame: %s\n", ex.what());
    } catch (tf2::TimeoutException & ex) {
      RCLCPP_ERROR(
        logger_,
        "Transform timeout with tolerance: %.4f", transform_tolerance_);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(
        logger_, "Failed to transform from %s to %s",
        in_pose.header.frame_id.c_str(), frame.c_str());
    }
    return false;
  }
  
//  double VhfPlusController::getCostmapMaxExtent() const
//  {
//    double x, y, z;
//    costmap_->getMetricSize(x, y, z);
//    return x / 2.0;
//  }

}  // namespace controller_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(controller_plugins::VhfPlusController, navigation_lite::Controller)
