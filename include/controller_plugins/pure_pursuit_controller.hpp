#ifndef NAVIGATION_LITE_PURE_PERSUIT_CONTROLLER_PLUGIN_HPP_
#define NAVIGATION_LITE_PURE_PERSUIT_CONTROLLER_PLUGIN_HPP_

#include <string>
#include <mutex>

#include "navigation_lite/controller.hpp"
#include "controller_plugins/pid.hpp"

enum control_mode {None, Pointing, Flying, Posing}; 

namespace controller_plugins
{
  class PurePursuitController : public navigation_lite::Controller
  {
    public:
      void configure(const rclcpp::Node::SharedPtr parent, 
                             std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                             std::shared_ptr<octomap::OcTree> costmap ) override;
      void setPath(const nav_msgs::msg::Path & path) override;
      
      geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & speed) override;
    
    protected:
      control_mode control_mode_;
      double getLookAheadDistance(const geometry_msgs::msg::Twist &);
      geometry_msgs::msg::PoseStamped getLookAheadPoint(
        const double & lookahead_dist,
        const geometry_msgs::msg::PoseStamped current_pose);

rclcpp::Node::SharedPtr node_;
      std::string plugin_name_;
      std::shared_ptr<tf2_ros::Buffer> tf_;
      std::shared_ptr<octomap::OcTree> costmap_;
      rclcpp::Logger logger_ {rclcpp::get_logger("PurePursuitController")};
      rclcpp::Clock::SharedPtr clock_;
  
      nav_msgs::msg::Path global_plan_;
      
      double max_angular_vel_;
      
      double desired_linear_vel_;
      double lookahead_dist_;
      double rotate_to_heading_angular_vel_;
      double max_lookahead_dist_;
      double min_lookahead_dist_;
      double lookahead_time_;
      double control_duration_;
      bool use_velocity_scaled_lookahead_dist_;
      double goal_dist_tol_;
      double goal_yaw_tol_;
      double yaw_control_limit_;
      double control_frequency_;
      double max_linear_accel_;
      std::string base_frame_id_;
      tf2::Duration transform_tolerance_;
    private:
      // PID Controllers  
      std::shared_ptr<PID> pid_x;
      std::shared_ptr<PID> pid_y;
      std::shared_ptr<PID> pid_z;
      std::shared_ptr<PID> pid_yaw;
      
      //Global variables
      double last_v_x, last_v_y;
      
  };

}  // namespace controller_plugins

#endif // NAVIGATION_LITE_PURE_PERSUIT_CONTROLLER_PLUGIN_HPP_