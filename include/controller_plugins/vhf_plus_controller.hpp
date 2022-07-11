#ifndef NAVIGATION_LITE_VHF_PLUS_CONTROLLER_PLUGIN_HPP_
#define NAVIGATION_LITE_VHF_PLUS_CONTROLLER_PLUGIN_HPP_

#include <string>
#include <mutex>
#include <limits>      // std::numeric_limits
#include <cmath>       // std::fabs
#include <algorithm>   // std::max

#include "navigation_lite/controller.hpp"
#include "controller_plugins/pid.hpp"


static constexpr unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;

namespace controller_plugins
{
  class VhfPlusController : public navigation_lite::Controller
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
      double getLookAheadDistance(const geometry_msgs::msg::Twist &);
//      geometry_msgs::msg::PoseStamped getLookAheadPoint(
//        const double & lookahead_dist,
//        const geometry_msgs::msg::PoseStamped & current_pose);


      rclcpp::Node::SharedPtr node_;
      std::string plugin_name_;
      std::shared_ptr<tf2_ros::Buffer> tf_;
      std::shared_ptr<octomap::OcTree> costmap_;
      rclcpp::Logger logger_ {rclcpp::get_logger("VhfPlusController")};
      rclcpp::Clock::SharedPtr clock_;
  
      nav_msgs::msg::Path global_plan_;
      
      double max_angular_vel_;
      double goal_dist_tol_;
      
      double desired_linear_vel_, base_desired_linear_vel_;
      double min_approach_linear_distance_;
      double lookahead_dist_;
      double rotate_to_heading_angular_vel_;
      double max_lookahead_dist_;
      double min_lookahead_dist_;
      double lookahead_time_;
      tf2::Duration transform_tolerance_;
      double control_duration_;
      bool use_velocity_scaled_lookahead_dist_;
      double yaw_control_limit_;
      double control_frequency_;
      double max_linear_accel_;
      bool allow_reversing_;
//      double max_robot_pose_search_dist_;
      bool use_interpolation_;
      
      bool use_regulated_linear_velocity_scaling_;
      bool use_cost_regulated_linear_velocity_scaling_;
      double cost_scaling_dist_;
      double cost_scaling_gain_;
      double regulated_linear_scaling_min_radius_;
      double regulated_linear_scaling_min_speed_;
      double max_allowed_time_to_collision_up_to_carrot_;
      double inflation_cost_scaling_factor_;
      double min_approach_linear_velocity_;
      bool use_rotate_to_heading_;
      double rotate_to_heading_min_angle_;
      double rotate_to_heading_min_distance_;
      double max_angular_accel_;
      
      double robot_radius_;
      double safety_radius_;
            
      std::string base_frame_id_;

    private:
      std::pair<int, int> get_ez_grid_pos(const octomap:: point3d & goal);
      std::pair<double, double> get_ez(const geometry_msgs::msg::PoseStamped & current_pose,
                                       const geometry_msgs::msg::PoseStamped & target_pose);
      
      nav_msgs::msg::Path transformGlobalPlan( const geometry_msgs::msg::PoseStamped & pose);
      bool transformPose( const std::string frame,
                          const geometry_msgs::msg::PoseStamped & in_pose,
                          geometry_msgs::msg::PoseStamped & out_pose) const;
//      double getCostmapMaxExtent() const;
      bool shouldRotateToPath(const geometry_msgs::msg::PoseStamped & carrot_pose, 
                              double & angle_to_path);
      bool shouldRotateToGoalHeading(const geometry_msgs::msg::PoseStamped & carrot_pose);
      void rotateToHeading( double & linear_vel, double & angular_vel,
                            const double & angle_to_path, 
                            const geometry_msgs::msg::Twist & curr_speed);
      void applyConstraints(const double & dist_error, const double & lookahead_dist,
                            const double & curvature, const geometry_msgs::msg::Twist & /*curr_speed*/,
                            double & linear_vel, double & sign);
      void applyVerticalConstraints(const double & dist_error, const double & lookahead_dist,
                                    const double & curvature, const geometry_msgs::msg::Twist & /*curr_speed*/,
                                    double & vertical_vel, double & sign) ;                     
      geometry_msgs::msg::PoseStamped getLookAheadPoint(
        const double & lookahead_dist,
        const geometry_msgs::msg::PoseStamped & current_pose);
            
      // PID Controllers  
      std::shared_ptr<PID> pid_x;
      std::shared_ptr<PID> pid_y;
      std::shared_ptr<PID> pid_z;
      std::shared_ptr<PID> pid_yaw;
      
      //Global variables
      double last_e_angle_, last_z_angle_;
      double last_v_x, last_v_y;
      
  };

}  // namespace controller_plugins

#endif // NAVIGATION_LITE_VHF_PLUS_CONTROLLER_PLUGIN_HPP_