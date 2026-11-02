#ifndef MY_PLANNER_H_
#define MY_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>

#include <deque>
#include <vector>
#include <algorithm>
#include <cmath>
#include <memory>

namespace tf { class TransformListener; }


namespace my_planner 
{
    class MyPlanner : public nav_core::BaseLocalPlanner 
    {
        public:
            MyPlanner();
            ~MyPlanner();

            void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
            bool isGoalReached();

        private:
            bool initialized_ = false;

            // move_base calls setPlan() to pass in the global path
            std::vector<geometry_msgs::PoseStamped> global_plan_;
            int target_index_ = 0;
            bool goal_reached_ = false;

            // TF / costmap
            costmap_2d::Costmap2DROS* costmap_ros_ = nullptr;
            std::unique_ptr<tf::TransformListener> tf_listener_;

            // Tunable params (defaults are conservative; raise them in launch/yaml to go faster)
            double lookahead_dist_ = 0.40;     // Pure pursuit lookahead distance (larger = smoother)
            double goal_dist_tol_  = 0.10;     // Goal distance tolerance
            double goal_yaw_tol_   = 0.15;     // Goal heading tolerance (radians)

            double max_vel_x_     = 0.60;      // Max linear speed
            double min_vel_x_     = 0.05;      // Min linear speed (avoid being too slow/stuck)
            double max_vel_theta_ = 1.20;      // Max angular speed
            double min_vel_theta_ = 0.20;      // Min angular speed (when turning is needed)

            double kp_lin_ = 1.0;              // Linear speed proportional gain
            double kp_ang_ = 2.5;              // Angular speed proportional gain

            double yaw_stop_thresh_ = 0.90;    // When the angle error is too large, turn in place first (radians)

            // Friendlier for the "wheel odometry only, no IMU / wheels-off-the-ground test" case:
            // - allow_in_place_rotate=false: never send in-place rotation (v=0,w!=0) commands
            // - goal_yaw_align=false: once within the distance tolerance, skip heading alignment and treat the goal as reached
            bool allow_in_place_rotate_ = true;
            bool goal_yaw_align_ = true;
    };
} // namespace my_planner
 
#endif // MY_PLANNER_H_
