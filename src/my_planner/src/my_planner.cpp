#include "my_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

PLUGINLIB_EXPORT_CLASS( my_planner::MyPlanner, nav_core::BaseLocalPlanner)

namespace my_planner 
{
    MyPlanner::MyPlanner()
    {
        setlocale(LC_ALL,"");
    }
    MyPlanner::~MyPlanner()
    {}
    void MyPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        if (initialized_) return;
        ROS_WARN("MyPlanner init: using Pure Pursuit + speed limits (params can make it faster/smoother)");

        costmap_ros_ = costmap_ros;
        tf_listener_ = std::make_unique<tf::TransformListener>();

        // Read params from the private namespace: ~/<name>/xxx
        ros::NodeHandle nh("~/" + name);
        nh.param("lookahead_dist", lookahead_dist_, lookahead_dist_);
        nh.param("goal_dist_tol",  goal_dist_tol_,  goal_dist_tol_);
        nh.param("goal_yaw_tol",   goal_yaw_tol_,   goal_yaw_tol_);

        nh.param("max_vel_x",      max_vel_x_,      max_vel_x_);
        nh.param("min_vel_x",      min_vel_x_,      min_vel_x_);
        nh.param("max_vel_theta",  max_vel_theta_,  max_vel_theta_);
        nh.param("min_vel_theta",  min_vel_theta_,  min_vel_theta_);

        nh.param("kp_lin",         kp_lin_,         kp_lin_);
        nh.param("kp_ang",         kp_ang_,         kp_ang_);
        nh.param("yaw_stop_thresh",yaw_stop_thresh_,yaw_stop_thresh_);

        nh.param("allow_in_place_rotate", allow_in_place_rotate_, allow_in_place_rotate_);
        nh.param("goal_yaw_align",        goal_yaw_align_,        goal_yaw_align_);

        initialized_ = true;
    }

    bool MyPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
    {
        global_plan_ = plan;
        goal_reached_ = false;
        target_index_ = 0;
        return true;
    }

    bool MyPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        cmd_vel = geometry_msgs::Twist();

        if (!initialized_) {
            ROS_WARN_THROTTLE(1.0, "MyPlanner: initialize() not called yet");
            return false;
        }
        if (global_plan_.empty()) {
            ROS_WARN_THROTTLE(1.0, "MyPlanner has not received a global path (plan is empty)");
            return false;
        }

        // 1) Goal pose in base_link
        const int final_index = static_cast<int>(global_plan_.size()) - 1;
        geometry_msgs::PoseStamped pose_final_bl;
        try {
            geometry_msgs::PoseStamped tmp = global_plan_[final_index];
            tmp.header.stamp = ros::Time(0);
            tf_listener_->transformPose("base_link", tmp, pose_final_bl);
        } catch (const tf::TransformException& e) {
            ROS_WARN_THROTTLE(1.0, "TF transform failed: cannot transform the goal to base_link: %s", e.what());
            return false;
        }

        const double dx_goal = pose_final_bl.pose.position.x;
        const double dy_goal = pose_final_bl.pose.position.y;
        const double dist_goal = std::hypot(dx_goal, dy_goal);

        // 2) Goal reached: optionally align the heading
        if (dist_goal < goal_dist_tol_) {
            const double yaw_err = tf::getYaw(pose_final_bl.pose.orientation);
            if (!goal_yaw_align_ || std::fabs(yaw_err) < goal_yaw_tol_) {
                goal_reached_ = true;
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                return true;
            }
            // Heading needs aligning, but if in-place rotation is disabled, do not rotate here; just stop (leave it to the upper layer/manual handling)
            if (!allow_in_place_rotate_) {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                return true;
            }

            double w = std::max(min_vel_theta_, std::min(max_vel_theta_, std::fabs(kp_ang_ * yaw_err)));
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = (yaw_err >= 0.0) ? w : -w;
            return true;
        }

        // 3) Pick the lookahead point (base_link frame)
        geometry_msgs::PoseStamped target_bl;
        bool found = false;
        for (int i = std::max(0, target_index_); i < static_cast<int>(global_plan_.size()); i++) {
            try {
                geometry_msgs::PoseStamped tmp = global_plan_[i];
                tmp.header.stamp = ros::Time(0);
                tf_listener_->transformPose("base_link", tmp, target_bl);
            } catch (const tf::TransformException&) {
                continue;
            }
            const double d = std::hypot(target_bl.pose.position.x, target_bl.pose.position.y);
            if (d >= lookahead_dist_) {
                target_index_ = i;
                found = true;
                break;
            }
        }
        if (!found) {
            // No point far enough away found; use the goal
            target_bl = pose_final_bl;
        }

        const double tx = target_bl.pose.position.x;
        const double ty = target_bl.pose.position.y;
        const double heading_err = std::atan2(ty, tx);

        /*
        // 4) Angle error too large: by default turn in place first; but with no IMU / a wheels-off-the-ground test, this can be turned off
        if (std::fabs(heading_err) > yaw_stop_thresh_ && allow_in_place_rotate_) {
            cmd_vel.linear.x = 0.0;
            double w = std::max(min_vel_theta_, std::min(max_vel_theta_, std::fabs(kp_ang_ * heading_err)));
            cmd_vel.angular.z = (heading_err >= 0.0) ? w : -w;
            return true;
        }
            */

        // 5) Linear speed: proportional to distance, also affected by heading error (the more off, the slower)
        const double dist_target = std::hypot(tx, ty);
        double v = kp_lin_ * dist_target;
        v *= std::max(0.0, std::cos(heading_err));  // The larger the heading error, the slower
        v = std::min(max_vel_x_, std::max(min_vel_x_, v));

        // 6) Angular speed: proportional to heading error, with limits
        double w = kp_ang_ * heading_err;
        w = std::max(-max_vel_theta_, std::min(max_vel_theta_, w));
        if (std::fabs(w) > 1e-6 && std::fabs(w) < min_vel_theta_) {
            w = (w > 0.0) ? min_vel_theta_ : -min_vel_theta_;
        }

        cmd_vel.linear.x = v;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = w;

        return true;
    }

    bool MyPlanner::isGoalReached()
    {
        return goal_reached_;
    }
} // namespace my_planner