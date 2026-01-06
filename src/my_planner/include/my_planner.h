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

            // move_base 会调用 setPlan() 传入全局路径
            std::vector<geometry_msgs::PoseStamped> global_plan_;
            int target_index_ = 0;
            bool goal_reached_ = false;

            // TF / costmap
            costmap_2d::Costmap2DROS* costmap_ros_ = nullptr;
            std::unique_ptr<tf::TransformListener> tf_listener_;

            // 可调参数（默认值偏保守，你可在 launch/yaml 里调大来提速）
            double lookahead_dist_ = 0.40;     // 纯追踪前视距离（越大越平滑）
            double goal_dist_tol_  = 0.10;     // 到点距离阈值
            double goal_yaw_tol_   = 0.15;     // 到点朝向阈值（弧度）

            double max_vel_x_     = 0.60;      // 最大线速度
            double min_vel_x_     = 0.05;      // 最小线速度（避免太慢/卡住）
            double max_vel_theta_ = 1.20;      // 最大角速度
            double min_vel_theta_ = 0.20;      // 最小角速度（需要转向时）

            double kp_lin_ = 1.0;              // 线速度比例系数
            double kp_ang_ = 2.5;              // 角速度比例系数

            double yaw_stop_thresh_ = 0.90;    // 角度误差太大时先原地转（弧度）

            // 对“只有轮子里程计、无 IMU / 悬空测试”的场景更友好：
            // - allow_in_place_rotate=false：绝不发原地转圈（v=0,w!=0）的命令
            // - goal_yaw_align=false：到达距离阈值后不做朝向对齐，直接认为到点
            bool allow_in_place_rotate_ = true;
            bool goal_yaw_align_ = true;
    };
} // namespace my_planner
 
#endif // MY_PLANNER_H_
