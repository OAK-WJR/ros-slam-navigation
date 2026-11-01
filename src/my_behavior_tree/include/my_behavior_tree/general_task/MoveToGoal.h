#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include "behaviortree_cpp_v3/behavior_tree.h"

class MoveToGoal : public BT::SyncActionNode
{
public:
    // 构造函数中初始化订阅和发布
    MoveToGoal(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        ros::NodeHandle nh;
        // 订阅 /waypoints，类型为 PoseArray
        sub_waypoints_ = nh.subscribe("/waypoints", 10, 
                                      &MoveToGoal::waypointsCallback, this);
        // 发布目标到 move_base 的默认导航话题
        pub_goal_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    }

    // 本节点需要两个输入端口：goal（索引）和 speed（仅用于示例）
    static BT::PortsList providedPorts()
    {
        return { 
            BT::InputPort<int>("goal"),
            BT::InputPort<int>("speed")
        };
    }

    // SyncActionNode 的核心逻辑在 tick() 中
    BT::NodeStatus tick() override
    {
        int goal_index;
        int speed;

        // 从黑板中读取参数
        if (!getInput<int>("goal", goal_index))
            throw BT::RuntimeError("Missing parameter [goal] in MoveToGoal");
        if (!getInput<int>("speed", speed))
            throw BT::RuntimeError("Missing parameter [speed] in MoveToGoal");

        // 如果还没有收到任何目标点，则返回 FAILURE
        if (waypoints_.empty())
        {
            ROS_WARN("MoveToGoal: 尚未收到任何 /waypoints 消息，无法前往目标");
            return BT::NodeStatus::FAILURE;
        }

        // 检查目标索引是否有效
        if (goal_index < 0 || goal_index >= static_cast<int>(waypoints_.size()))
        {
            ROS_WARN("MoveToGoal: 目标索引 %d 越界，目前有 %zu 个目标",
                     goal_index, waypoints_.size());
            return BT::NodeStatus::FAILURE;
        }

        // 取出目标 Pose 并封装为 PoseStamped 消息
        geometry_msgs::PoseStamped goal_msg;
        goal_msg.header.frame_id = "map";
        goal_msg.header.stamp = ros::Time::now();
        goal_msg.pose = waypoints_[goal_index];

        ROS_INFO("MoveToGoal: 发布目标点索引 %d (speed=%d)", goal_index, speed);

        // 发布到导航话题
        pub_goal_.publish(goal_msg);

        // 此处直接返回 SUCCESS（未等待导航反馈）
        return BT::NodeStatus::SUCCESS;
    }

private:
    // 回调函数：收到 PoseArray 消息后，保存最新的所有目标点
    void waypointsCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
    {
        // 直接将 PoseArray 中所有 Pose 保存到内部变量 waypoints_
        waypoints_ = msg->poses;
        ROS_INFO("MoveToGoal: 收到 /waypoints 消息，共 %zu 个目标", waypoints_.size());
    }

    ros::Subscriber sub_waypoints_;
    ros::Publisher  pub_goal_;
    std::vector<geometry_msgs::Pose> waypoints_;
};
