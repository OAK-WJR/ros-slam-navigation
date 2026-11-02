#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include "behaviortree_cpp_v3/behavior_tree.h"

class MoveToGoal : public BT::SyncActionNode
{
public:
    // Initialize the subscriber and publisher in the constructor
    MoveToGoal(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        ros::NodeHandle nh;
        // Subscribe to /waypoints, type PoseArray
        sub_waypoints_ = nh.subscribe("/waypoints", 10, 
                                      &MoveToGoal::waypointsCallback, this);
        // Publish the goal to move_base's default navigation topic
        pub_goal_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    }

    // This node needs two input ports: goal (index) and speed (example only)
    static BT::PortsList providedPorts()
    {
        return { 
            BT::InputPort<int>("goal"),
            BT::InputPort<int>("speed")
        };
    }

    // SyncActionNode's core logic is in tick()
    BT::NodeStatus tick() override
    {
        int goal_index;
        int speed;

        // Read params from the blackboard
        if (!getInput<int>("goal", goal_index))
            throw BT::RuntimeError("Missing parameter [goal] in MoveToGoal");
        if (!getInput<int>("speed", speed))
            throw BT::RuntimeError("Missing parameter [speed] in MoveToGoal");

        // If no goal points have been received yet, return FAILURE
        if (waypoints_.empty())
        {
            ROS_WARN("MoveToGoal: no /waypoints message received yet, cannot go to the goal");
            return BT::NodeStatus::FAILURE;
        }

        // Check whether the goal index is valid
        if (goal_index < 0 || goal_index >= static_cast<int>(waypoints_.size()))
        {
            ROS_WARN("MoveToGoal: goal index %d out of range, there are %zu goals now",
                     goal_index, waypoints_.size());
            return BT::NodeStatus::FAILURE;
        }

        // Take the goal Pose and wrap it in a PoseStamped message
        geometry_msgs::PoseStamped goal_msg;
        goal_msg.header.frame_id = "map";
        goal_msg.header.stamp = ros::Time::now();
        goal_msg.pose = waypoints_[goal_index];

        ROS_INFO("MoveToGoal: published goal point index %d (speed=%d)", goal_index, speed);

        // Publish to the navigation topic
        pub_goal_.publish(goal_msg);

        // Return SUCCESS right away here (does not wait for navigation feedback)
        return BT::NodeStatus::SUCCESS;
    }

private:
    // Callback: after a PoseArray message arrives, save all the latest goal points
    void waypointsCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
    {
        // Save all Poses in the PoseArray directly to the internal variable waypoints_
        waypoints_ = msg->poses;
        ROS_INFO("MoveToGoal: received /waypoints message, %zu goals in total", waypoints_.size());
    }

    ros::Subscriber sub_waypoints_;
    ros::Publisher  pub_goal_;
    std::vector<geometry_msgs::Pose> waypoints_;
};
