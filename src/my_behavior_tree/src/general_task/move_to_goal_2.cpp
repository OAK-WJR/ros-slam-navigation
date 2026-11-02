#include "my_behavior_tree/general_task/move_to_goal_2.h"
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <yaml-cpp/yaml.h>
#include <unordered_map>
#include <fstream>
#include <boost/filesystem.hpp>


// Define the global variable waypoints
std::unordered_map<std::string, geometry_msgs::Pose> waypoints;

// Implement the loadWaypoints function
void loadWaypoints(const std::string& filename)
{
  try
  {

    {
      // Use a local scope so the yaml object is destroyed as early as possible
      YAML::Node yaml = YAML::LoadFile(filename);
      for (const auto& it : yaml)
      {
        std::string goal_name = it.first.as<std::string>();
        geometry_msgs::Pose pose;
        pose.position.x = it.second["x"].as<double>();
        pose.position.y = it.second["y"].as<double>();
        pose.orientation.w = 1.0;
        waypoints[goal_name] = pose;
      }
    }
    ROS_INFO("📡 Read %lu goal points", waypoints.size());
  }
  catch(const std::exception &e)
  {
    ROS_ERROR("Failed to load waypoints file: %s", e.what());
    throw std::runtime_error(e.what());
  }
}



// Implement the MoveToGoal constructor
MoveToGoal2::MoveToGoal2(const std::string &name, const BT::NodeConfiguration &config)
    : BT::StatefulActionNode(name, config), ac_("move_base", true)
{
}

// Implement the static function providedPorts
BT::PortsList MoveToGoal2::providedPorts()
{
    return {BT::InputPort<std::string>("goal")};
}

// Implement the onStart function
BT::NodeStatus MoveToGoal2::onStart()
{
    std::string goal_name;
    if (!getInput("goal", goal_name))
    {
        ROS_ERROR("MoveToGoal: goal point not set!");
        return BT::NodeStatus::FAILURE;
    }

    if (waypoints.find(goal_name) == waypoints.end())
    {
        ROS_ERROR("MoveToGoal: goal point %s does not exist!", goal_name.c_str());
        return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::Pose goal_pose = waypoints[goal_name];

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose = goal_pose;

    ROS_INFO("📍 Sending navigation goal point: %s (x=%.2f, y=%.2f)",
             goal_name.c_str(), goal_pose.position.x, goal_pose.position.y);
    ac_.sendGoal(goal);
    return BT::NodeStatus::RUNNING;
}

// Implement the onRunning function
BT::NodeStatus MoveToGoal2::onRunning()
{
    if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("✅ Goal point reached");
        return BT::NodeStatus::SUCCESS;
    }
    else if (ac_.getState() == actionlib::SimpleClientGoalState::ABORTED)
    {
        ROS_WARN("❌ Failed to reach goal point");
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

// Implement the onHalted function
void MoveToGoal2::onHalted()
{
    ac_.cancelGoal();
    ROS_WARN("⏹️ Canceled the current goal");
}

// Register the node (for BehaviorTreeFactory)
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<MoveToGoal2>("MoveToGoal2");
}
