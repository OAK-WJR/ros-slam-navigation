#include "my_behavior_tree/general_task/move_to_goal_2.h"
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <yaml-cpp/yaml.h>
#include <unordered_map>
#include <fstream>
#include <boost/filesystem.hpp>


// 定义全局变量 waypoints
std::unordered_map<std::string, geometry_msgs::Pose> waypoints;

// 实现 loadWaypoints 函数
void loadWaypoints(const std::string& filename)
{
  try
  {

    {
      // 使用局部作用域确保 yaml 对象尽早析构
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
    ROS_INFO("📡 读取 %lu 个目标点", waypoints.size());
  }
  catch(const std::exception &e)
  {
    ROS_ERROR("加载 waypoints 文件失败: %s", e.what());
    throw std::runtime_error(e.what());
  }
}



// 实现 MoveToGoal 的构造函数
MoveToGoal2::MoveToGoal2(const std::string &name, const BT::NodeConfiguration &config)
    : BT::StatefulActionNode(name, config), ac_("move_base", true)
{
}

// 实现静态函数 providedPorts
BT::PortsList MoveToGoal2::providedPorts()
{
    return {BT::InputPort<std::string>("goal")};
}

// 实现 onStart 函数
BT::NodeStatus MoveToGoal2::onStart()
{
    std::string goal_name;
    if (!getInput("goal", goal_name))
    {
        ROS_ERROR("MoveToGoal: 目标点未设置！");
        return BT::NodeStatus::FAILURE;
    }

    if (waypoints.find(goal_name) == waypoints.end())
    {
        ROS_ERROR("MoveToGoal: 目标点 %s 不存在！", goal_name.c_str());
        return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::Pose goal_pose = waypoints[goal_name];

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose = goal_pose;

    ROS_INFO("📍 发送导航目标点: %s (x=%.2f, y=%.2f)",
             goal_name.c_str(), goal_pose.position.x, goal_pose.position.y);
    ac_.sendGoal(goal);
    return BT::NodeStatus::RUNNING;
}

// 实现 onRunning 函数
BT::NodeStatus MoveToGoal2::onRunning()
{
    if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("✅ 目标点到达成功");
        return BT::NodeStatus::SUCCESS;
    }
    else if (ac_.getState() == actionlib::SimpleClientGoalState::ABORTED)
    {
        ROS_WARN("❌ 目标点到达失败");
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

// 实现 onHalted 函数
void MoveToGoal2::onHalted()
{
    ac_.cancelGoal();
    ROS_WARN("⏹️ 取消当前目标");
}

// 注册节点（供 BehaviorTreeFactory 使用）
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<MoveToGoal2>("MoveToGoal2");
}
