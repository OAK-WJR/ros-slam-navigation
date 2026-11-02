#ifndef MOVE_TO_GOAL_H2
#define MOVE_TO_GOAL_H2

#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <yaml-cpp/yaml.h>
#include <unordered_map>
#include <fstream>

using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
extern std::unordered_map<std::string, geometry_msgs::Pose> waypoints;

// Declare the loadWaypoints implementation
void loadWaypoints(const std::string& filename);

class MoveToGoal2 : public BT::StatefulActionNode
{
public:
    MoveToGoal2(const std::string &name, const BT::NodeConfiguration &config);

    // Static interface, used to register ports
    static BT::PortsList providedPorts();

    // Virtual functions a BT node must implement
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    MoveBaseClient ac_;
};

#endif // MOVE_TO_GOAL_H
