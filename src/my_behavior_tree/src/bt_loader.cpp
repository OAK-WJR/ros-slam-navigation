#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "my_behavior_tree/general_task/MoveToGoal.h"

// Changed the AutoLineUp node to a StatefulActionNode so it runs for 1 second before returning SUCCESS
class AutoLineUp : public BT::StatefulActionNode
{
public:
    AutoLineUp(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<int>("goal") };
    }

    // onStart() is called on the first tick; records the start time
    BT::NodeStatus onStart() override
    {
        int goal;
        if (!getInput<int>("goal", goal))
            throw BT::RuntimeError("Missing parameter [goal] in AutoLineUp");
        start_time_ = ros::Time::now();
        ROS_INFO("AutoLineUp started, goal = %d", goal);
        return BT::NodeStatus::RUNNING;
    }

    // onRunning() is called on later ticks; keeps returning RUNNING and only returns SUCCESS after 1 second
    BT::NodeStatus onRunning() override
    {
        double elapsed = (ros::Time::now() - start_time_).toSec();
        if(elapsed < 3.0)
        {
            ROS_INFO("AutoLineUp running, elapsed = %.2f", elapsed);
            return BT::NodeStatus::RUNNING;
        }
        ROS_INFO("AutoLineUp succeeded");
        return BT::NodeStatus::SUCCESS;
    }

    // onHalted() is called when the node is interrupted
    void onHalted() override
    {
        ROS_WARN("AutoLineUp halted");
    }

private:
    ros::Time start_time_;
};

// Other nodes stay synchronous, with sleep(1) added to demo a delay
class IncrementVariable : public BT::SyncActionNode
{
public:
    IncrementVariable(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<int>("variable"), BT::OutputPort<int>("variable") };
    }

    BT::NodeStatus tick() override
    {
        int var;
        if (!getInput<int>("variable", var))
            throw BT::RuntimeError("Missing parameter [variable] in IncrementVariable");
        var++;
        ROS_INFO("IncrementVariable: variable incremented to %d", var);
        setOutput("variable", var);
        return BT::NodeStatus::SUCCESS;
    }
};

class IntakeGP : public BT::SyncActionNode
{
public:
    IntakeGP(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("GP"), BT::InputPort<int>("level") };
    }

    BT::NodeStatus tick() override
    {
        ros::Duration(3.0).sleep();
        std::string gp;
        int level;
        if (!getInput<std::string>("GP", gp))
            throw BT::RuntimeError("Missing parameter [GP] in IntakeGP");
        if (!getInput<int>("level", level))
            throw BT::RuntimeError("Missing parameter [level] in IntakeGP");
        ROS_INFO("IntakeGP: GP = %s, level = %d", gp.c_str(), level);
        return BT::NodeStatus::SUCCESS;
    }
};

class LaunchGP : public BT::SyncActionNode
{
public:
    LaunchGP(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("GP"), BT::InputPort<int>("level") };
    }

    BT::NodeStatus tick() override
    {
        ros::Duration(3.0).sleep();
        std::string gp;
        int level;
        if (!getInput<std::string>("GP", gp))
            throw BT::RuntimeError("Missing parameter [GP] in LaunchGP");
        if (!getInput<int>("level", level))
            throw BT::RuntimeError("Missing parameter [level] in LaunchGP");
        ROS_INFO("LaunchGP: GP = %s, level = %d", gp.c_str(), level);
        return BT::NodeStatus::SUCCESS;
    }
};

class SetPose : public BT::SyncActionNode
{
public:
    SetPose(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("GP"),
                 BT::InputPort<int>("level"),
                 BT::InputPort<std::string>("part") };
    }

    BT::NodeStatus tick() override
    {
        ros::Duration(3.0).sleep();
        std::string gp, part;
        int level;
        if (!getInput<std::string>("GP", gp))
            throw BT::RuntimeError("Missing parameter [GP] in SetPose");
        if (!getInput<int>("level", level))
            throw BT::RuntimeError("Missing parameter [level] in SetPose");
        if (!getInput<std::string>("part", part))
            throw BT::RuntimeError("Missing parameter [part] in SetPose");
        ROS_INFO("SetPose: GP = %s, level = %d, part = %s", gp.c_str(), level, part.c_str());
        return BT::NodeStatus::SUCCESS;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bt_ros_node");
    ros::NodeHandle nh;

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<AutoLineUp>("AutoLineUp"); // Use the AutoLineUp that was made stateful
    factory.registerNodeType<IncrementVariable>("IncrementVariable");
    factory.registerNodeType<IntakeGP>("IntakeGP");
    factory.registerNodeType<LaunchGP>("LaunchGP");
    factory.registerNodeType<MoveToGoal>("MoveToGoal");
    factory.registerNodeType<SetPose>("SetPose");

    // Note: make sure the XML file path is correct
    std::string xml_file = "/root/catkin_ws/src/my_behavior_tree/config/tree8.xml";
    auto tree = factory.createTreeFromFile(xml_file);

    // Use PublisherZMQ to publish logs for Groot visualization (ports must be different)
    BT::PublisherZMQ publisher(tree, 25, 1666, 1667);

    ros::Rate rate(10);

    while (ros::ok())
    {
        tree.tickRoot();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
