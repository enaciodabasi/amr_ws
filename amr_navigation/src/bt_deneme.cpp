#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"


#include <iostream>
#include <chrono>

struct Position
{
    double pos_x;
    double pos_y;
    double pos_z;

    double orient_x;
    double orient_y;
    double orient_z;
    double orient_w;
};

namespace BT
{
    template<> inline Position convertFromString(StringView str)
    {
        auto parts = splitString(str, ';');

        if(parts.size() != 7)
        {
            throw RuntimeError("invalid input");
        }
        else
        {
            Position output;
            output.pos_x = convertFromString<double>(parts[0]);
            output.pos_y = convertFromString<double>(parts[1]);
            output.pos_z = convertFromString<double>(parts[2]);
            output.orient_x = convertFromString<double>(parts[3]);
            output.orient_y = convertFromString<double>(parts[4]);
            output.orient_z = convertFromString<double>(parts[5]);
            output.orient_w = convertFromString<double>(parts[6]);

            return output;
        }
    }
}

class MoveToChargingPort : public BT::StatefulActionNode
{
    public:

    MoveToChargingPort(const std::string& name, const BT::NodeConfiguration& config)
        : StatefulActionNode(name, config)
    {

    }


    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override;

    private:

    Position m_ChargingPortPosition;

    std::chrono::system_clock::time_point m_CompletionTime;

};

BT::NodeStatus MoveToChargingPort::onStart()
{
    m_CompletionTime = std::chrono::system_clock::now() + std::chrono::milliseconds(300);
    return BT::NodeStatus::RUNNING;
}



class Charge : public BT::StatefulActionNode
{
    public:

    Charge(const std::string& name, const BT::NodeConfiguration& config)
        : StatefulActionNode(name, config)
    {

    }

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override;

    private:


};

class MoveToGoal : public BT::StatefulActionNode
{
    public:

    MoveToGoal(const std::string& name, const BT::NodeConfiguration& config)
        : StatefulActionNode(name, config)
    {

    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<Position>("goal_pos")};
    }

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override;

    private:

    Position m_Goal;

    std::chrono::system_clock::time_point m_CompletionTime;
    
};

BT::NodeStatus MoveToGoal::onStart()
{
    if(!getInput<Position>("goal_pos", m_Goal))
    {
        throw BT::RuntimeError("Missing Goal Input");
    }

    printf("Goal Request: position: x: %f, y: %f, z:%f   orientation: x: %f, y: %f, z: %f, w: %f \n", m_Goal.pos_x, m_Goal.pos_y, m_Goal.pos_z, m_Goal.orient_x, m_Goal.orient_y, m_Goal.orient_z, m_Goal.orient_w);

    m_CompletionTime = std::chrono::system_clock::now() + std::chrono::milliseconds(200);

    return BT::NodeStatus::RUNNING;

}

BT::NodeStatus MoveToGoal::onRunning()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    if(std::chrono::system_clock::now() >= m_CompletionTime)
    {
        std::cout << "Goal achieved." << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void MoveToGoal::onHalted()
{
    std::cout << "Aborted" << std::endl;
}

class GetWaypoint : public BT::SyncActionNode
{
    public:

    GetWaypoint(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        std::cout << "Created getWaypoint Action Node" << std::endl;
    }

    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<Position>("goal_pos")};
    }

    BT::NodeStatus tick() override
    {
        Position goal;
        goal.pos_x = 1.1;
        goal.pos_y = 1.5;
        goal.pos_z = 0.0;
        goal.orient_x = 0.0;
        goal.orient_y = 0.0;
        goal.orient_z = 0.0;
        goal.orient_w = 1.2;
        setOutput<Position>("goal_pos", goal);

        std::cout << "Sending goal output" << std::endl;

        return BT::NodeStatus::SUCCESS;
    }
};

BT::NodeStatus isBatteryOk()
{
    std::cout << "Battery is OK" << std::endl;

    return BT::NodeStatus::SUCCESS;
}


int main()
{

    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<GetWaypoint>("getWaypoint");
    factory.registerSimpleCondition("isBatteryOk", std::bind(&isBatteryOk));

    factory.registerNodeType<MoveToGoal>("moveToGoal");

    auto tree = factory.createTreeFromFile("/home/naci/amr_ws/src/amr_navigation/bt_trees/bt_deneme.xml");

    while(true)
        tree.tickRootWhileRunning();

    return 0;

}

