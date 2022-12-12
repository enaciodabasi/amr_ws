#ifndef MOVE_TO_GOAL_SEQUENCE_BTNODES_HPP
#define MOVE_TO_GOAL_SEQUENCE_BTNODES_HPP

#include <rclcpp/rclcpp.hpp>
#include <amr_interfaces/msg/bt_position_msg.hpp>
#include <amr_interfaces/srv/bt_position_srv.hpp>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"


#include <iostream>
#include <memory>
#include <chrono>

struct PositionGoal
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
    template<> inline PositionGoal convertFromString(StringView str)
    {
        auto parts = splitString(str, ';');

        if(parts.size() != 7)
        {
            throw RuntimeError("invalid input");
        }
        else
        {
            PositionGoal output;
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

class GetWaypoint : public BT::SyncActionNode
{
    public:

    GetWaypoint(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<PositionGoal>("goal_pos")};
    }

    BT::NodeStatus tick() override;

    private:

    rclcpp::executors::SingleThreadedExecutor m_Executor;

    std::shared_ptr<rclcpp::Node> m_Node;

    rclcpp::Subscription<amr_interfaces::msg::BtPositionMsg>::SharedPtr m_BtPositionSub;

    void callback_bt_position(const amr_interfaces::msg::BtPositionMsg::SharedPtr& msg);

};

class MoveToGoal : public BT::StatefulActionNode
{
    public:

    MoveToGoal(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<PositionGoal>("goal_pos")};
    }

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override;

    private:
    
};

#endif