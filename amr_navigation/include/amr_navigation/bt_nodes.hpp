#ifndef BT_NODES_HPP
#define BT_NODES_HPP

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"


#include <iostream>
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose.hpp>

struct Position
{
    double pos_x;
    double pos_y;
    double pos_z;

    double ort_x;
    double ort_y;
    double ort_z;
    double ort_w;
};

namespace BT
{
    template<> inline
    Position convertFromString(StringView key)
    {
        auto parts = BT::splitString(key, ';');
        if(parts.size() != 7)
        {
            throw BT::RuntimeError("invalid input");
        }
        else
        {
            Position output;

            output.pos_x = convertFromString<double>(parts[0]);
            output.pos_y = convertFromString<double>(parts[1]);
            output.pos_z = convertFromString<double>(parts[2]);
            output.ort_x = convertFromString<double>(parts[3]);
            output.ort_y = convertFromString<double>(parts[4]);
            output.ort_z = convertFromString<double>(parts[5]);
            output.ort_w = convertFromString<double>(parts[6]);

            return output;
        }

    }
}

class Nav2Client : public BT::AsyncActionNode
{
    public:

    Nav2Client(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<Position>("goal")};
    }

    virtual BT::NodeStatus tick() override;

    virtual void halt() override
    {
        m_Aborted = true;
    }

    private:

    bool m_Aborted;

    rclcpp::Node::SharedPtr m_Node;

    std::string m_TargetFrameID = "map";
};

//void battery_sub_callback()

BT::NodeStatus CheckBattery();

BT::NodeStatus IsBatteryFull();

#endif