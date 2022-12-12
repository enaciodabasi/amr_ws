#ifndef NAV2_CLIENT_HPP
#define NAV2_CLIENT_HPP

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "behaviortree_cpp_v3/action_node.h"

class Nav2Client : public BT::AsyncActionNode
{
    public:

    Nav2Client(const std::string& name, const BT::NodeConfiguration& config);

    
};

#endif