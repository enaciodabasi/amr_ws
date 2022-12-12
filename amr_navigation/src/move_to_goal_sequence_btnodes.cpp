#include "../include/amr_navigation/move_to_goal_sequence_btnodes.hpp"

GetWaypoint::GetWaypoint(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
{

}

BT::NodeStatus GetWaypoint::tick()
{
    m_Node = std::make_shared<rclcpp::Node>("get_bt_position_goal_node");

    m_BtPositionSub = m_Node->create_subscription<amr_interfaces::msg::BtPositionMsg>(
        "bt_topics/position_goal",
        rclcpp::SystemDefaultsQoS(),
        std::bind()
    )

    m_Executor.add_node(m_Node);

    m_Executor.spin_once();
}



