#include "../include/amr_navigation/bt_nodes.hpp"

Nav2Client::Nav2Client(const std::string& name, const BT::NodeConfiguration& config)
    : BT::AsyncActionNode(name, config)
{

}

BT::NodeStatus Nav2Client::tick()
{
    m_Node = std::make_shared<rclcpp::Node>("nav2_client");

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigateToPoseClient = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        m_Node,
        "navigate_to_pose"
    );
    
    // Connect to the action server "navigate_to_pose"
    if(!navigateToPoseClient->wait_for_action_server(std::chrono::microseconds(5000)))
    {
        RCLCPP_ERROR(m_Node->get_logger(), "Action server Navigate to Pose is not available after 5 seconds. Aborting...");
        return BT::NodeStatus::FAILURE;
    }

    Position position;
    // Check input from BT Port
    if(!getInput<Position>("goal", position))
    {
        throw BT::RuntimeError("missing required input [goal]");
    }

    m_Aborted = false;

    // Create action goal for action server "navigate_to_pose"
    nav2_msgs::action::NavigateToPose::Goal goalMsg;
    goalMsg.pose.header.frame_id = m_TargetFrameID;
    goalMsg.pose.header.stamp = m_Node->get_clock()->now();
    goalMsg.pose.pose.position.x = position.pos_x;
    goalMsg.pose.pose.position.y = position.pos_y;
    goalMsg.pose.pose.position.z = position.pos_z;
    goalMsg.pose.pose.orientation.x = position.ort_x;
    goalMsg.pose.pose.orientation.y = position.ort_y;
    goalMsg.pose.pose.orientation.z = position.ort_z;
    goalMsg.pose.pose.orientation.w = position.ort_w;

    auto goalHandleFuture = navigateToPoseClient->async_send_goal(goalMsg);
    if(rclcpp::spin_until_future_complete(m_Node, goalHandleFuture) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(m_Node->get_logger(), "Could not send goal to server.");
        return BT::NodeStatus::FAILURE;
    }

    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goalHandle = goalHandleFuture.get();
    if(!goalHandle)
    {
        RCLCPP_ERROR(m_Node->get_logger(), "Goal was rejected by the server.");
        return BT::NodeStatus::FAILURE;
    }

    auto resultFuture = navigateToPoseClient->async_get_result(goalHandle);

    if(rclcpp::spin_until_future_complete(m_Node, resultFuture) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(m_Node->get_logger(), "Could not get result.");
        return BT::NodeStatus::FAILURE;
    }

    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult wrappedResult = resultFuture.get();

    if(wrappedResult.code == rclcpp_action::ResultCode::SUCCEEDED)
    {

    }
    else if(wrappedResult.code == rclcpp_action::ResultCode::ABORTED)
    {
        RCLCPP_ERROR(m_Node->get_logger(), "Goal was aborted.");
        return BT::NodeStatus::FAILURE;
    }    
    else if(wrappedResult.code == rclcpp_action::ResultCode::CANCELED)
    {
        RCLCPP_ERROR(m_Node->get_logger(), "Goal was canceled.");
        return BT::NodeStatus::FAILURE;
    }
    else
    {
        RCLCPP_ERROR(m_Node->get_logger(), "Unknown result code.");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_ERROR(m_Node->get_logger(), "Goal achieved.");
    return BT::NodeStatus::SUCCESS;
}

// ------- Battery Conditions ------- 

BT::NodeStatus CheckBattery()
{
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("check_battery");

    bool* isBatteryOk = new bool();

    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr batteryStateSub = node->create_subscription<sensor_msgs::msg::BatteryState>(
        "/amr/battery_state",
        rclcpp::SensorDataQoS(),
        [node, isBatteryOk](const sensor_msgs::msg::BatteryState::SharedPtr msg){
            
            float batteryPercentage = msg->percentage;

            if(batteryPercentage < 0.05)
            {
                *isBatteryOk = false;
            }
            else
            {
                *isBatteryOk = true;
            }
        }
    );

    rclcpp::executors::SingleThreadedExecutor exc;
    exc.add_node(node);

    exc.spin_once();

    if(!*isBatteryOk)
    {
        return BT::NodeStatus::FAILURE;
    }

    exc.remove_node(node);
    delete isBatteryOk;
    return BT::NodeStatus::SUCCESS;

}

BT::NodeStatus IsBatteryFull()
{
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("check_battery");

    bool* isBatteryFull = new bool();

    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr batteryStateSub = node->create_subscription<sensor_msgs::msg::BatteryState>(
        "/amr/battery_state",
        rclcpp::SensorDataQoS(),
        [node, isBatteryFull](const sensor_msgs::msg::BatteryState::SharedPtr msg){
            
            float batteryPercentage = msg->percentage;

            if(batteryPercentage >= 0.99)
            {
                *isBatteryFull = true;
            }
            else
            {
                *isBatteryFull = false;
            }
        }
    );

    rclcpp::executors::SingleThreadedExecutor exc;
    exc.add_node(node);

    exc.spin_once();

    if(!*isBatteryFull)
    {
        return BT::NodeStatus::SUCCESS;
    }

    exc.remove_node(node);
    delete isBatteryFull;
    return BT::NodeStatus::FAILURE;
}