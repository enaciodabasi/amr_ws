#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class ManualControllerNode : public rclcpp::Node
{
    public:

    ManualControllerNode();

    private:

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_SpeedCommandSub;    

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_TwistPublisher;

    void callback_speed_command(const std_msgs::msg::Float64MultiArray::SharedPtr speed_command_msg);

    geometry_msgs::msg::Twist speedCommandToTwist(const std_msgs::msg::Float64MultiArray& speed_command);
};