#pragma once

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <vector>
#include <memory>

class VelocityPublisher : public rclcpp::Node
{
    public:

    VelocityPublisher();

    private:

    struct WheelVelocities
    {
        double front_left_wheel_velocity;
        double front_right_wheel_velocity;
        double rear_left_wheel_velocity;
        double rear_right_wheel_velocity;
    };

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_TwistSubscriber;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_VelocityPublisher;

    int m_WheelSeparationWidth;

    int m_WheelSeparationLength;

    int m_WheelRadius;
    
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg);

    WheelVelocities twistToWheelVelocity(const geometry_msgs::msg::Twist twist_msg);

};