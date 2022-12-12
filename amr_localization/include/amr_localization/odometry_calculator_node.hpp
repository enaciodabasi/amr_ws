#pragma once

#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <amr_interfaces/msg/encoder_data.hpp>

class OdometryNode : public rclcpp::Node
{   

    public:
    
    OdometryNode(const std::string& node_name);

    // Private Member Variables
    private:

    struct
    {
        double Rr; // Radius of the wheels [m]

        double Lx; // Half of the distance between front wheels [m]
        
        double Ly; // Half of the distance between front wheels and rear wheels [m]

    } m_WheelInfo;

    double m_Vx; // Longitudinal Velocity

    double m_Vy; // Transversel Velocity

    double m_Wz; // Angular Velocity

    double m_X; // X coordinate of the position of the robot

    double m_Y; // Y coordinate of the position of the robot

    double m_Th; // Th

    unsigned long m_OdometryTimer = 0;

    const int m_OdomFrequency = 50;
    
    tf2_ros::TransformBroadcaster* m_OdomBroadcaster;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_EncoderDataSubscriber;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_OdometryPublisher;

    rclcpp::TimerBase::SharedPtr m_OdometryPublisherTimer;

    // Private Member Functions 
    private:

    double calculate_Vx(const std_msgs::msg::Float32MultiArray encoder_data);
    double calculate_Vy(const std_msgs::msg::Float32MultiArray encoder_data);
    double calculate_Wz(const std_msgs::msg::Float32MultiArray encoder_data);

    // Callback Function for listening to encoder data published from the hardware interface
    void encoder_data_callback(const std_msgs::msg::Float32MultiArray::SharedPtr encoder_data);

};
