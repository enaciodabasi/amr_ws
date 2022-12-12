#include "../include/amr_hardware_interface/amr_velocity_publisher.hpp"

VelocityPublisher::VelocityPublisher()
    : Node("amr_velocity_publisher")
{

    using namespace std::placeholders;

    m_TwistSubscriber = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10,
        std::bind(&VelocityPublisher::twist_callback, this, _1)
    );

    m_VelocityPublisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/amr/new_wheel_velocities",
        rclcpp::SystemDefaultsQoS()
    );

}

void VelocityPublisher::twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
{

    WheelVelocities newVelocities = twistToWheelVelocity(*twist_msg);

    std_msgs::msg::Float64MultiArray velMsg;
    velMsg.data.resize(4);
    velMsg.data[0] = newVelocities.front_left_wheel_velocity;
    velMsg.data[1] = newVelocities.front_right_wheel_velocity;
    velMsg.data[2] = newVelocities.rear_left_wheel_velocity;
    velMsg.data[3] = newVelocities.rear_right_wheel_velocity;

    m_VelocityPublisher->publish(velMsg);

}

VelocityPublisher::WheelVelocities VelocityPublisher::twistToWheelVelocity(const geometry_msgs::msg::Twist twist_msg)
{
    double front_left_wheel_vel = (1 / m_WheelRadius) * (twist_msg.linear.x - twist_msg.linear.y - ((m_WheelSeparationWidth + m_WheelSeparationLength) * twist_msg.angular.z));
    double front_right_wheel_vel = (1 / m_WheelRadius) * (twist_msg.linear.x + twist_msg.linear.y + ((m_WheelSeparationWidth + m_WheelSeparationLength) * twist_msg.angular.z));
    double rear_left_wheel_vel = (1 / m_WheelRadius) * (twist_msg.linear.x + twist_msg.linear.y - ((m_WheelSeparationWidth + m_WheelSeparationLength) * twist_msg.angular.z));
    double rear_right_wheel_vel = (1 / m_WheelRadius) * (twist_msg.linear.x - twist_msg.linear.y + ((m_WheelSeparationWidth + m_WheelSeparationLength) * twist_msg.angular.z));

    WheelVelocities wheelVelocities;
    wheelVelocities.front_left_wheel_velocity = front_left_wheel_vel;
    wheelVelocities.front_right_wheel_velocity = front_right_wheel_vel;
    wheelVelocities.rear_left_wheel_velocity = rear_left_wheel_vel;
    wheelVelocities.rear_right_wheel_velocity = rear_right_wheel_vel;

    return wheelVelocities;

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<VelocityPublisher> odometry_node = std::make_shared<VelocityPublisher>();
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(odometry_node);
    exec.spin();

    rclcpp::shutdown();
    return 0; 
}