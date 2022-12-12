#include "../include/amr_localization/odometry_calculator_node.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <chrono>

OdometryNode::OdometryNode(const std::string& node_name)
    : Node(node_name)
{
    using namespace std::placeholders;
    RCLCPP_INFO_ONCE(this->get_logger(), "Creating Odometry Node.");

    m_OdomBroadcaster = new tf2_ros::TransformBroadcaster(this);

    // Initialize Wheel Info member variable

    auto epoch = std::chrono::high_resolution_clock::from_time_t(0);
    auto now = std::chrono::high_resolution_clock::now();
    m_OdometryTimer = std::chrono::duration_cast<std::chrono::milliseconds>(now-epoch).count();

    m_WheelInfo.Rr = 0.01;
    m_WheelInfo.Ly = 0.01;
    m_WheelInfo.Lx = 0.01;

    // Create Subscriptions

    m_EncoderDataSubscriber = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/amr/encoder_data",
        rclcpp::SensorDataQoS(),
        std::bind(&OdometryNode::encoder_data_callback, this, _1)
    );

    // Create Publishers

    m_OdometryPublisher = this->create_publisher<nav_msgs::msg::Odometry>(
        "/amr/wheel/odom/unfiltered",
        10
    );

}

void OdometryNode::encoder_data_callback(const std_msgs::msg::Float32MultiArray::SharedPtr encoder_data)
{

    rclcpp::Time currentTime = this->get_clock()->now();

    m_Vx = calculate_Vx(*encoder_data);

    m_Vy = calculate_Vy(*encoder_data);

    m_Wz = calculate_Wz(*encoder_data);
    
    auto epoch = std::chrono::high_resolution_clock::from_time_t(0);
    auto now = std::chrono::high_resolution_clock::now();
    unsigned long odomCurrentTime = std::chrono::duration_cast<std::chrono::milliseconds>(now-epoch).count();
    float dt = (float)(odomCurrentTime - m_OdometryTimer);

    m_X += (m_Vx * std::cos(m_Th)) - (m_Vy * std::sin(m_Th));
    m_Y += (m_Vx * std::sin(m_Th) + m_Vy * std::cos(m_Th));
    m_Th += m_Wz;
    m_Th = std::fmod(m_Th, 2*M_PI);

    // Create Quaternion of rotations from the Yaw value (m_Th)
    tf2::Quaternion quatFromYaw;
    quatFromYaw.setRPY(0, 0, m_Th);
    
    // Copy contents of the tf2::Quaternion to a geometry_msgs::Quaternion
    geometry_msgs::msg::Quaternion odomQuaternion;
    odomQuaternion.x = quatFromYaw.getX();
    odomQuaternion.y = quatFromYaw.getY();
    odomQuaternion.z = quatFromYaw.getZ();
    odomQuaternion.w = quatFromYaw.getW();
    
    // Create the transform object to send via the TF2 Broadcaster
    geometry_msgs::msg::TransformStamped odomTransform;
    odomTransform.header.stamp = currentTime;
    odomTransform.header.frame_id = "odom";
    odomTransform.child_frame_id = "base_link";

    odomTransform.transform.translation.x = m_X;
    odomTransform.transform.translation.y = m_Y;
    odomTransform.transform.translation.z = 0.0;
    odomTransform.transform.rotation = odomQuaternion;
    
    // Send odometry transform
    m_OdomBroadcaster->sendTransform(odomTransform);

    // Prepare a nav_msgs::Odometry message to publish over ROS2 Network

    nav_msgs::msg::Odometry odomMsg;
    odomMsg.header.stamp = currentTime;
    odomMsg.header.frame_id = "odom";
    odomMsg.pose.pose.position.x = m_X;
    odomMsg.pose.pose.position.y = m_Y;
    odomMsg.pose.pose.position.z = 0.0;
    odomMsg.pose.pose.orientation = odomQuaternion;
    odomMsg.child_frame_id = "base_link";
    odomMsg.twist.twist.linear.x = m_Vx / dt;
    odomMsg.twist.twist.linear.y = m_Vy / dt;
    odomMsg.twist.twist.angular.z = m_Wz / dt;

    m_OdometryPublisher->publish(odomMsg);

    if((odomCurrentTime - m_OdomFrequency) > (m_OdometryTimer + m_OdomFrequency))
        m_OdometryTimer = odomCurrentTime;
    else
        m_OdometryTimer = m_OdometryTimer + m_OdomFrequency;

}

double OdometryNode::calculate_Vx(const std_msgs::msg::Float32MultiArray encoder_data)
{   
    
    float front_left_wheel_encoder = encoder_data.data[0];
    float front_right_wheel_encoder = encoder_data.data[1];
    float rear_left_wheel_encoder = encoder_data.data[2];
    float rear_right_wheel_encoder = encoder_data.data[3];

    return ((front_left_wheel_encoder + front_right_wheel_encoder + rear_left_wheel_encoder + rear_right_wheel_encoder) * (m_WheelInfo.Rr/4));
}

double OdometryNode::calculate_Vy(const std_msgs::msg::Float32MultiArray encoder_data)
{

    float front_left_wheel_encoder = encoder_data.data[0];
    float front_right_wheel_encoder = encoder_data.data[1];
    float rear_left_wheel_encoder = encoder_data.data[2];
    float rear_right_wheel_encoder = encoder_data.data[3];

    return ((-front_left_wheel_encoder + front_right_wheel_encoder + rear_left_wheel_encoder - rear_right_wheel_encoder) * (m_WheelInfo.Rr / 4));
}

double OdometryNode::calculate_Wz(const std_msgs::msg::Float32MultiArray encoder_data)
{
    float front_left_wheel_encoder = encoder_data.data[0];
    float front_right_wheel_encoder = encoder_data.data[1];
    float rear_left_wheel_encoder = encoder_data.data[2];
    float rear_right_wheel_encoder = encoder_data.data[3];
    
    return ((-front_left_wheel_encoder + front_right_wheel_encoder - rear_left_wheel_encoder + rear_right_wheel_encoder) * (m_WheelInfo.Rr / (4 * (m_WheelInfo.Lx + m_WheelInfo.Ly))));
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<OdometryNode> odometry_node = std::make_shared<OdometryNode>("odometry_node");
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(odometry_node);

    exec.spin();

    rclcpp::shutdown();
    return 0; 

}