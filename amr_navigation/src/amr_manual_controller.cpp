#include "../include/amr_navigation/amr_manual_controller.hpp"

ManualControllerNode::ManualControllerNode()
    : Node("manual_controller_node")
{

    using namespace std::placeholders;

    RCLCPP_INFO_ONCE(this->get_logger(), "Creating manual control node.");

    m_SpeedCommandSub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/amr/manual_controller/speed_command",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&ManualControllerNode::callback_speed_command, this, _1)    
    );

    m_TwistPublisher = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10
    );
    
}

void ManualControllerNode::callback_speed_command(const std_msgs::msg::Float64MultiArray::SharedPtr speed_command_msg)
{
    geometry_msgs::msg::Twist twistMsg = speedCommandToTwist(*speed_command_msg);

    RCLCPP_INFO(this->get_logger(), "Publishing Twist command at /cmd_vel");
    
    m_TwistPublisher->publish(twistMsg);
}

geometry_msgs::msg::Twist ManualControllerNode::speedCommandToTwist(const std_msgs::msg::Float64MultiArray& speed_command)
{

    geometry_msgs::msg::Twist tempTwist;

    tempTwist.linear.x = speed_command.data[0];
    tempTwist.linear.y = speed_command.data[1];
    tempTwist.linear.z = speed_command.data[2];

    tempTwist.angular.x = speed_command.data[3];
    tempTwist.angular.y = speed_command.data[4];
    tempTwist.angular.z = speed_command.data[5];

    return tempTwist;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<ManualControllerNode> node = std::make_shared<ManualControllerNode>();

    rclcpp::executors::SingleThreadedExecutor exc;
    exc.add_node(node);

    exc.spin();

    exc.remove_node(node);
    rclcpp::shutdown();

    return 0;
}