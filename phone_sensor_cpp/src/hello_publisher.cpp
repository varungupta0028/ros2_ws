#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class HelloPublisherNode : public rclcpp::Node
{
public:
    HelloPublisherNode() : Node("talker")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
        timer_= this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&HelloPublisherNode::publish_hello, this));
    }
private:

    void publish_hello() {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello, ROS2!";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HelloPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}