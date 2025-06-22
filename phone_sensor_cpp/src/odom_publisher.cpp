#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdomPublisherNode : public rclcpp::Node
{
public:
    OdomPublisherNode() : Node("odom_publisher")
    {   
        x_ = 0.1;
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        RCLCPP_INFO(this->get_logger(), "Odom publisher node has been started");
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&OdomPublisherNode::publish_odom, this));
    }

private: 

    void publish_odom(){
        auto msg = nav_msgs::msg::Odometry();
        msg.pose.pose.position.x = x_;
        msg.pose.pose.position.y = 0.0;
        msg.pose.pose.position.z = 0.0;

        odom_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published Odometry: x= %.2f", x_);
        x_ += 0.1;
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double x_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}   