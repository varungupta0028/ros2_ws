#include "rclcpp/rclcpp.hpp" 
#include "nav_msgs/msg/odometry.hpp"

using namespace std::placeholders;

class OdomSubscriberNode : public rclcpp::Node
{
public:
    OdomSubscriberNode() : Node("odom_subscriber")  
    {   
        odom_subsriber_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&OdomSubscriberNode::odom_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "odom subscriber node has been started");
    }
private:

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double x_cordinate = msg->pose.pose.position.x;
        if (x_cordinate > 0.5) {
            RCLCPP_INFO(this->get_logger(), "X-Cordinate exceeded 50cm: %.2f meters", x_cordinate);
        }
        else {
            RCLCPP_INFO(this->get_logger(), "X-Cordinate is less than 50cm ");
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subsriber_ ;

};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}