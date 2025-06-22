#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class GPSSubscriber : public rclcpp::Node {
public:
    GPSSubscriber() : Node("gps_subscriber") {
        gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", 10, std::bind(&GPSSubscriber::gps_callback, this, std::placeholders::_1));
    }

private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received GPS Data: Lat: %.6f, Lon: %.6f, Alt: %.2f",
                    msg->latitude, msg->longitude, msg->altitude);
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSSubscriber>());
    rclcpp::shutdown();
    return 0;
}
