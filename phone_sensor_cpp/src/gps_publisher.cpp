#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <boost/asio.hpp>
#include <sstream>
#include <thread>

using namespace boost::asio;

class GPSPublisher : public rclcpp::Node {
public:
    GPSPublisher() 
        : Node("gps_publisher"), socket_(io_service_, ip::udp::endpoint(ip::udp::v4(), 5006)) {
        
        gps_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
        io_thread_ = std::thread([this]() { io_service_.run(); }); // Run io_service in a separate thread
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), 
                    std::bind(&GPSPublisher::receive_data, this));
    }

    ~GPSPublisher() {
        io_service_.stop();
        if (io_thread_.joinable()) {
            io_thread_.join();
        }
    }

private:
    void receive_data() {
        try {
            char buffer[1024] = {0};  // Ensure buffer is zero-initialized
            ip::udp::endpoint sender_endpoint;
            size_t len = socket_.receive_from(boost::asio::buffer(buffer), sender_endpoint);

            std::string data(buffer, len);
            RCLCPP_DEBUG(this->get_logger(), "Raw GPS data received: %s", data.c_str());

            std::stringstream ss(data);
            double latitude, longitude, altitude;
            if (!(ss >> latitude >> longitude >> altitude)) {
                RCLCPP_WARN(this->get_logger(), "Invalid GPS data format received.");
                return;
            }

            auto gps_msg = sensor_msgs::msg::NavSatFix();
            gps_msg.latitude = latitude;
            gps_msg.longitude = longitude;
            gps_msg.altitude = altitude;
            gps_msg.header.stamp = this->now();
            gps_msg.header.frame_id = "gps";

            gps_publisher_->publish(gps_msg);
            RCLCPP_INFO(this->get_logger(), "Published GPS: [Lat: %.6f, Lon: %.6f, Alt: %.2f]", 
                        latitude, longitude, altitude);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error receiving GPS data: %s", e.what());
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    io_service io_service_;
    ip::udp::socket socket_;
    std::thread io_thread_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSPublisher>());
    rclcpp::shutdown();
    return 0;
}
