#ifndef PHONE_SENSORS_CPP__SENSOR_RECEIVER_HPP_
#define PHONE_SENSORS_CPP__SENSOR_RECEIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/illuminance.hpp>

class PhoneSensorReceiver : public rclcpp::Node
{
public:
    PhoneSensorReceiver();

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void magnetic_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg);
    void light_callback(const sensor_msgs::msg::Illuminance::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr magnetic_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Illuminance>::SharedPtr light_subscription_;

    // Publishers to republish the data (optional)
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magnetic_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Illuminance>::SharedPtr light_publisher_;
};

#endif  // PHONE_SENSORS_CPP__SENSOR_RECEIVER_HPP_

// phone_sensors_cpp/src/sensor_receiver.cpp
// #include "phone_sensors_cpp/sensor_receiver.hpp"

PhoneSensorReceiver::PhoneSensorReceiver()
: Node("phone_sensor_receiver")
{
    // Create subscriptions for topics that will be published by the phone
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/phone/imu", 10,
        std::bind(&PhoneSensorReceiver::imu_callback, this, std::placeholders::_1));
    
    magnetic_subscription_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
        "/phone/magnetic", 10,
        std::bind(&PhoneSensorReceiver::magnetic_callback, this, std::placeholders::_1));
    
    light_subscription_ = this->create_subscription<sensor_msgs::msg::Illuminance>(
        "/phone/light", 10,
        std::bind(&PhoneSensorReceiver::light_callback, this, std::placeholders::_1));

    // Create publishers (optional, if you want to republish the data)
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "/phone/imu/filtered", 10);
    magnetic_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>(
        "/phone/magnetic/filtered", 10);
    light_publisher_ = this->create_publisher<sensor_msgs::msg::Illuminance>(
        "/phone/light/filtered", 10);

    RCLCPP_INFO(this->get_logger(), "Phone sensor receiver has started");
}

void PhoneSensorReceiver::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(),
        "Received IMU: acc_x=%.2f, acc_y=%.2f, acc_z=%.2f",
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z);
    
    // Optionally republish the data (you can add filtering here)
    imu_publisher_->publish(*msg);
}

void PhoneSensorReceiver::magnetic_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(),
        "Received Magnetic: x=%.2f, y=%.2f, z=%.2f",
        msg->magnetic_field.x,
        msg->magnetic_field.y,
        msg->magnetic_field.z);
    
    magnetic_publisher_->publish(*msg);
}

void PhoneSensorReceiver::light_callback(const sensor_msgs::msg::Illuminance::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(),
        "Received Light: %.2f lux",
        msg->illuminance);
    
    light_publisher_->publish(*msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PhoneSensorReceiver>());
    rclcpp::shutdown();
    return 0;
}