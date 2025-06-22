// # include/phone_sensors_cpp/sensor_subscriber.hpp:
#ifndef PHONE_SENSORS_CPP__SENSOR_SUBSCRIBER_HPP_
#define PHONE_SENSORS_CPP__SENSOR_SUBSCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/illuminance.hpp>

class PhoneSensorSubscriber : public rclcpp::Node
{
public:
  PhoneSensorSubscriber();

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void magnetic_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg);
  void light_callback(const sensor_msgs::msg::Illuminance::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr magnetic_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Illuminance>::SharedPtr light_subscription_;
};

#endif  // PHONE_SENSORS_CPP__SENSOR_SUBSCRIBER_HPP_

// # src/sensor_subscriber.cpp:
// #include "phone_sensor_cpp/sensor_subscriber.hpp"

PhoneSensorSubscriber::PhoneSensorSubscriber()
: Node("phone_sensor_subscriber")
{
  // Create subscriptions
  imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "phone/imu", 10,
    std::bind(&PhoneSensorSubscriber::imu_callback, this, std::placeholders::_1));
  
  magnetic_subscription_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
    "phone/magnetic", 10,
    std::bind(&PhoneSensorSubscriber::magnetic_callback, this, std::placeholders::_1));
  
  light_subscription_ = this->create_subscription<sensor_msgs::msg::Illuminance>(
    "phone/light", 10,
    std::bind(&PhoneSensorSubscriber::light_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Phone sensor subscriber has started");
}

void PhoneSensorSubscriber::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(),
    "IMU: acc_x=%.2f, acc_y=%.2f, acc_z=%.2f, gyro_x=%.2f, gyro_y=%.2f, gyro_z=%.2f",
    msg->linear_acceleration.x,
    msg->linear_acceleration.y,
    msg->linear_acceleration.z,
    msg->angular_velocity.x,
    msg->angular_velocity.y,
    msg->angular_velocity.z);
}

void PhoneSensorSubscriber::magnetic_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(),
    "Magnetic Field: x=%.2f, y=%.2f, z=%.2f",
    msg->magnetic_field.x,
    msg->magnetic_field.y,
    msg->magnetic_field.z);
}

void PhoneSensorSubscriber::light_callback(const sensor_msgs::msg::Illuminance::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(),
    "Light: illuminance=%.2f lux, variance=%.2f",
    msg->illuminance,
    msg->variance);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PhoneSensorSubscriber>());
  rclcpp::shutdown();
  return 0;
}