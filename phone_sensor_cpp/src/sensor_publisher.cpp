// # include/phone_sensors_cpp/sensor_publisher.hpp:
#ifndef PHONE_SENSORS_CPP__SENSOR_PUBLISHER_HPP_
#define PHONE_SENSORS_CPP__SENSOR_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/illuminance.hpp>
#include <random>

class PhoneSensorPublisher : public rclcpp::Node
{
public:
  PhoneSensorPublisher();

private:
  void publish_imu();
  void publish_magnetic();
  void publish_light();

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magnetic_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Illuminance>::SharedPtr light_publisher_;
  
  rclcpp::TimerBase::SharedPtr imu_timer_;
  rclcpp::TimerBase::SharedPtr magnetic_timer_;
  rclcpp::TimerBase::SharedPtr light_timer_;
  
  std::random_device rd_;
  std::mt19937 gen_;
  std::normal_distribution<> imu_noise_;
  std::normal_distribution<> mag_noise_;
  std::normal_distribution<> light_noise_;
};

#endif  // PHONE_SENSORS_CPP__SENSOR_PUBLISHER_HPP_

// # src/sensor_publisher.cpp:
// #include "phone_sensor_cpp/sensor_publisher.hpp"

PhoneSensorPublisher::PhoneSensorPublisher()
: Node("phone_sensor_publisher"),
  gen_(rd_()),
  imu_noise_(0.0, 0.1),
  mag_noise_(0.0, 1.0),
  light_noise_(250.0, 10.0)
{
  // Create publishers
  imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
    "phone/imu", 10);
  magnetic_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>(
    "phone/magnetic", 10);
  light_publisher_ = this->create_publisher<sensor_msgs::msg::Illuminance>(
    "phone/light", 10);

  // Create timers
  imu_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),  // 10 Hz
    std::bind(&PhoneSensorPublisher::publish_imu, this));
  magnetic_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(200),  // 5 Hz
    std::bind(&PhoneSensorPublisher::publish_magnetic, this));
  light_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),  // 1 Hz
    std::bind(&PhoneSensorPublisher::publish_light, this));

  RCLCPP_INFO(this->get_logger(), "Phone sensor publisher has started");
}

void PhoneSensorPublisher::publish_imu()
{
  auto message = sensor_msgs::msg::Imu();
  message.header.stamp = this->now();
  message.header.frame_id = "phone_imu_frame";

  // Mock IMU data - replace with actual phone sensor data
  message.linear_acceleration.x = imu_noise_(gen_);
  message.linear_acceleration.y = 9.81 + imu_noise_(gen_);  // Gravity
  message.linear_acceleration.z = imu_noise_(gen_);

  message.angular_velocity.x = imu_noise_(gen_);
  message.angular_velocity.y = imu_noise_(gen_);
  message.angular_velocity.z = imu_noise_(gen_);

  imu_publisher_->publish(message);
}

void PhoneSensorPublisher::publish_magnetic()
{
  auto message = sensor_msgs::msg::MagneticField();
  message.header.stamp = this->now();
  message.header.frame_id = "phone_mag_frame";

  // Mock magnetic field data - replace with actual phone sensor data
  message.magnetic_field.x = 25.0 + mag_noise_(gen_);
  message.magnetic_field.y = 40.0 + mag_noise_(gen_);
  message.magnetic_field.z = -10.0 + mag_noise_(gen_);

  magnetic_publisher_->publish(message);
}

void PhoneSensorPublisher::publish_light()
{
  auto message = sensor_msgs::msg::Illuminance();
  message.header.stamp = this->now();
  message.header.frame_id = "phone_light_frame";

  // Mock light sensor data - replace with actual phone sensor data
  message.illuminance = light_noise_(gen_);
  message.variance = 5.0;

  light_publisher_->publish(message);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PhoneSensorPublisher>());
  rclcpp::shutdown();
  return 0;
}
