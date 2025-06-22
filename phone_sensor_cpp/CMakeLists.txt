cmake_minimum_required(VERSION 3.8)
project(phone_sensor_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosbridge_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(sensor_publisher src/sensor_publisher.cpp )
ament_target_dependencies(sensor_publisher rclcpp sensor_msgs geometry_msgs rosbridge_msgs)

add_executable(sensor_subscriber src/sensor_subscriber.cpp)
ament_target_dependencies(sensor_subscriber rclcpp sensor_msgs geometry_msgs)

add_executable(gps_publisher src/gps_publisher.cpp)
ament_target_dependencies(gps_publisher rclcpp sensor_msgs Boost)

add_executable(gps_subscriber src/gps_subscriber.cpp)
ament_target_dependencies(gps_subscriber rclcpp sensor_msgs )

add_executable(sensor_receiver src/sensor_receiver.cpp)
ament_target_dependencies(sensor_receiver rclcpp sensor_msgs geometry_msgs rosbridge_msgs)

add_executable(odom_subscriber src/odom_subscriber.cpp)
ament_target_dependencies(odom_subscriber rclcpp nav_msgs)

add_executable(odom_publisher src/odom_publisher.cpp)
ament_target_dependencies(odom_publisher rclcpp nav_msgs)

add_executable(hello_publisher src/hello_publisher.cpp)
ament_target_dependencies(hello_publisher rclcpp std_msgs)


install(TARGETS
  sensor_publisher
  sensor_subscriber
  gps_publisher
  gps_subscriber
  sensor_receiver 
  odom_subscriber
  odom_publisher
  hello_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch  
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
