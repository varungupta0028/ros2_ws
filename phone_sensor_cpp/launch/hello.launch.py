from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="phone_sensor_cpp",
            executable="hello_publisher",
            name="talker",
            output= "screen"
        )
    ])  