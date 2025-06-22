from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Launch ROSbridge server
        ExecuteProcess(
            cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
            output='screen'
        ),
        
        # Launch our sensor receiver
        Node(
            package='phone_sensors_cpp',
            executable='sensor_receiver',
            name='sensor_receiver',
            output='screen'
        )
    ])
