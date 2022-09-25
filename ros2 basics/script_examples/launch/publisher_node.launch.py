from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_learning_pkg',
            executable='publisher.py',
            name="hello_world_node "
        ),
        ExecuteProcess(
            cmd=['ros2', 'topic', 'list'],
            output='screen'
        )
    ])