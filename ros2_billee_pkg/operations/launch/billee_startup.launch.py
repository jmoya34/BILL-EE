from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_learning_pkg',
            executable='camera_server.py',
            name="camera_server_publisher"
        ),
        Node(
            package='ros_learning_pkg',
            executable='antenna_reciver.py',
            name="controller_inputs_pub_node"
        )
        # ExecuteProcess(
        #     cmd=['ros2', 'topic', 'list'],
        #     output='screen'
        # )
    ])