from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pid_controller',
            executable='pid_controller_node',
            name='pid_controller_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
        ),
        Node(
            package='joint_simulator',
            executable='joint_simulator_node',
            name='joint_simulator_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn'],
        ),
        Node(
            package='reference_input_node',
            executable='reference_input_node',
            name='reference_input_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
        ),
    ])
