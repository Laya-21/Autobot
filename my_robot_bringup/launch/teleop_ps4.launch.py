from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Joystick driver node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),

        # Teleop Twist Joy node
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[{
                'joy_config': 'ps4',
                'axis_linear.x': 1,  # Left joystick vertical axis
                'axis_angular.yaw': 0,  # Left joystick horizontal axis
                'scale_linear.x': 2.6,
                'scale_angular.yaw': 2.5,
                'enable_button': 7,  # R2 button
            }],
        ),
    ])
