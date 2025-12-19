from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_bot_perception',
            executable='circle_ball_node',
            name='circle_ball_node',
            output='screen'
        ),
        Node(
            package='my_bot_perception',
            executable='ball_goal_transformer',
            name='ball_goal_transformer',
            output='screen'
        ),
    ])
