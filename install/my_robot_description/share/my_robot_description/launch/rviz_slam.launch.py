from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config_name = LaunchConfiguration('rviz_config')

    # Path to RViz config: <pkg_share>/config/<rviz_config_name>
    rviz_config = PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        'config',
        rviz_config_name
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],      # this is now a valid substitution list
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='display.rviz',
            description='RViz config file name in the config/ directory',
        ),
        rviz_node,
    ])
