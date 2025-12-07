from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    # 1) Include the assessment world + spheres
    assessment_complete = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('assessment_world'),
                'launch',
                'assessment_complete.launch.py'
            ])
        ])
    )

    # 2) Prepare your robot description (xacro -> robot_description)
    pkg_my_robot = get_package_share_directory('my_robot_description')  # or 'My_Robot' if that's the package name
    robot_xacro = os.path.join(pkg_my_robot, 'urdf', 'My_Robot.xacro')
    ros_gz_bridge_config = os.path.join(pkg_my_robot, 'config', 'ros_gz_bridge_gazebo.yaml')
    slam_params = os.path.join(pkg_my_robot, 'config', 'mapper_params_online_async.yaml')

    robot_description_config = xacro.process_file(robot_xacro)
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # 3) Spawn your robot into the assessment world (after a small delay)
    spawn_my_robot = TimerAction(
        period=5.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', '/robot_description',
                '-name', 'My_Robot',
                '-allow_renaming', 'false',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.32',
                '-Y', '0.0',
            ],
            output='screen'
        )]
    )

    # 4) ros_gz_bridge for your sensors / cmd_vel / odom / tf
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': ros_gz_bridge_config}],
        output='screen'
    )

    # 5) SLAM (optional but you already set this up)
    # slam_launch = IncludeLaunchDescription(
    # PythonLaunchDescriptionSource([
    #     os.path.join(
    #         get_package_share_directory('slam_toolbox'),
    #         'launch',
    #         'online_async_launch.py'
    #     )
    # ]),
    # launch_arguments={
    #     'slam_params_file': slam_params,
    #     'use_sim_time': 'true',
    # }.items(),
    # )

    # # 6) Teleoperation node (optional to include here; otherwise run manually)
    # teleop_node = Node(
    #     package='teleop_twist_keyboard',
    #     executable='teleop_twist_keyboard',
    #     name='teleop',
    #     output='screen',
    #     emulate_tty=True,
    #     prefix=['xterm -e ']  # opens in its own terminal; remove if you prefer same terminal
    # )

    return LaunchDescription([
        assessment_complete,   # world + spheres
        robot_state_publisher, # robot_description + TF
        spawn_my_robot,        # spawn robot in the world
        ros_gz_bridge,         # sensor/velocity bridge
        # slam_launch,             # mapping
        # teleop_node,           # keyboard control
    ])
