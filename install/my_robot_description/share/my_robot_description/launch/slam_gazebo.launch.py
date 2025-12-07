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
    # 1) Gazebo world (assessment) – starts immediately
    assessment_complete = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('assessment_world'),
                'launch',
                'assessment_world.launch.py'
            ])
        ])
    )

    # 2) RViz for SLAM – we'll wrap this in a TimerAction
    rviz_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_robot_description'),
                'launch',
                'rviz_slam.launch.py'
            ])
        ])
    )

    # Delay RViz by 3 seconds after launch start
    rviz_slam_delayed = TimerAction(
        period=6.0,
        actions=[rviz_slam]
    )

    # 3) Xacro → robot_description → robot_state_publisher
    pkg_my_robot = get_package_share_directory('my_robot_description')
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

    # 4) Spawn robot into gz sim – delayed 5 seconds
    spawn_my_robot = TimerAction(
        period=5.0,   # 5 seconds after launch start
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

    # 5) ros_gz_bridge (no delay needed, but you can add one if you want)
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': ros_gz_bridge_config}],
        output='screen'
    )

    # 6) SLAM Toolbox (starts once scan/tf are there; ok to start immediately)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ]),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'true',
        }.items(),
    )

        # Teleop Node (to map and test only)
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        output='screen',
        prefix=['xterm -e ']  # opens in its own terminal; remove if you prefer same terminal
    )

    return LaunchDescription([
        assessment_complete,   # Gazebo world immediately
        robot_state_publisher, # /robot_description + TF
        ros_gz_bridge,         # ROS <-> gz topics
        slam_launch,           # SLAM toolbox
        rviz_slam_delayed,     # RViz after 3s
        spawn_my_robot,        # Spawn robot after 5s
        teleop_node,           # Opens Terminal for Teleop
    ])

#------------
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, TimerAction
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import PathJoinSubstitution
# from launch_ros.substitutions import FindPackageShare
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# import os
# import xacro


# def generate_launch_description():
#     # 1) Include the assessment world + spheres
#     assessment_complete = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([
#             PathJoinSubstitution([
#                 FindPackageShare('assessment_world'),
#                 'launch',
#                 'assessment_world.launch.py'
#             ])
#         ])
#     )

#     rviz_slam = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([
#             PathJoinSubstitution([
#                 FindPackageShare('my_robot_description'),
#                 'launch',
#                 'rviz_slam.launch.py'
#             ])
#         ])
#     )

#     # 2) Prepare your robot description (xacro -> robot_description)
#     pkg_my_robot = get_package_share_directory('my_robot_description')
#     robot_xacro = os.path.join(pkg_my_robot, 'urdf', 'My_Robot.xacro')
#     ros_gz_bridge_config = os.path.join(pkg_my_robot, 'config', 'ros_gz_bridge_gazebo.yaml')
#     slam_params = os.path.join(pkg_my_robot, 'config', 'mapper_params_online_async.yaml')

#     robot_description_config = xacro.process_file(robot_xacro)
#     robot_description = {'robot_description': robot_description_config.toxml()}

#     robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         name='robot_state_publisher',
#         output='screen',
#         parameters=[robot_description, {'use_sim_time': True}],
#     )

#     # 3) Spawn your robot into the assessment world (after a small delay)
#     spawn_my_robot = TimerAction(
#         period=5.0,
#         actions=[Node(
#             package='ros_gz_sim',
#             executable='create',
#             arguments=[
#                 '-topic', '/robot_description',
#                 '-name', 'My_Robot',
#                 '-allow_renaming', 'false',
#                 '-x', '0.0',
#                 '-y', '0.0',
#                 '-z', '0.32',
#                 '-Y', '0.0',
#             ],
#             output='screen'
#         )]
#     )

#     # 4) ros_gz_bridge for your sensors / cmd_vel / odom / tf
#     ros_gz_bridge = Node(
#         package='ros_gz_bridge',
#         executable='parameter_bridge',
#         parameters=[{'config_file': ros_gz_bridge_config}],
#         output='screen'
#     )

#     # 5) SLAM
#     slam_launch = IncludeLaunchDescription(
#     PythonLaunchDescriptionSource([
#         os.path.join(
#             get_package_share_directory('slam_toolbox'),
#             'launch',
#             'online_async_launch.py'
#         )
#     ]),
#     launch_arguments={
#         'slam_params_file': slam_params,
#         'use_sim_time': 'true',
#     }.items(),
#     )

#     # 6) Teleoperation node
#     teleop_node = Node(
#         package='teleop_twist_keyboard',
#         executable='teleop_twist_keyboard',
#         name='teleop',
#         output='screen',
#         prefix=['xterm -e ']  # opens in its own terminal; remove if you prefer same terminal
#     )

#     return LaunchDescription([
#         assessment_complete,   # world + spheres
#         robot_state_publisher, # robot_description + TF
#         spawn_my_robot,        # spawn robot in the world
#         ros_gz_bridge,         # sensor/velocity bridge
#         slam_launch,           # mapping
#         teleop_node,           # keyboard control
#         rviz_slam,             # rviz
#     ])
