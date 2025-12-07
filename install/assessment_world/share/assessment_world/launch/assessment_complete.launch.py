from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Include assessment_world launch file
    assessment_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('assessment_world'),
                'launch',
                'assessment_world.launch.py'
            ])
        ])
    )
    
    # Include spawn_spheres launch file with delay
    spawn_spheres_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('assessment_world'),
                'launch',
                'spawn_spheres.launch.py'
            ])
        ])
    )
    
    # my_robot=IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('assessment_world'),
    #             'launch',
    #             'spawn_spheres.launch.py'
    #         ])
    #     ])
    # )

    # Delay spawning spheres to ensure world is fully loaded
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_spheres_launch]
    )

    return LaunchDescription([
        assessment_world_launch,
        delayed_spawn,
        # my_robot
    ])
