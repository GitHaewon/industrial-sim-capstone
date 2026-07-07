from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Start the capstone demo with presentation-friendly defaults."""

    random_seed = LaunchConfiguration('random_seed')
    use_rviz = LaunchConfiguration('rviz')
    dynamic_obstacle = LaunchConfiguration('dynamic_obstacle')
    factory_share = Path(get_package_share_directory('factory_description'))
    factory_launch = factory_share / 'launch' / 'factory_test.launch.py'

    return LaunchDescription([
        DeclareLaunchArgument(
            'random_seed',
            default_value='42',
            description='Deterministic seed for the presentation demo',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Open RViz2 with the factory navigation layout',
        ),
        DeclareLaunchArgument(
            'dynamic_obstacle',
            default_value='true',
            description='Show the bounded dynamic obstacle avoidance demo',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(factory_launch)),
            launch_arguments={
                'random_seed': random_seed,
                'rviz': use_rviz,
                'dynamic_obstacle': dynamic_obstacle,
            }.items(),
        ),
    ])
