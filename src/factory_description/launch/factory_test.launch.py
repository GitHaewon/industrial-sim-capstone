import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    world_path = os.path.join(
        get_package_share_directory('factory_description'),
        'worlds',
        'factory_test.world',
    )

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path],
        output='screen',
    )

    return LaunchDescription([gazebo])
