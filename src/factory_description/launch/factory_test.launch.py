from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    random_seed = LaunchConfiguration('random_seed')
    amr_share = Path(get_package_share_directory('amr_control'))
    nav2_params = amr_share / 'config' / 'nav2_params.yaml'
    map_yaml = amr_share / 'config' / 'factory_map.yaml'
    world_path = Path(
        get_package_share_directory('factory_description')
    ) / 'worlds' / 'factory_test.sdf'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim'),
            '/launch/gz_sim.launch.py',
        ]),
        launch_arguments={
            'gz_args': f'-r {world_path}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    conveyor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/conveyor/track_cmd_vel'
            '@std_msgs/msg/Float64]gz.msgs.Double',
            '/pickup/logical_camera'
            '@ros_gz_interfaces/msg/LogicalCameraImage'
            '[gz.msgs.LogicalCameraImage',
            '/factory/camera/image'
            '@sensor_msgs/msg/Image[gz.msgs.Image',
            '/factory/camera/depth_image'
            '@sensor_msgs/msg/Image[gz.msgs.Image',
            '/factory/camera/camera_info'
            '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/arm/yaw_cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/arm/lift_cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/world/factory_test/model/sorting_arm/joint_state'
            '@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/arm/a_red_cube/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/arm/a_red_cube/detach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/arm/b_green_cylinder/attach'
            '@std_msgs/msg/Empty]gz.msgs.Empty',
            '/arm/b_green_cylinder/detach'
            '@std_msgs/msg/Empty]gz.msgs.Empty',
            '/arm/c_blue_hex/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/arm/c_blue_hex/detach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/bin/a_red_cube/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/bin/a_red_cube/detach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/bin/b_green_cylinder/attach'
            '@std_msgs/msg/Empty]gz.msgs.Empty',
            '/bin/b_green_cylinder/detach'
            '@std_msgs/msg/Empty]gz.msgs.Empty',
            '/bin/c_blue_hex/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/bin/c_blue_hex/detach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/world/factory_test/set_pose'
            '@ros_gz_interfaces/srv/SetEntityPose',
            '/model/bin_a_red/cmd_vel'
            '@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/bin_a_red/ground_truth_odometry'
            '@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/bin_a_red/odometry'
            '@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/bin_a_red/scan'
            '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/model/bin_b_green/cmd_vel'
            '@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/bin_b_green/ground_truth_odometry'
            '@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/bin_b_green/odometry'
            '@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/bin_b_green/scan'
            '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/model/bin_c_blue/cmd_vel'
            '@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/bin_c_blue/ground_truth_odometry'
            '@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/bin_c_blue/odometry'
            '@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/bin_c_blue/scan'
            '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        remappings=[
            (
                '/model/bin_a_red/ground_truth_odometry',
                '/model/bin_a_red/ground_truth',
            ),
            (
                '/model/bin_a_red/odometry',
                '/model/bin_a_red/wheel_odometry',
            ),
            (
                '/model/bin_b_green/ground_truth_odometry',
                '/model/bin_b_green/ground_truth',
            ),
            (
                '/model/bin_b_green/odometry',
                '/model/bin_b_green/wheel_odometry',
            ),
            (
                '/model/bin_c_blue/ground_truth_odometry',
                '/model/bin_c_blue/ground_truth',
            ),
            (
                '/model/bin_c_blue/odometry',
                '/model/bin_c_blue/wheel_odometry',
            ),
        ],
        output='screen',
    )

    conveyor_controller = Node(
        package='conveyor_control',
        executable='conveyor_node',
        parameters=[{
            'speed': 0.35,
            'auto_start': False,
        }],
        output='screen',
    )

    arm_controller = Node(
        package='arm_control',
        executable='arm_controller',
        output='screen',
    )

    random_spawner = Node(
        package='conveyor_control',
        executable='random_spawner',
        parameters=[{
            'random_seed': ParameterValue(
                random_seed, value_type=int
            ),
        }],
        output='screen',
    )

    vision_node = Node(
        package='item_vision',
        executable='vision_node',
        output='screen',
    )

    amr_controller = Node(
        package='amr_control',
        executable='amr_controller',
        parameters=[{
            'max_retries': 2,
            'navigation_timeout': 180.0,
            'stuck_timeout': 40.0,
            'retry_delay': 2.0,
            'arrival_tolerance': 0.55,
        }],
        output='screen',
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[
            nav2_params,
            {'yaml_filename': str(map_yaml)},
        ],
        output='screen',
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[nav2_params],
        output='screen',
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[nav2_params],
        remappings=[('cmd_vel', '/nav/cmd_vel')],
        output='screen',
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[nav2_params],
        output='screen',
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[nav2_params],
        remappings=[('cmd_vel', '/nav/cmd_vel')],
        output='screen',
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[nav2_params],
        output='screen',
    )

    nav2_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[nav2_params],
        output='screen',
    )

    factory_manager = Node(
        package='factory_manager',
        executable='factory_manager',
        output='screen',
    )

    dashboard = Node(
        package='factory_manager',
        executable='dashboard_node',
        parameters=[{
            'port': 8080,
            'auto_open': True,
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'random_seed',
            default_value='42',
            description='Seed for item order and conveyor spacing',
        ),
        gazebo,
        conveyor_bridge,
        conveyor_controller,
        arm_controller,
        random_spawner,
        vision_node,
        map_server,
        amcl,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        nav2_lifecycle,
        amr_controller,
        factory_manager,
        dashboard,
    ])
