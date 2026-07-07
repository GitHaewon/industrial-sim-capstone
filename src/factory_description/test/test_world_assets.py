from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).parents[1]


def test_factory_world_is_valid_xml():
    world = PACKAGE_ROOT / 'worlds' / 'factory_test.sdf'

    root = ET.parse(world).getroot()

    assert root.tag == 'sdf'
    assert root.find("./world[@name='factory_test']") is not None


def test_world_does_not_require_classic_gazebo_models():
    world_text = (
        PACKAGE_ROOT / 'worlds' / 'factory_test.sdf'
    ).read_text(encoding='utf-8')

    assert 'model://' not in world_text
    assert 'gz::sim::systems::Physics' in world_text


def test_world_contains_sorting_cell_components():
    world = ET.parse(
        PACKAGE_ROOT / 'worlds' / 'factory_test.sdf'
    ).getroot().find("./world[@name='factory_test']")
    model_names = {model.attrib['name'] for model in world.findall('model')}

    assert {
        'sorting_conveyor',
        'sorting_arm',
        'overhead_rgbd_camera',
        'pickup_sensor',
        'pickup_sensor',
        'bin_a_red',
        'bin_b_green',
        'bin_c_blue',
        'warehouse_zones',
        'warehouse_building',
        'warehouse_racking',
        'warehouse_floor_markings',
        'moving_pallet_obstacle',
    } <= model_names

    rgbd_sensor = world.find(
        "./model[@name='overhead_rgbd_camera']"
        "/link/sensor[@type='rgbd_camera']"
    )
    assert rgbd_sensor is not None

    pickup_sensor = world.find(
        "./model[@name='pickup_sensor']"
        "/link/sensor[@type='logical_camera']"
    )
    assert pickup_sensor is not None
    assert pickup_sensor.findtext('topic') == '/pickup/logical_camera'

    pickup_sensor = world.find(
        "./model[@name='pickup_sensor']"
        "/link/sensor[@type='logical_camera']"
    )
    assert pickup_sensor is not None
    assert pickup_sensor.findtext('topic') == '/pickup/logical_camera'

    conveyor_plugin = world.find(
        "./model[@name='sorting_conveyor']"
        "/plugin[@name='gz::sim::systems::TrackController']"
    )
    assert conveyor_plugin is not None
    assert conveyor_plugin.findtext('velocity_topic') == (
        '/conveyor/track_cmd_vel'
    )

    arm = world.find("./model[@name='sorting_arm']")
    joint_names = {joint.attrib['name'] for joint in arm.findall('joint')}
    assert {'yaw_joint', 'lift_joint', 'world_fixed'} <= joint_names

    detachable_joints = arm.findall(
        "./plugin[@name='gz::sim::systems::DetachableJoint']"
    )
    assert len(detachable_joints) == 3

    for bin_name in ('bin_a_red', 'bin_b_green', 'bin_c_blue'):
        mobile_bin = world.find(f"./model[@name='{bin_name}']")
        assert mobile_bin.findtext('static') != 'true'
        assert mobile_bin.find(
            "./plugin[@name='gz::sim::systems::DiffDrive']"
        ) is not None
        lidar = mobile_bin.find("./link/sensor[@type='gpu_lidar']")
        assert lidar is not None
        assert lidar.findtext('topic') == f'/model/{bin_name}/scan'
        cargo_joint = mobile_bin.find(
            "./plugin[@name='gz::sim::systems::DetachableJoint']"
        )
        assert cargo_joint is not None
        assert cargo_joint.findtext('parent_link') == 'bin'
        assert mobile_bin.find("./joint[@name='left_wheel_joint']") is not None
        assert mobile_bin.find(
            "./joint[@name='right_wheel_joint']"
        ) is not None

    warehouse = world.find("./model[@name='warehouse_building']")
    assert warehouse.find("./link/collision[@name='west_wall_collision']") \
        is not None
    assert warehouse.find("./link/visual[@name='dock_door_b']") is not None

    racks = world.find("./model[@name='warehouse_racking']/link")
    rack_visual_names = {
        visual.attrib['name'] for visual in racks.findall('visual')
    }
    assert len(rack_visual_names) >= 40
    assert {'cargo_n_1', 'cargo_s_5'} <= rack_visual_names

    moving_obstacle = world.find("./model[@name='moving_pallet_obstacle']")
    assert moving_obstacle is not None
    assert moving_obstacle.findtext('static') == 'true'
    assert moving_obstacle.find(
        "./link/collision[@name='body_collision']"
    ) is not None


def test_launch_uses_ros_gz_sim():
    launch_text = (
        PACKAGE_ROOT / 'launch' / 'factory_test.launch.py'
    ).read_text(encoding='utf-8')

    assert "FindPackageShare('ros_gz_sim')" in launch_text
    assert 'factory_test.sdf' in launch_text
    assert "cmd=['gazebo'" not in launch_text
    assert "package='item_vision'" in launch_text
    assert "executable='random_spawner'" in launch_text
    assert "executable='amr_controller'" in launch_text
    assert "executable='factory_manager'" in launch_text
    assert "executable='dashboard_node'" in launch_text
    assert "package='nav2_controller'" in launch_text
    assert "package='nav2_amcl'" in launch_text
    assert "package='nav2_planner'" in launch_text
    assert "executable='bt_navigator'" in launch_text
    assert '/factory/camera/depth_image' in launch_text
    assert '/world/factory_test/set_pose' in launch_text
    assert '/model/bin_a_red/scan' in launch_text
    assert '/model/bin_b_green/scan' in launch_text
    assert '/model/bin_c_blue/scan' in launch_text
    assert '/model/bin_a_red/wheel_odometry' in launch_text
    assert "executable='dynamic_obstacle_demo'" in launch_text
    assert "executable='rviz2'" in launch_text
    assert "factory_nav.rviz" in launch_text
    assert "dynamic_obstacle" in launch_text
    assert "SetEnvironmentVariable('GZ_IP', '127.0.0.1')" in launch_text
    assert "SetEnvironmentVariable('IGN_IP', '127.0.0.1')" in launch_text
    assert "SetEnvironmentVariable(" in launch_text
    assert 'industrial_sim_capstone' in launch_text


def test_sorting_bins_do_not_overlap():
    world = ET.parse(
        PACKAGE_ROOT / 'worlds' / 'factory_test.sdf'
    ).getroot().find("./world[@name='factory_test']")
    bin_names = ('bin_a_red', 'bin_b_green', 'bin_c_blue')
    bin_positions = []

    for name in bin_names:
        pose_values = world.find(
            f"./model[@name='{name}']/pose"
        ).text.split()
        bin_positions.append(tuple(map(float, pose_values[:2])))

    minimum_axis_separation = 1.05
    for index, first in enumerate(bin_positions):
        for second in bin_positions[index + 1:]:
            x_separation = abs(first[0] - second[0])
            y_separation = abs(first[1] - second[1])
            assert max(x_separation, y_separation) >= minimum_axis_separation


def test_delivery_zones_align_with_three_loading_doors():
    world = ET.parse(
        PACKAGE_ROOT / 'worlds' / 'factory_test.sdf'
    ).getroot().find("./world[@name='factory_test']")
    expected_x = (-5.2, 0.0, 5.2)

    for suffix, destination_x in zip(('a', 'b', 'c'), expected_x):
        zone_pose = world.find(
            "./model[@name='warehouse_zones']/link/"
            f"visual[@name='zone_{suffix}']/pose"
        ).text.split()
        door_pose = world.find(
            "./model[@name='warehouse_building']/link/"
            f"visual[@name='dock_door_{suffix}']/pose"
        ).text.split()
        assert float(zone_pose[0]) == destination_x
        assert float(zone_pose[0]) == float(door_pose[0])
        assert float(zone_pose[1]) > float(door_pose[1])
