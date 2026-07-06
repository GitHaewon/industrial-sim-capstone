"""Seeded three-item batch placement for the factory conveyor."""

from dataclasses import asdict, dataclass
import json
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import Bool, Empty, String


MODEL_BY_CLASS = {
    'A': 'item_a_red_cube',
    'B': 'item_b_green_cylinder',
    'C': 'item_c_blue_hex',
}


@dataclass(frozen=True)
class ItemPlacement:
    """One reproducible item placement in conveyor arrival order."""

    work_id: str
    item_class: str
    model: str
    x: float
    y: float
    z: float


def generate_layout(
    seed,
    run_id='test-run',
    leading_x=-0.9,
    minimum_gap=0.9,
    maximum_gap=1.2,
):
    """Return one of each class in seeded arrival order and spacing."""
    if minimum_gap <= 0.0 or maximum_gap < minimum_gap:
        raise ValueError('invalid item gap range')

    generator = random.Random(seed)
    classes = list(MODEL_BY_CLASS)
    generator.shuffle(classes)

    x = leading_x
    placements = []
    for index, item_class in enumerate(classes, start=1):
        placements.append(ItemPlacement(
            work_id=f'{run_id}-{index:02d}-{item_class}',
            item_class=item_class,
            model=MODEL_BY_CLASS[item_class],
            x=round(x, 4),
            y=0.0,
            z=0.98,
        ))
        if index < len(classes):
            x -= generator.uniform(minimum_gap, maximum_gap)
    return placements


class RandomSpawner(Node):
    """Move the fixed detachable items into a randomized starting batch."""

    def __init__(self):
        super().__init__('random_item_spawner')
        self.declare_parameter('random_seed', 42)
        self.declare_parameter('placement_delay', 3.2)
        self.declare_parameter('leading_x', -0.9)
        self.declare_parameter('minimum_gap', 0.9)
        self.declare_parameter('maximum_gap', 1.2)

        self.seed = self.get_parameter(
            'random_seed'
        ).get_parameter_value().integer_value
        self.leading_x = float(self.get_parameter('leading_x').value)
        self.minimum_gap = float(self.get_parameter('minimum_gap').value)
        self.maximum_gap = float(self.get_parameter('maximum_gap').value)
        self.cycle_number = 0
        self.placements = []

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.manifest_publisher = self.create_publisher(
            String, '/item_spawner/manifest', latched_qos
        )
        self.ready_publisher = self.create_publisher(
            Bool, '/item_spawner/ready', latched_qos
        )
        self.bin_detach_publishers = {
            item_class: self.create_publisher(
                Empty,
                f'/bin/{model.removeprefix("item_")}/detach',
                10,
            )
            for item_class, model in MODEL_BY_CLASS.items()
        }
        self.pose_client = self.create_client(
            SetEntityPose, '/world/factory_test/set_pose'
        )
        self.create_subscription(
            Bool, '/amr/return_complete', self._return_callback, 10
        )
        self.pending = 0
        self.started = False
        self.delay = self.get_parameter(
            'placement_delay'
        ).get_parameter_value().double_value
        self.started_at = time.monotonic()
        self.create_timer(0.1, self._try_place_batch)
        self._prepare_cycle()

    def _prepare_cycle(self):
        self.cycle_number += 1
        cycle_seed = self.seed + self.cycle_number - 1
        run_id = (
            f'cycle-{self.cycle_number}-{int(time.time())}'
            f'-seed-{cycle_seed}'
        )
        self.placements = generate_layout(
            seed=cycle_seed,
            run_id=run_id,
            leading_x=self.leading_x,
            minimum_gap=self.minimum_gap,
            maximum_gap=self.maximum_gap,
        )
        self.started = False
        self.pending = 0
        self.started_at = time.monotonic()
        order = ' -> '.join(item.item_class for item in self.placements)
        self.get_logger().info(
            f'Prepared cycle={self.cycle_number} seed={cycle_seed} '
            f'arrival order: {order}'
        )

    def _return_callback(self, message):
        if message.data and self.started and self.pending == 0:
            self.ready_publisher.publish(Bool(data=False))
            for publisher in self.bin_detach_publishers.values():
                publisher.publish(Empty())
            self._prepare_cycle()

    def _try_place_batch(self):
        if self.started:
            return
        for publisher in self.bin_detach_publishers.values():
            publisher.publish(Empty())
        if time.monotonic() - self.started_at < self.delay:
            return
        if not self.pose_client.service_is_ready():
            self.get_logger().info(
                'Waiting for Gazebo set_pose service',
                throttle_duration_sec=2.0,
            )
            return

        self.started = True
        self.pending = len(self.placements)
        for placement in self.placements:
            request = SetEntityPose.Request()
            request.entity = Entity(
                name=placement.model,
                type=Entity.MODEL,
            )
            request.pose.position.x = placement.x
            request.pose.position.y = placement.y
            request.pose.position.z = placement.z
            request.pose.orientation.w = 1.0
            future = self.pose_client.call_async(request)
            future.add_done_callback(
                lambda result, item=placement:
                self._pose_result(result, item)
            )

    def _pose_result(self, future, placement):
        try:
            success = future.result().success
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(
                f'Could not place {placement.model}: {error}'
            )
            success = False

        if not success:
            self.get_logger().error(
                f'Gazebo rejected pose for {placement.model}'
            )
            return

        self.get_logger().info(
            f'Placed {placement.work_id} at x={placement.x:.2f}'
        )
        self.pending -= 1
        if self.pending == 0:
            self._publish_ready()

    def _publish_ready(self):
        manifest = {
            'run_id': self.placements[0].work_id.rsplit('-', 2)[0],
            'seed': self.seed + self.cycle_number - 1,
            'cycle': self.cycle_number,
            'arrival_order': [
                placement.item_class for placement in self.placements
            ],
            'items': [asdict(item) for item in self.placements],
        }
        self.manifest_publisher.publish(
            String(data=json.dumps(manifest, ensure_ascii=False))
        )
        self.ready_publisher.publish(Bool(data=True))
        self.get_logger().info(
            'Random batch is ready; conveyor may start'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RandomSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
