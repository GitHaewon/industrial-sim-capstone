"""Command the Gazebo conveyor and stop items at the pickup zone."""

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import LogicalCameraImage
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray


class ConveyorNode(Node):
    """Drive the physical belt and monitor workpiece poses."""

    def __init__(self):
        super().__init__('conveyor_node')
        self.declare_parameter('speed', 0.35)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('vision_pickup_x', 0.75)
        self.declare_parameter('vision_max_abs_y', 0.65)

        self.speed = float(self.get_parameter('speed').value)
        self.vision_pickup_x = float(
            self.get_parameter('vision_pickup_x').value
        )
        self.vision_max_abs_y = float(
            self.get_parameter('vision_max_abs_y').value
        )
        self.running = bool(self.get_parameter('auto_start').value)
        self.stopped_item = ''
        self.arrival_order = []
        self.loaded_count = 0
        self.waiting_for_manifest_logged = False

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.velocity_publisher = self.create_publisher(
            Float64, '/conveyor/track_cmd_vel', 10
        )
        self.running_publisher = self.create_publisher(
            Bool, '/conveyor/running', 10
        )
        self.state_publisher = self.create_publisher(
            String, '/conveyor/state', 10
        )

        self.create_subscription(
            Bool, '/conveyor/start', self._start_callback, 10
        )
        self.create_subscription(
            Bool, '/conveyor/stop', self._stop_callback, 10
        )
        self.create_subscription(
            LogicalCameraImage,
            '/pickup/logical_camera',
            self._logical_camera_callback,
            10,
        )
        self.create_subscription(
            Detection3DArray,
            '/vision/detections',
            self._vision_callback,
            10,
        )
        self.create_subscription(
            String,
            '/item_spawner/manifest',
            self._manifest_callback,
            latched_qos,
        )
        self.create_subscription(
            Int32,
            '/box/item_count',
            self._count_callback,
            10,
        )
        self.create_timer(0.1, self._publish_command)

        initial_state = 'RUNNING' if self.running else 'STOPPED'
        self.get_logger().info(
            f'Conveyor {initial_state.lower()} at {self.speed:.2f} m/s'
        )

    def _start_callback(self, message):
        if message.data:
            self.stopped_item = ''
            self._set_running(True, 'START_COMMAND')

    def _stop_callback(self, message):
        if message.data:
            self._set_running(False, 'STOP_COMMAND')

    def _logical_camera_callback(self, message):
        if not self.running:
            return

        expected_class = self._expected_class()
        if expected_class is None:
            self._log_waiting_for_manifest()
            return

        for detected_model in message.model:
            item_name = detected_model.name
            if not item_name.startswith('item_'):
                continue
            item_class = self._class_from_item_name(item_name)
            if item_class != expected_class:
                continue

            self.stopped_item = item_name
            self.get_logger().info(
                f'Pickup sensor detected {item_name}; '
                f'expected={expected_class}'
            )
            self._set_running(False, 'ITEM_READY')
            return

    def _manifest_callback(self, message):
        try:
            manifest = json.loads(message.data)
        except json.JSONDecodeError:
            self.get_logger().warning('Invalid item manifest JSON')
            return
        self.arrival_order = list(manifest.get('arrival_order', []))
        self.loaded_count = 0
        self.waiting_for_manifest_logged = False
        self.get_logger().info(
            'Received item manifest: '
            + ' -> '.join(self.arrival_order)
        )

    def _count_callback(self, message):
        self.loaded_count = max(0, int(message.data))

    def _expected_class(self):
        if not self.arrival_order:
            return None
        if 0 <= self.loaded_count < len(self.arrival_order):
            return self.arrival_order[self.loaded_count]
        return None

    def _class_from_item_name(self, item_name):
        if '_a_' in item_name:
            return 'A'
        if '_b_' in item_name:
            return 'B'
        if '_c_' in item_name:
            return 'C'
        return None

    def _log_waiting_for_manifest(self):
        if self.waiting_for_manifest_logged:
            return
        self.waiting_for_manifest_logged = True
        self.get_logger().warning(
            'Waiting for item manifest before stopping conveyor'
        )

    def _vision_callback(self, message):
        """Fallback stop trigger when Gazebo logical camera is unavailable."""
        if not self.running:
            return

        expected_class = self._expected_class()
        if expected_class is None:
            self._log_waiting_for_manifest()
            return
        for detection in message.detections:
            if not detection.results:
                continue
            item_class = detection.results[0].hypothesis.class_id
            position = detection.bbox.center.position
            if (
                item_class not in {'A', 'B', 'C'}
                or item_class != expected_class
                or position.x < self.vision_pickup_x
                or abs(position.y) > self.vision_max_abs_y
            ):
                continue

            self.stopped_item = f'vision_class_{item_class}'
            self.get_logger().info(
                'Vision pickup fallback detected '
                f'class={item_class} at x={position.x:.2f}, '
                f'y={position.y:.2f}; '
                f'expected={expected_class}'
            )
            self._set_running(False, 'ITEM_READY')
            return

    def _set_running(self, running, reason):
        if self.running == running:
            return

        self.running = running
        state = 'RUNNING' if running else 'STOPPED'
        self.get_logger().info(f'{state}: {reason}')
        self.state_publisher.publish(String(data=f'{state}:{reason}'))

    def _publish_command(self):
        velocity = self.speed if self.running else 0.0
        self.velocity_publisher.publish(Float64(data=velocity))
        self.running_publisher.publish(Bool(data=self.running))


def main(args=None):
    rclpy.init(args=args)
    node = ConveyorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.velocity_publisher.publish(Float64(data=0.0))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
