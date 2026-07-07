"""Feedback-driven pick-and-place coordinator for the Gazebo sorting arm."""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from std_msgs.msg import Float64
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray


class ArmController(Node):
    """Sequence physical arm joints and detachable suction joints."""

    BIN_YAW = {
        'item_a_red_cube': 2.3,
        'item_b_green_cylinder': -1.571,
        'item_c_blue_hex': -2.618,
    }
    CLASS_TO_ITEM = {
        'A': 'item_a_red_cube',
        'B': 'item_b_green_cylinder',
        'C': 'item_c_blue_hex',
    }

    def __init__(self):
        super().__init__('arm_controller')
        self.state = 'INITIALIZING'
        self.state_started = time.monotonic()
        self.current_item = ''
        self.yaw_position = 0.0
        self.lift_position = 0.0
        self.processed_count = 0
        self.pending_pick = False
        self.spawner_ready = False
        self.latest_detections = []
        self.last_detection_time = 0.0
        self.arrival_order = []
        self.waiting_for_manifest_logged = False

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.yaw_publisher = self.create_publisher(
            Float64, '/arm/yaw_cmd', 10
        )
        self.lift_publisher = self.create_publisher(
            Float64, '/arm/lift_cmd', 10
        )
        self.conveyor_start_publisher = self.create_publisher(
            Bool, '/conveyor/start', 10
        )
        self.complete_publisher = self.create_publisher(
            Bool, '/arm/task_complete', 10
        )
        self.state_publisher = self.create_publisher(
            String, '/arm/state', 10
        )

        self.attach_publishers = {}
        self.detach_publishers = {}
        self.bin_attach_publishers = {}
        for item_name in self.BIN_YAW:
            topic_suffix = item_name.removeprefix('item_')
            self.attach_publishers[item_name] = self.create_publisher(
                Empty, f'/arm/{topic_suffix}/attach', 10
            )
            self.detach_publishers[item_name] = self.create_publisher(
                Empty, f'/arm/{topic_suffix}/detach', 10
            )
            self.bin_attach_publishers[item_name] = self.create_publisher(
                Empty, f'/bin/{topic_suffix}/attach', 10
            )

        self.create_subscription(
            String, '/conveyor/state', self._conveyor_state_callback, 10
        )
        self.create_subscription(
            Bool, '/item_spawner/ready', self._spawner_ready_callback,
            latched_qos
        )
        self.create_subscription(
            String, '/item_spawner/manifest', self._manifest_callback,
            latched_qos
        )
        self.create_subscription(
            Detection3DArray,
            '/vision/detections',
            self._vision_callback,
            10,
        )
        self.create_subscription(
            JointState,
            '/world/factory_test/model/sorting_arm/joint_state',
            self._joint_state_callback,
            10,
        )
        self.create_timer(0.05, self._update)
        self.get_logger().info('Sorting arm initialization started')

    def _spawner_ready_callback(self, message):
        if message.data:
            self.spawner_ready = True
            if self.state == 'COMPLETE':
                self.processed_count = 0
                self.pending_pick = False
                self.current_item = ''
                self._transition('INITIALIZING')
            self.get_logger().info('New random item batch is ready')

    def _manifest_callback(self, message):
        try:
            manifest = json.loads(message.data)
        except json.JSONDecodeError:
            self.get_logger().warning('Invalid item manifest JSON')
            return
        self.arrival_order = list(manifest.get('arrival_order', []))
        self.waiting_for_manifest_logged = False
        self.get_logger().info(
            'Received item manifest: '
            + ' -> '.join(self.arrival_order)
        )

    def _conveyor_state_callback(self, message):
        if self.state != 'WAITING' or message.data != 'STOPPED:ITEM_READY':
            return

        self.pending_pick = True

    def _vision_callback(self, message):
        self.latest_detections = list(message.detections)
        self.last_detection_time = time.monotonic()

    def _begin_vision_pick(self):
        if time.monotonic() - self.last_detection_time > 1.0:
            return False

        expected_class = ''
        if 0 <= self.processed_count < len(self.arrival_order):
            expected_class = self.arrival_order[self.processed_count]
        if not expected_class:
            if not self.waiting_for_manifest_logged:
                self.waiting_for_manifest_logged = True
                self.get_logger().warning(
                    'Waiting for item manifest before picking'
                )
            return False

        candidates = []
        for detection in self.latest_detections:
            if not detection.results:
                continue
            hypothesis = detection.results[0].hypothesis
            if (
                hypothesis.class_id not in self.CLASS_TO_ITEM
                or hypothesis.score < 0.45
                or hypothesis.class_id != expected_class
            ):
                continue
            candidates.append(detection)
        if not candidates:
            return
        detection = max(
            candidates, key=lambda item: item.bbox.center.position.x
        )
        hypothesis = detection.results[0].hypothesis
        position = detection.bbox.center.position
        self.current_item = self.CLASS_TO_ITEM[hypothesis.class_id]
        self.pending_pick = False
        self.get_logger().info(
            'Vision selected '
            f'class={hypothesis.class_id} '
            f'expected={expected_class} '
            f'confidence={hypothesis.score:.2f} '
            f'world=({position.x:.2f}, '
            f'{position.y:.2f}, '
            f'{position.z:.2f})'
        )
        self._transition('ATTACHING')
        return True

    def _joint_state_callback(self, message):
        positions = dict(zip(message.name, message.position))
        self.yaw_position = positions.get('yaw_joint', self.yaw_position)
        self.lift_position = positions.get('lift_joint', self.lift_position)

    def _transition(self, state):
        self.state = state
        self.state_started = time.monotonic()
        detail = f'{state}:{self.current_item}' if self.current_item else state
        self.state_publisher.publish(String(data=detail))
        self.get_logger().info(detail)

    def _at_yaw(self, target):
        error = math.atan2(
            math.sin(target - self.yaw_position),
            math.cos(target - self.yaw_position),
        )
        return abs(error) < 0.06

    def _at_lift(self, target):
        return abs(target - self.lift_position) < 0.035

    def _publish_joint_targets(self, yaw, lift):
        self.yaw_publisher.publish(Float64(data=yaw))
        self.lift_publisher.publish(Float64(data=lift))

    def _update(self):
        elapsed = time.monotonic() - self.state_started

        if self.state == 'INITIALIZING':
            self._publish_joint_targets(0.0, 0.0)
            for publisher in self.detach_publishers.values():
                publisher.publish(Empty())
            if elapsed > 3.0 and self.spawner_ready:
                if self.processed_count < 3:
                    self.conveyor_start_publisher.publish(Bool(data=True))
                self._transition('WAITING')

        elif self.state == 'WAITING':
            self._publish_joint_targets(0.0, 0.0)
            if self.pending_pick:
                self._begin_vision_pick()

        elif self.state == 'ATTACHING':
            self._publish_joint_targets(0.0, 0.0)
            self.attach_publishers[self.current_item].publish(Empty())
            if elapsed > 0.7:
                self._transition('LIFTING')

        elif self.state == 'LIFTING':
            self._publish_joint_targets(0.0, 0.55)
            if self._at_lift(0.55):
                self._transition('ROTATING_TO_BIN')

        elif self.state == 'ROTATING_TO_BIN':
            target_yaw = self.BIN_YAW[self.current_item]
            self._publish_joint_targets(target_yaw, 0.55)
            if self._at_yaw(target_yaw):
                self._transition('LOWERING_TO_BIN')

        elif self.state == 'LOWERING_TO_BIN':
            target_yaw = self.BIN_YAW[self.current_item]
            self._publish_joint_targets(target_yaw, 0.0)
            if self._at_lift(0.0):
                self._transition('DETACHING')

        elif self.state == 'DETACHING':
            target_yaw = self.BIN_YAW[self.current_item]
            self._publish_joint_targets(target_yaw, 0.0)
            self.detach_publishers[self.current_item].publish(Empty())
            self.bin_attach_publishers[self.current_item].publish(Empty())
            if elapsed > 0.7:
                self.processed_count += 1
                self.complete_publisher.publish(Bool(data=True))
                self._transition('RETURN_LIFT')

        elif self.state == 'RETURN_LIFT':
            target_yaw = self.BIN_YAW[self.current_item]
            self._publish_joint_targets(target_yaw, 0.55)
            if self._at_lift(0.55):
                self._transition('RETURN_ROTATE')

        elif self.state == 'RETURN_ROTATE':
            self._publish_joint_targets(0.0, 0.55)
            if self._at_yaw(0.0):
                self._transition('RETURN_LOWER')

        elif self.state == 'RETURN_LOWER':
            self._publish_joint_targets(0.0, 0.0)
            if self._at_lift(0.0):
                completed_item = self.current_item
                self.current_item = ''
                if self.processed_count < 3:
                    self.conveyor_start_publisher.publish(Bool(data=True))
                    self._transition('WAITING')
                else:
                    self.spawner_ready = False
                    self._transition('COMPLETE')
                self.get_logger().info(
                    f'Completed {completed_item}; total={self.processed_count}'
                )


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
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
