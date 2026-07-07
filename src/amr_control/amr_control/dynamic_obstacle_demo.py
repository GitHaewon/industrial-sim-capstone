"""Move a Gazebo obstacle near the active Nav2 path.

The obstacle is intentionally simple: a kinematic pallet model is moved through
Gazebo's SetEntityPose service while Nav2 is active. The active carrier's
LiDAR sees the model and the obstacle layer marks it in the local/global
costmaps, making collision detection and replanning visible in RViz2.
"""

from math import cos, pi
import time

from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import String


class DynamicObstacleDemo(Node):
    """Animate a warehouse pallet for a bounded avoidance demonstration."""

    def __init__(self):
        super().__init__('dynamic_obstacle_demo')
        self.declare_parameter('enabled', True)
        self.declare_parameter('model_name', 'moving_pallet_obstacle')
        self.declare_parameter('crossing_y', -5.95)
        self.declare_parameter('left_x', -1.8)
        self.declare_parameter('right_x', 1.8)
        self.declare_parameter('park_x', 8.2)
        self.declare_parameter('park_y', -5.95)
        self.declare_parameter('period', 8.0)
        self.declare_parameter('active_duration', 10.0)
        self.declare_parameter('max_activations', 1)
        self.declare_parameter('activation_state_prefix', 'NAVIGATING')

        self.enabled = bool(self.get_parameter('enabled').value)
        self.model_name = str(self.get_parameter('model_name').value)
        self.crossing_y = float(self.get_parameter('crossing_y').value)
        self.left_x = float(self.get_parameter('left_x').value)
        self.right_x = float(self.get_parameter('right_x').value)
        self.park_x = float(self.get_parameter('park_x').value)
        self.park_y = float(self.get_parameter('park_y').value)
        self.period = float(self.get_parameter('period').value)
        self.active_duration = float(
            self.get_parameter('active_duration').value
        )
        self.max_activations = int(
            self.get_parameter('max_activations').value
        )
        self.activation_state_prefix = str(
            self.get_parameter('activation_state_prefix').value
        )

        self.active = False
        self.parked = False
        self.activation_count = 0
        self.started_at = time.monotonic()
        self.last_service_warning_at = 0.0
        self.client = self.create_client(
            SetEntityPose, '/world/factory_test/set_pose'
        )
        self.event_publisher = self.create_publisher(
            String, '/factory/event', 10
        )
        self.create_subscription(
            String, '/amr/state', self._state_callback, 10
        )
        self.create_timer(0.10, self._update)
        self.get_logger().info(
            'Dynamic obstacle demo ready: '
            f'{self.model_name} crosses x={self.left_x:.1f}..'
            f'{self.right_x:.1f} at y={self.crossing_y:.1f}'
        )

    def _state_callback(self, message):
        if not self.enabled:
            return
        should_move = message.data.startswith(self.activation_state_prefix)
        if should_move and not self.active:
            if self.activation_count >= self.max_activations:
                return
            self.activation_count += 1
            self.started_at = time.monotonic()
            self.parked = False
            self.get_logger().info(
                'Dynamic obstacle activated for Nav2 avoidance demo'
            )
            self.event_publisher.publish(String(
                data='warning:동적 장애물 회피 데모 활성화'
            ))
        elif not should_move and self.active:
            self._park_obstacle()
        self.active = should_move

    def _update(self):
        if not self.enabled:
            return
        if not self.client.service_is_ready():
            now = time.monotonic()
            if now - self.last_service_warning_at > 3.0:
                self.last_service_warning_at = now
                self.get_logger().warning(
                    'Waiting for /world/factory_test/set_pose bridge'
                )
            return

        if not self.active:
            if not self.parked:
                self._park_obstacle()
            return

        elapsed = time.monotonic() - self.started_at
        if elapsed > self.active_duration:
            self.active = False
            self._park_obstacle()
            self.get_logger().info(
                'Dynamic obstacle moved to safe parking position'
            )
            self.event_publisher.publish(String(
                data='info:동적 장애물 안전 위치 복귀'
            ))
            return

        phase = (elapsed % self.period) / self.period
        midpoint = (self.left_x + self.right_x) / 2.0
        half_span = (self.right_x - self.left_x) / 2.0
        x = midpoint - half_span * cos(2.0 * pi * phase)
        yaw = 0.0 if phase < 0.5 else pi
        self._send_pose(x, self.crossing_y, yaw)

    def _park_obstacle(self):
        self._send_pose(self.park_x, self.park_y, yaw=0.0)
        self.parked = True

    def _send_pose(self, x, y, yaw):
        request = SetEntityPose.Request()
        request.entity = Entity(
            name=self.model_name,
            type=Entity.MODEL,
        )
        request.pose = Pose()
        request.pose.position.x = x
        request.pose.position.y = y
        request.pose.position.z = 0.0
        request.pose.orientation.z = 0.0 if yaw == 0.0 else 1.0
        request.pose.orientation.w = 1.0 if yaw == 0.0 else 0.0
        self.client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleDemo()
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
