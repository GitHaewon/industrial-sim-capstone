"""Nav2 action coordination and command routing for mobile sorting boxes."""

from dataclasses import dataclass
from math import atan2, cos, hypot, pi, sin
import time

from action_msgs.msg import GoalStatus
from amr_control.navigation_policy import retry_allowed
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return (angle + pi) % (2.0 * pi) - pi


@dataclass
class BoxCarrier:
    """Configuration and latest pose for a mobile sorting box."""

    model: str
    item_class: str
    initial_x: float
    initial_y: float
    initial_yaw: float
    goals: tuple
    odometry: Odometry | None = None
    goal_index: int = 0
    arrived: bool = False

    def world_pose(self):
        """Return Gazebo ground-truth odometry in the map frame."""
        pose = self.odometry.pose.pose
        yaw = atan2(
            2.0 * (
                pose.orientation.w * pose.orientation.z
                + pose.orientation.x * pose.orientation.y
            ),
            1.0 - 2.0 * (
                pose.orientation.y * pose.orientation.y
                + pose.orientation.z * pose.orientation.z
            ),
        )
        return pose.position.x, pose.position.y, normalize_angle(yaw)


class AmrController(Node):
    """Send each loaded box to Nav2 and retry failed navigation goals."""

    CARRIERS = (
        (
            'bin_a_red', 'A', 0.5, -2.95, pi,
            ((-5.2, -7.55, -pi / 2.0),),
        ),
        (
            'bin_b_green', 'B', 2.75, -1.5, 0.0,
            (
                (4.5, -1.5, 0.0),
                (4.5, -5.8, -pi / 2.0),
                (0.0, -7.55, -pi / 2.0),
            ),
        ),
        (
            'bin_c_blue', 'C', 2.0, -2.8, -pi / 2.0,
            (
                (2.0, -4.8, -pi / 2.0),
                (5.2, -6.4, -0.5),
                (5.2, -7.55, -pi / 2.0),
            ),
        ),
    )

    def __init__(self):
        super().__init__('amr_controller')
        self.declare_parameter('max_retries', 2)
        self.declare_parameter('navigation_timeout', 120.0)
        self.declare_parameter('stuck_timeout', 18.0)
        self.declare_parameter('retry_delay', 2.0)
        self.declare_parameter('arrival_tolerance', 0.55)
        self.max_retries = int(self.get_parameter('max_retries').value)
        self.navigation_timeout = float(
            self.get_parameter('navigation_timeout').value
        )
        self.stuck_timeout = float(self.get_parameter('stuck_timeout').value)
        self.retry_delay = float(self.get_parameter('retry_delay').value)
        self.arrival_tolerance = float(
            self.get_parameter('arrival_tolerance').value
        )

        self.carriers = [
            BoxCarrier(*configuration) for configuration in self.CARRIERS
        ]
        self.active_index = 0
        self.mode = 'WAITING'
        self.failed_attempts = 0
        self.goal_serial = 0
        self.goal_handle = None
        self.goal_sent_at = 0.0
        self.next_goal_at = 0.0
        self.best_distance = float('inf')
        self.last_progress_at = 0.0
        self.progress_pose = None

        self.command_publishers = {}
        for carrier in self.carriers:
            self.command_publishers[carrier.model] = self.create_publisher(
                Twist, f'/model/{carrier.model}/cmd_vel', 10
            )
            self.create_subscription(
                Odometry,
                f'/model/{carrier.model}/odometry',
                lambda message, name=carrier.model:
                self._odometry_callback(message, name),
                10,
            )

        self.nav_odometry_publisher = self.create_publisher(
            Odometry, '/nav/odom', 20
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Twist, '/nav/cmd_vel', self._nav_command_callback, 20
        )
        self.navigator = ActionClient(
            self, NavigateToPose, '/navigate_to_pose'
        )
        self.complete_publisher = self.create_publisher(
            Bool, '/amr/delivery_complete', 10
        )
        self.failure_publisher = self.create_publisher(
            Bool, '/amr/delivery_failed', 10
        )
        self.state_publisher = self.create_publisher(
            String, '/amr/state', 10
        )
        self.create_subscription(
            Bool, '/amr/start_delivery', self._start_callback, 10
        )
        self.create_timer(0.05, self._update)
        self.get_logger().info(
            'Nav2 box coordinator ready (retry limit: '
            f'{self.max_retries})'
        )

    @property
    def active_carrier(self):
        return self.carriers[self.active_index]

    def _start_callback(self, message):
        if not message.data or self.mode != 'WAITING':
            return
        if any(carrier.odometry is None for carrier in self.carriers):
            self._terminal_failure('ODOMETRY_NOT_READY')
            return

        self.active_index = 0
        for carrier in self.carriers:
            carrier.goal_index = 0
            carrier.arrived = False
        self.mode = 'SWITCHING'
        self.next_goal_at = time.monotonic() + 0.8
        self.failed_attempts = 0
        self.state_publisher.publish(String(data='NAV2_STARTING:A'))
        self.get_logger().info(
            'Nav2 delivery started: A/B/C -> left/center/right doors'
        )

    def _odometry_callback(self, message, model):
        carrier = next(
            item for item in self.carriers if item.model == model
        )
        carrier.odometry = message
        if carrier is self.active_carrier:
            self._publish_nav_pose(carrier)

    def _publish_nav_pose(self, carrier):
        x, y, yaw = carrier.world_pose()
        stamp = self.get_clock().now().to_msg()
        orientation_z = sin(yaw / 2.0)
        orientation_w = cos(yaw / 2.0)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = 'map'
        odometry.child_frame_id = 'base_link'
        odometry.pose.pose.position.x = x
        odometry.pose.pose.position.y = y
        odometry.pose.pose.orientation.z = orientation_z
        odometry.pose.pose.orientation.w = orientation_w
        odometry.twist = carrier.odometry.twist
        self.nav_odometry_publisher.publish(odometry)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = orientation_z
        transform.transform.rotation.w = orientation_w
        self.tf_broadcaster.sendTransform(transform)

    def _nav_command_callback(self, message):
        if self.mode != 'NAVIGATING':
            return
        self.get_logger().info(
            'Nav2 command '
            f'v={message.linear.x:.2f} w={message.angular.z:.2f}',
            throttle_duration_sec=5.0,
        )
        self.command_publishers[
            self.active_carrier.model
        ].publish(message)

    def _update(self):
        now = time.monotonic()
        if self.mode in ('SWITCHING', 'RETRY_WAIT') and now >= self.next_goal_at:
            self._send_goal()
        elif (
            self.mode == 'GOAL_PENDING'
            and now - self.goal_sent_at > 10.0
        ):
            self._attempt_failed('GOAL_RESPONSE_TIMEOUT')
        elif self.mode == 'NAVIGATING':
            self._monitor_navigation(now)

    def _send_goal(self):
        if not self.navigator.server_is_ready():
            self._attempt_failed('NAV2_SERVER_UNAVAILABLE')
            return

        carrier = self.active_carrier
        goal_x, goal_y, goal_yaw = carrier.goals[carrier.goal_index]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        goal.pose.pose.orientation.z = sin(goal_yaw / 2.0)
        goal.pose.pose.orientation.w = cos(goal_yaw / 2.0)

        self.goal_serial += 1
        serial = self.goal_serial
        self.mode = 'GOAL_PENDING'
        self.goal_sent_at = time.monotonic()
        future = self.navigator.send_goal_async(
            goal, feedback_callback=self._feedback_callback
        )
        future.add_done_callback(
            lambda result, token=serial:
            self._goal_response_callback(result, token)
        )
        self.state_publisher.publish(String(
            data=f'NAV2_PLANNING:{carrier.item_class}'
        ))

    def _goal_response_callback(self, future, serial):
        if serial != self.goal_serial or self.mode != 'GOAL_PENDING':
            return
        try:
            self.goal_handle = future.result()
        except Exception as error:  # noqa: BLE001
            self._attempt_failed(f'GOAL_EXCEPTION:{error}')
            return
        if not self.goal_handle.accepted:
            self._attempt_failed('GOAL_REJECTED')
            return

        now = time.monotonic()
        self.mode = 'NAVIGATING'
        self.goal_sent_at = now
        self.last_progress_at = now
        self.best_distance = self._goal_distance()
        self.progress_pose = self.active_carrier.world_pose()
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, token=serial:
            self._result_callback(result, token)
        )
        self.state_publisher.publish(String(
            data=f'NAVIGATING:{self.active_carrier.item_class}'
        ))

    def _feedback_callback(self, _message):
        if self.mode != 'NAVIGATING':
            return
        distance = self._goal_distance()
        if distance + 0.10 < self.best_distance:
            self.best_distance = distance
            self.last_progress_at = time.monotonic()

    def _monitor_navigation(self, now):
        current_pose = self.active_carrier.world_pose()
        if self.progress_pose is None:
            self.progress_pose = current_pose
        moved = hypot(
            current_pose[0] - self.progress_pose[0],
            current_pose[1] - self.progress_pose[1],
        )
        turned = abs(normalize_angle(
            current_pose[2] - self.progress_pose[2]
        ))
        # A differential-drive box may need a long in-place turn before its
        # distance to the goal decreases. Physical rotation is still progress.
        if moved > 0.08 or turned > 0.10:
            self.progress_pose = current_pose
            self.last_progress_at = now

        if now - self.goal_sent_at > self.navigation_timeout:
            self._attempt_failed('TIMEOUT')
        elif now - self.last_progress_at > self.stuck_timeout:
            self._attempt_failed('NO_PROGRESS')

    def _result_callback(self, future, serial):
        if serial != self.goal_serial or self.mode != 'NAVIGATING':
            return
        wrapped_result = future.result()
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self._attempt_failed(
                f'NAV2_STATUS_{wrapped_result.status}'
            )
            return
        distance = self._goal_distance()
        if distance > self.arrival_tolerance:
            self._attempt_failed(f'GOAL_ERROR_{distance:.2f}M')
            return
        self._carrier_arrived(distance)

    def _goal_distance(self):
        x, y, _ = self.active_carrier.world_pose()
        goal_x, goal_y, _ = self.active_carrier.goals[
            self.active_carrier.goal_index
        ]
        return hypot(
            goal_x - x,
            goal_y - y,
        )

    def _attempt_failed(self, reason):
        if self.mode in ('FAILED', 'DELIVERED'):
            return
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.goal_serial += 1
        self.failed_attempts += 1
        self._stop_active()

        if retry_allowed(self.failed_attempts, self.max_retries):
            self.mode = 'RETRY_WAIT'
            self.next_goal_at = time.monotonic() + self.retry_delay
            self.state_publisher.publish(String(
                data=(
                    f'RETRY:{self.active_carrier.item_class}:'
                    f'{self.failed_attempts}:{reason}'
                )
            ))
            self.get_logger().warning(
                f'Box {self.active_carrier.item_class} navigation failed '
                f'({reason}); retry {self.failed_attempts}/'
                f'{self.max_retries}'
            )
        else:
            self._terminal_failure(reason)

    def _terminal_failure(self, reason):
        self.mode = 'FAILED'
        self._stop_all()
        item_class = self.active_carrier.item_class
        self.state_publisher.publish(
            String(data=f'FAILED:{item_class}:{reason}')
        )
        self.failure_publisher.publish(Bool(data=True))
        self.get_logger().error(
            f'Box {item_class} delivery failed permanently: {reason}'
        )

    def _carrier_arrived(self, distance):
        carrier = self.active_carrier
        self._stop_active()
        if carrier.goal_index < len(carrier.goals) - 1:
            carrier.goal_index += 1
            self.failed_attempts = 0
            self.goal_handle = None
            self.mode = 'SWITCHING'
            self.next_goal_at = time.monotonic() + 0.5
            self.state_publisher.publish(String(
                data=(
                    f'WAYPOINT:{carrier.item_class}:'
                    f'{carrier.goal_index}/{len(carrier.goals) - 1}'
                )
            ))
            self.get_logger().info(
                f'Box {carrier.item_class} reached Nav2 waypoint '
                f'{carrier.goal_index}/{len(carrier.goals) - 1}'
            )
            return

        carrier.arrived = True
        self.state_publisher.publish(
            String(data=f'ARRIVED:{carrier.item_class}')
        )
        self.get_logger().info(
            f'Box {carrier.item_class} arrived via Nav2 '
            f'(error {distance:.2f} m)'
        )
        if self.active_index == len(self.carriers) - 1:
            self.mode = 'DELIVERED'
            self.complete_publisher.publish(Bool(data=True))
            self.state_publisher.publish(String(data='DELIVERED'))
            self.get_logger().info(
                'All loaded boxes remain at their loading doors'
            )
            return

        self.active_index += 1
        self.failed_attempts = 0
        self.goal_handle = None
        self.mode = 'SWITCHING'
        self.next_goal_at = time.monotonic() + 0.8

    def _stop_active(self):
        self.command_publishers[
            self.active_carrier.model
        ].publish(Twist())

    def _stop_all(self):
        for publisher in self.command_publishers.values():
            publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = AmrController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._stop_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
