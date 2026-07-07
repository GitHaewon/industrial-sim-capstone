"""Nav2 action coordination and command routing for mobile sorting boxes."""

from dataclasses import dataclass
from math import atan2, cos, hypot, pi, sin
import time

from action_msgs.msg import GoalStatus
from amr_control.navigation_policy import retry_allowed
from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    TransformStamped,
    Twist,
)
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return (angle + pi) % (2.0 * pi) - pi


def clamp(value, minimum, maximum):
    """Limit a value to the inclusive range [minimum, maximum]."""
    return max(minimum, min(maximum, value))


@dataclass
class BoxCarrier:
    """Configuration and latest pose for a mobile sorting box."""

    model: str
    item_class: str
    initial_x: float
    initial_y: float
    initial_yaw: float
    goals: tuple
    escape_pose: tuple | None = None
    ground_truth: Odometry | None = None
    wheel_odometry: Odometry | None = None
    scan: LaserScan | None = None
    localized_pose: tuple | None = None
    goal_index: int = 0
    arrived: bool = False
    escaped: bool = False

    def world_pose(self):
        """Return the AMCL pose, falling back to truth before localization."""
        if self.localized_pose is not None:
            return self.localized_pose
        return self.truth_pose()

    def truth_pose(self):
        """Return Gazebo ground truth used only to seed and verify AMCL."""
        pose = self.ground_truth.pose.pose
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
            ((-5.2, -6.35, -pi / 2.0),),
        ),
        (
            'bin_b_green', 'B', 2.75, -1.5, 0.0,
            (
                (4.4, -5.8, -pi / 2.0),
                (0.0, -6.35, -pi / 2.0),
            ),
            (4.4, -3.5, -pi / 2.0),
        ),
        (
            'bin_c_blue', 'C', 2.0, -2.8, -pi / 2.0,
            (
                (2.0, -4.8, -pi / 2.0),
                (5.2, -5.8, -0.5),
                (5.2, -6.35, -pi / 2.0),
            ),
        ),
    )

    def __init__(self):
        super().__init__('amr_controller')
        self.declare_parameter('max_retries', 2)
        self.declare_parameter('navigation_timeout', 120.0)
        self.declare_parameter('stuck_timeout', 18.0)
        self.declare_parameter('retry_delay', 2.0)
        self.declare_parameter('arrival_tolerance', 0.8)
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
        self.localization_pending = False
        self.localization_requested_at = 0.0
        self.localization_last_publish_at = 0.0
        self.localization_candidate = None
        self.localization_candidate_since = 0.0
        self.escape_started_at = 0.0

        self.command_publishers = {}
        for carrier in self.carriers:
            self.command_publishers[carrier.model] = self.create_publisher(
                Twist, f'/model/{carrier.model}/cmd_vel', 10
            )
            self.create_subscription(
                Odometry,
                f'/model/{carrier.model}/ground_truth',
                lambda message, name=carrier.model:
                self._ground_truth_callback(message, name),
                10,
            )
            self.create_subscription(
                Odometry,
                f'/model/{carrier.model}/wheel_odometry',
                lambda message, name=carrier.model:
                self._wheel_odometry_callback(message, name),
                20,
            )
            self.create_subscription(
                LaserScan,
                f'/model/{carrier.model}/scan',
                lambda message, name=carrier.model:
                self._scan_callback(message, name),
                qos_profile_sensor_data,
            )

        self.nav_odometry_publisher = self.create_publisher(
            Odometry, '/nav/odom', 20
        )
        self.scan_publisher = self.create_publisher(
            LaserScan, '/scan', qos_profile_sensor_data
        )
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_lidar_transform()
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_pose_callback,
            10,
        )
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
            'Nav2 LiDAR/AMCL box coordinator ready (retry limit: '
            f'{self.max_retries})'
        )

    @property
    def active_carrier(self):
        return self.carriers[self.active_index]

    def _start_callback(self, message):
        if not message.data or self.mode != 'WAITING':
            return
        if any(
            carrier.ground_truth is None
            or carrier.wheel_odometry is None
            or carrier.scan is None
            for carrier in self.carriers
        ):
            self._terminal_failure('SENSOR_DATA_NOT_READY')
            return

        self.active_index = 0
        for carrier in self.carriers:
            carrier.goal_index = 0
            carrier.arrived = False
            carrier.escaped = False
        if self.active_carrier.localized_pose is None:
            self.mode = 'LOCALIZING'
            self._request_localization()
        else:
            self.mode = 'SWITCHING'
            self.next_goal_at = time.monotonic() + 0.8
        self.failed_attempts = 0
        self.state_publisher.publish(String(data='NAV2_STARTING:A'))
        self.get_logger().info(
            'Nav2 delivery started: A/B/C -> left/center/right doors'
        )

    def _carrier_for_model(self, model):
        return next(item for item in self.carriers if item.model == model)

    def _ground_truth_callback(self, message, model):
        self._carrier_for_model(model).ground_truth = message

    def _wheel_odometry_callback(self, message, model):
        carrier = self._carrier_for_model(model)
        carrier.wheel_odometry = message
        if carrier is not self.active_carrier:
            return

        stamp = self.get_clock().now().to_msg()
        odometry = message
        odometry.header.stamp = stamp
        odometry.header.frame_id = 'odom'
        odometry.child_frame_id = 'base_link'
        self.nav_odometry_publisher.publish(odometry)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = odometry.pose.pose.position.x
        transform.transform.translation.y = odometry.pose.pose.position.y
        transform.transform.translation.z = odometry.pose.pose.position.z
        transform.transform.rotation = odometry.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def _scan_callback(self, message, model):
        carrier = self._carrier_for_model(model)
        carrier.scan = message
        if carrier is not self.active_carrier:
            return
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_scan'
        self.scan_publisher.publish(message)

    def _publish_lidar_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'base_scan'
        transform.transform.translation.z = 0.9
        transform.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(transform)

    def _request_localization(self):
        carrier = self.active_carrier
        if carrier.ground_truth is None:
            return
        x, y, yaw = carrier.truth_pose()
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = Time().to_msg()
        initial_pose.header.frame_id = 'map'
        initial_pose.pose.pose.position.x = x
        initial_pose.pose.pose.position.y = y
        initial_pose.pose.pose.orientation.z = sin(yaw / 2.0)
        initial_pose.pose.pose.orientation.w = cos(yaw / 2.0)
        initial_pose.pose.covariance[0] = 0.04
        initial_pose.pose.covariance[7] = 0.04
        initial_pose.pose.covariance[35] = 0.03
        carrier.localized_pose = None
        self.localization_pending = True
        self.localization_candidate = None
        self.localization_candidate_since = 0.0
        now = time.monotonic()
        if self.localization_requested_at == 0.0:
            self.localization_requested_at = now
        self.localization_last_publish_at = now
        self.initial_pose_publisher.publish(initial_pose)
        self.get_logger().info(
            f'AMCL initial pose requested for box {carrier.item_class}'
        )

    def _amcl_pose_callback(self, message):
        carrier = self.active_carrier
        if carrier.ground_truth is None:
            return
        pose = message.pose.pose
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
        localized_pose = (
            pose.position.x,
            pose.position.y,
            normalize_angle(yaw),
        )
        if self.localization_pending:
            truth_x, truth_y, _ = carrier.truth_pose()
            position_error = hypot(
                localized_pose[0] - truth_x,
                localized_pose[1] - truth_y,
            )
            if position_error > 1.0:
                self.localization_candidate = None
                self.localization_candidate_since = 0.0
                return
            now = time.monotonic()
            if self.localization_candidate is None:
                self.localization_candidate_since = now
            elif hypot(
                localized_pose[0] - self.localization_candidate[0],
                localized_pose[1] - self.localization_candidate[1],
            ) > 0.3:
                self.localization_candidate_since = now
            self.localization_candidate = localized_pose
            return

        truth_x, truth_y, _ = carrier.truth_pose()
        if hypot(
            localized_pose[0] - truth_x,
            localized_pose[1] - truth_y,
        ) > 1.5:
            self.get_logger().warning(
                f'AMCL pose rejected for box {carrier.item_class}: '
                'outside simulation safety bound',
                throttle_duration_sec=2.0,
            )
            return
        carrier.localized_pose = localized_pose

    def _complete_localization(self):
        carrier = self.active_carrier
        carrier.localized_pose = self.localization_candidate
        self.localization_pending = False
        self.localization_requested_at = 0.0
        self.localization_candidate = None
        self.localization_candidate_since = 0.0
        self.get_logger().info(
            f'AMCL localized box {carrier.item_class}'
        )
        if self.mode == 'LOCALIZING':
            self.mode = 'SWITCHING'
            self.next_goal_at = time.monotonic() + 0.8

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
        carrier = self.active_carrier
        sensors_ready = (
            carrier.ground_truth is not None
            and carrier.wheel_odometry is not None
            and carrier.scan is not None
        )
        if (
            self.mode == 'WAITING'
            and sensors_ready
            and carrier.localized_pose is None
            and not self.localization_pending
        ):
            self._request_localization()
        elif (
            self.localization_pending
            and self.localization_candidate is not None
            and now - self.localization_candidate_since > 1.0
        ):
            self._complete_localization()
        elif (
            self.localization_pending
            and now - self.localization_last_publish_at > 2.0
        ):
            self._request_localization()

        if (
            self.mode == 'LOCALIZING'
            and self.localization_requested_at == 0.0
            and sensors_ready
            and now >= self.next_goal_at
        ):
            self._request_localization()
        elif (
            self.mode == 'LOCALIZING'
            and self.localization_requested_at > 0.0
            and now - self.localization_requested_at > 15.0
        ):
            self._terminal_failure('LOCALIZATION_TIMEOUT')
        elif (
            self.mode in ('SWITCHING', 'RETRY_WAIT')
            and now >= self.next_goal_at
        ):
            if (
                self.mode == 'SWITCHING'
                and carrier.escape_pose is not None
                and not carrier.escaped
            ):
                self._begin_escape()
            else:
                self._send_goal()
        elif (
            self.mode == 'GOAL_PENDING'
            and now - self.goal_sent_at > 10.0
        ):
            self._attempt_failed('GOAL_RESPONSE_TIMEOUT')
        elif self.mode == 'ESCAPING':
            self._drive_escape(now)
        elif self.mode == 'NAVIGATING':
            self._monitor_navigation(now)

    def _begin_escape(self):
        carrier = self.active_carrier
        self.mode = 'ESCAPING'
        self.escape_started_at = time.monotonic()
        self.progress_pose = carrier.truth_pose()
        self.last_progress_at = self.escape_started_at
        self.state_publisher.publish(String(
            data=f'STAGING_ESCAPE:{carrier.item_class}'
        ))
        self.get_logger().info(
            f'Box {carrier.item_class} leaving tight loading bay before Nav2'
        )

    def _drive_escape(self, now):
        carrier = self.active_carrier
        if carrier.escape_pose is None:
            carrier.escaped = True
            self.mode = 'SWITCHING'
            self.next_goal_at = now + 0.2
            return

        x, y, yaw = carrier.truth_pose()
        goal_x, goal_y, goal_yaw = carrier.escape_pose
        dx = goal_x - x
        dy = goal_y - y
        distance = hypot(dx, dy)
        yaw_error = normalize_angle(goal_yaw - yaw)
        command = Twist()

        if distance <= 0.40 and abs(yaw_error) <= 0.50:
            self._finish_escape()
            return

        if distance > 0.18:
            target_yaw = atan2(dy, dx)
            heading_error = normalize_angle(target_yaw - yaw)
            if abs(heading_error) > 0.28:
                command.angular.z = clamp(1.25 * heading_error, -0.45, 0.45)
            else:
                command.linear.x = clamp(0.65 * distance, 0.08, 0.28)
                command.angular.z = clamp(1.1 * heading_error, -0.35, 0.35)
        else:
            if abs(yaw_error) > 0.16:
                command.angular.z = clamp(1.0 * yaw_error, -0.35, 0.35)
            else:
                self._finish_escape()
                return

        self.command_publishers[carrier.model].publish(command)

        moved = hypot(
            x - self.progress_pose[0],
            y - self.progress_pose[1],
        ) if self.progress_pose is not None else 0.0
        turned = abs(normalize_angle(
            yaw - self.progress_pose[2]
        )) if self.progress_pose is not None else 0.0
        if moved > 0.04 or turned > 0.08:
            self.progress_pose = (x, y, yaw)
            self.last_progress_at = now

        if now - self.escape_started_at > 60.0:
            self._terminal_failure('STAGING_ESCAPE_TIMEOUT')
        elif now - self.last_progress_at > self.stuck_timeout:
            self._terminal_failure('STAGING_ESCAPE_STUCK')

    def _finish_escape(self):
        carrier = self.active_carrier
        pose = carrier.truth_pose()
        self._stop_active()
        carrier.escaped = True
        carrier.localized_pose = None
        self.localization_requested_at = 0.0
        self.localization_pending = False
        self.localization_candidate = None
        self.localization_candidate_since = 0.0
        self.mode = 'LOCALIZING'
        self.next_goal_at = time.monotonic() + 0.5
        self.state_publisher.publish(String(
            data=f'STAGED_FOR_NAV2:{carrier.item_class}'
        ))
        self.get_logger().info(
            f'Box {carrier.item_class} staged safely at '
            f'({pose[0]:.2f}, {pose[1]:.2f}, yaw={pose[2]:.2f}); '
            'handing back to Nav2'
        )

    def _send_goal(self):
        if self.active_carrier.localized_pose is None:
            self.mode = 'LOCALIZING'
            self.localization_requested_at = 0.0
            self._request_localization()
            return
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

        distance = self._goal_distance()
        if (
            self.active_carrier.goal_index
            == len(self.active_carrier.goals) - 1
            and distance <= self.arrival_tolerance
        ):
            self.get_logger().info(
                f'Box {self.active_carrier.item_class} accepted at '
                f'dock standoff (error {distance:.2f} m)'
            )
            self.goal_serial += 1
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            self.goal_handle = None
            self._carrier_arrived(distance)
            return

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
        self.mode = 'LOCALIZING'
        self.localization_requested_at = 0.0
        self.localization_pending = False
        self.localization_candidate = None
        self.localization_candidate_since = 0.0
        self.active_carrier.localized_pose = None
        self.next_goal_at = time.monotonic() + 1.0

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
