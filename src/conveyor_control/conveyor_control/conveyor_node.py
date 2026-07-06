"""Command the Gazebo conveyor and stop items at the pickup zone."""

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import LogicalCameraImage
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import String


class ConveyorNode(Node):
    """Drive the physical belt and monitor workpiece poses."""

    def __init__(self):
        super().__init__('conveyor_node')
        self.declare_parameter('speed', 0.35)
        self.declare_parameter('auto_start', True)

        self.speed = float(self.get_parameter('speed').value)
        self.running = bool(self.get_parameter('auto_start').value)
        self.stopped_item = ''

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

        for detected_model in message.model:
            item_name = detected_model.name
            if not item_name.startswith('item_'):
                continue

            self.stopped_item = item_name
            self.get_logger().info(f'Pickup sensor detected {item_name}')
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
