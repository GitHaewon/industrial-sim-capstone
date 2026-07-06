"""End-to-end state coordinator for the factory demonstration."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String


class FactoryManager(Node):
    """Count sorted items and dispatch the loaded outbound AMR."""

    def __init__(self):
        super().__init__('factory_manager')
        self.declare_parameter('target_item_count', 3)
        self.target_count = self.get_parameter(
            'target_item_count'
        ).get_parameter_value().integer_value
        self.item_count = 0
        self.delivery_requested = False

        self.state_publisher = self.create_publisher(
            String, '/factory/state', 10
        )
        self.count_publisher = self.create_publisher(
            Int32, '/box/item_count', 10
        )
        self.box_ready_publisher = self.create_publisher(
            Bool, '/box/ready', 10
        )
        self.delivery_publisher = self.create_publisher(
            Bool, '/amr/start_delivery', 10
        )
        self.create_subscription(
            Bool, '/item_spawner/ready', self._batch_ready_callback, 10
        )
        self.create_subscription(
            String, '/conveyor/state', self._conveyor_callback, 10
        )
        self.create_subscription(
            String, '/arm/state', self._arm_state_callback, 10
        )
        self.create_subscription(
            Bool, '/arm/task_complete', self._arm_complete_callback, 10
        )
        self.create_subscription(
            Bool,
            '/amr/delivery_complete',
            self._delivery_complete_callback,
            10,
        )
        self.create_subscription(
            Bool,
            '/amr/delivery_failed',
            self._delivery_failed_callback,
            10,
        )
        self.create_subscription(
            Bool,
            '/amr/return_complete',
            self._return_complete_callback,
            10,
        )
        self._publish_state('IDLE')
        self.get_logger().info(
            f'Factory manager target item count={self.target_count}'
        )

    def _publish_state(self, state):
        self.state_publisher.publish(String(data=state))
        self.get_logger().info(f'FACTORY_STATE: {state}')

    def _batch_ready_callback(self, message):
        if message.data and self.item_count == 0:
            self._publish_state('CONVEYOR_RUNNING')

    def _conveyor_callback(self, message):
        if (
            message.data == 'STOPPED:ITEM_READY'
            and self.item_count < self.target_count
        ):
            self._publish_state('ITEM_READY')
        elif (
            message.data == 'RUNNING:START_COMMAND'
            and self.item_count < self.target_count
        ):
            self._publish_state('CONVEYOR_RUNNING')

    def _arm_state_callback(self, message):
        if message.data.startswith('ATTACHING'):
            self._publish_state('PICKING')

    def _arm_complete_callback(self, message):
        if not message.data or self.item_count >= self.target_count:
            return
        self.item_count += 1
        self.count_publisher.publish(Int32(data=self.item_count))
        self.get_logger().info(
            f'Box load progress: {self.item_count}/{self.target_count}'
        )
        if self.item_count == self.target_count:
            self.box_ready_publisher.publish(Bool(data=True))
            self._publish_state('BOX_READY')
            self.delivery_publisher.publish(Bool(data=True))
            self.delivery_requested = True
            self._publish_state('AMR_MOVING')

    def _delivery_complete_callback(self, message):
        if message.data and self.delivery_requested:
            self._publish_state('DELIVERED')

    def _delivery_failed_callback(self, message):
        if message.data and self.delivery_requested:
            self._publish_state('DELIVERY_FAILED')

    def _return_complete_callback(self, message):
        if not message.data or not self.delivery_requested:
            return
        self.item_count = 0
        self.delivery_requested = False
        self.count_publisher.publish(Int32(data=0))
        self.box_ready_publisher.publish(Bool(data=False))
        self._publish_state('RESETTING')


def main(args=None):
    rclpy.init(args=args)
    node = FactoryManager()
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
