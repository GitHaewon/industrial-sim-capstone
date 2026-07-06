"""Detect colored workpieces and estimate their 3D pickup positions."""

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection3D
from vision_msgs.msg import Detection3DArray
from vision_msgs.msg import ObjectHypothesisWithPose


class VisionNode(Node):
    """Classify A/B/C workpieces from the simulated overhead RGB-D camera."""

    COLOR_RANGES = {
        'A': (
            (np.array([0, 100, 60]), np.array([12, 255, 255])),
            (np.array([168, 100, 60]), np.array([179, 255, 255])),
        ),
        'B': (
            (np.array([38, 80, 50]), np.array([88, 255, 255])),
        ),
        'C': (
            (np.array([92, 90, 50]), np.array([138, 255, 255])),
        ),
    }

    DRAW_COLORS = {
        'A': (0, 0, 255),
        'B': (0, 255, 0),
        'C': (255, 80, 0),
    }

    def __init__(self):
        super().__init__('vision_node')
        self.declare_parameter('minimum_area', 180.0)
        self.declare_parameter('camera_world_x', -0.5)
        self.declare_parameter('camera_world_y', 0.0)
        self.declare_parameter('camera_world_z', 4.05)
        self.declare_parameter('conveyor_max_abs_y', 0.65)
        self.declare_parameter('conveyor_min_z', 0.75)

        self.minimum_area = float(
            self.get_parameter('minimum_area').value
        )
        self.camera_world_x = float(
            self.get_parameter('camera_world_x').value
        )
        self.camera_world_y = float(
            self.get_parameter('camera_world_y').value
        )
        self.camera_world_z = float(
            self.get_parameter('camera_world_z').value
        )
        self.conveyor_max_abs_y = float(
            self.get_parameter('conveyor_max_abs_y').value
        )
        self.conveyor_min_z = float(
            self.get_parameter('conveyor_min_z').value
        )

        self.bridge = CvBridge()
        self.depth_image = None
        self.camera_info = None
        self.detection_publisher = self.create_publisher(
            Detection3DArray, '/vision/detections', 10
        )
        self.debug_publisher = self.create_publisher(
            Image, '/vision/debug_image', 10
        )
        self.status_publisher = self.create_publisher(
            String, '/vision/status', 10
        )
        self.create_subscription(
            Image,
            '/factory/camera/depth_image',
            self._depth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            '/factory/camera/camera_info',
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            '/factory/camera/image',
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info('RGB-D item vision started')

    def _depth_callback(self, message):
        depth = self.bridge.imgmsg_to_cv2(
            message, desired_encoding='passthrough'
        )
        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) / 1000.0
        self.depth_image = np.asarray(depth, dtype=np.float32)

    def _camera_info_callback(self, message):
        self.camera_info = message

    def _mask_for_class(self, hsv_image, item_class):
        mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
        for lower, upper in self.COLOR_RANGES[item_class]:
            mask = cv2.bitwise_or(
                mask, cv2.inRange(hsv_image, lower, upper)
            )
        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    def _depth_at(self, u, v):
        if self.depth_image is None:
            return None
        if not (
            0 <= v < self.depth_image.shape[0]
            and 0 <= u < self.depth_image.shape[1]
        ):
            return None

        y_min, y_max = max(0, v - 4), min(self.depth_image.shape[0], v + 5)
        x_min, x_max = max(0, u - 4), min(self.depth_image.shape[1], u + 5)
        window = self.depth_image[y_min:y_max, x_min:x_max]
        valid = window[np.isfinite(window) & (window > 0.1)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _make_detection(
        self, item_class, contour, hsv_image, header
    ):
        moments = cv2.moments(contour)
        if moments['m00'] == 0:
            return None
        u = int(moments['m10'] / moments['m00'])
        v = int(moments['m01'] / moments['m00'])
        depth = self._depth_at(u, v)
        if depth is None or self.camera_info is None:
            return None

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
        if fx <= 0 or fy <= 0:
            return None

        camera_x = (u - cx) * depth / fx
        camera_y = (v - cy) * depth / fy
        area = cv2.contourArea(contour)
        hull_area = cv2.contourArea(cv2.convexHull(contour))
        solidity = area / hull_area if hull_area > 0 else 0.0
        mean_saturation = cv2.mean(
            hsv_image[:, :, 1],
            mask=cv2.drawContours(
                np.zeros(hsv_image.shape[:2], dtype=np.uint8),
                [contour],
                -1,
                255,
                thickness=-1,
            ),
        )[0]
        confidence = min(
            1.0, 0.55 * solidity + 0.45 * mean_saturation / 255.0
        )

        perimeter = cv2.arcLength(contour, True)
        vertices = len(
            cv2.approxPolyDP(contour, 0.04 * perimeter, True)
        )
        shape = 'box' if vertices <= 5 else 'round'

        world_x = self.camera_world_x - camera_y
        world_y = self.camera_world_y - camera_x
        world_z = self.camera_world_z - depth

        detection = Detection3D()
        detection.header = header
        detection.header.frame_id = 'world'
        detection.id = f'{item_class}:{u}:{v}'
        detection.bbox.center.position.x = world_x
        detection.bbox.center.position.y = world_y
        detection.bbox.center.position.z = world_z
        detection.bbox.center.orientation.w = 1.0
        detection.bbox.size.x = 0.35
        detection.bbox.size.y = 0.35
        detection.bbox.size.z = 0.32

        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = item_class
        hypothesis.hypothesis.score = float(confidence)
        hypothesis.pose.pose.position.x = world_x
        hypothesis.pose.pose.position.y = world_y
        hypothesis.pose.pose.position.z = world_z
        hypothesis.pose.pose.orientation.w = 1.0
        detection.results = [hypothesis]
        return detection, u, v, confidence, shape

    def _image_callback(self, message):
        if self.depth_image is None or self.camera_info is None:
            return

        bgr_image = self.bridge.imgmsg_to_cv2(
            message, desired_encoding='bgr8'
        )
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        debug_image = bgr_image.copy()
        detections = []

        for item_class in ('A', 'B', 'C'):
            mask = self._mask_for_class(hsv_image, item_class)
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            for contour in contours:
                if cv2.contourArea(contour) < self.minimum_area:
                    continue
                result = self._make_detection(
                    item_class, contour, hsv_image, message.header
                )
                if result is None:
                    continue
                detection, u, v, confidence, _shape = result
                position = detection.bbox.center.position
                if (
                    abs(position.y) > self.conveyor_max_abs_y
                    or position.z < self.conveyor_min_z
                ):
                    continue
                detections.append(detection)
                color = self.DRAW_COLORS[item_class]
                cv2.drawContours(debug_image, [contour], -1, color, 2)
                cv2.putText(
                    debug_image,
                    f'{item_class} {confidence:.2f}',
                    (u + 8, v - 8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    color,
                    2,
                )

        detections.sort(
            key=lambda item: item.bbox.center.position.x, reverse=True
        )
        output = Detection3DArray()
        output.header = message.header
        output.detections = detections
        self.detection_publisher.publish(output)
        self.status_publisher.publish(
            String(data=f'DETECTED:{len(detections)}')
        )

        debug_message = self.bridge.cv2_to_imgmsg(
            debug_image, encoding='bgr8'
        )
        debug_message.header = message.header
        self.debug_publisher.publish(debug_message)


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
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
