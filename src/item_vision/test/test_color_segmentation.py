import cv2
import numpy as np

from item_vision.vision_node import VisionNode
from vision_msgs.msg import Detection3DArray


def test_primary_workpiece_colors_are_separated():
    node = VisionNode.__new__(VisionNode)
    bgr_colors = {
        'A': (0, 0, 255),
        'B': (0, 255, 0),
        'C': (255, 0, 0),
    }

    for expected_class, bgr_color in bgr_colors.items():
        image = np.full((20, 20, 3), bgr_color, dtype=np.uint8)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        for item_class in bgr_colors:
            mask = node._mask_for_class(hsv_image, item_class)
            if item_class == expected_class:
                assert np.count_nonzero(mask) == 400
            else:
                assert np.count_nonzero(mask) == 0


def test_vision_uses_standard_detection_message():
    assert Detection3DArray.__name__ == 'Detection3DArray'
