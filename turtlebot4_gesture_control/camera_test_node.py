#!/usr/bin/env python3
"""camera_test_node - Part 1: read and display the laptop webcam with OpenCV.

ros2 run turtlebot4_gesture_control camera_test_node --ros-args -p camera_index:=0
Press q / ESC (or close the window) to quit.
"""

import time

import cv2
import rclpy
from rclpy.node import Node

WINDOW = 'Webcam test (OpenCV)'


class CameraTestNode(Node):

    def __init__(self):
        super().__init__('camera_test_node')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        idx = int(self.get_parameter('camera_index').value)
        self.cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(idx)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.get_parameter('frame_width').value))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.get_parameter('frame_height').value))
        if not self.cap.isOpened():
            raise RuntimeError(f'Cannot open webcam index {idx} (check: v4l2-ctl --list-devices)')
        self.get_logger().info(f'Webcam {idx} opened: '
                               f'{int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x'
                               f'{int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}')

    def run(self):
        fps, last = 0.0, time.monotonic()
        while rclpy.ok():
            ok, frame = self.cap.read()
            if not ok:
                self.get_logger().warn('frame read failed', throttle_duration_sec=2.0)
                time.sleep(0.05)
                continue
            frame = cv2.flip(frame, 1)
            now = time.monotonic()
            fps = 0.9 * fps + 0.1 / max(now - last, 1e-3)
            last = now
            h, w = frame.shape[:2]
            cv2.putText(frame, f'{w}x{h}  FPS {fps:4.1f}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.imshow(WINDOW, frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraTestNode()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
