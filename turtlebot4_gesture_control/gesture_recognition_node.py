#!/usr/bin/env python3
"""gesture_recognition_node

Webcam (OpenCV) -> MediaPipe Gesture Recognizer -> temporal filter + wave
detector -> publishes the stable gesture label on /detected_gesture
(std_msgs/String) every frame (the stream also works as a heartbeat for the
behaviour node). Shows the camera window with landmarks, gesture name,
confidence and the robot state received on /behavior_state.
"""

import os
import time

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision
from rclpy.node import Node
from std_msgs.msg import String

from .gesture_logic import (OPEN_PALM, WAVE, GestureFilter, GestureStabilizer,
                            WaveDetector)

PKG = 'turtlebot4_gesture_control'
WINDOW = 'Gesture Recognition - TurtleBot 4'

HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12), (9, 13), (13, 14), (14, 15),
    (15, 16), (13, 17), (17, 18), (18, 19), (19, 20), (0, 17),
]
PALM_IDS = (0, 5, 9, 13, 17)

# Gesture -> robot action, drawn as a legend on the window
LEGEND = [
    ('Wave', 'greet (turn L-R)'),
    ('Pointing_Up', 'select fwd-back x4'),
    ('Victory', 'select circle'),
    ('Thumb_Up', 'confirm / resume'),
    ('Open_Palm', 'STOP'),
    ('Closed_Fist', 'cancel -> idle'),
]

# BGR colours by robot state
STATE_COLOURS = {
    'IDLE': (180, 180, 180),
    'SELECTED': (0, 215, 255),
    'EXECUTING': (0, 200, 0),
    'PAUSED': (0, 0, 255),
}


class GestureRecognitionNode(Node):

    def __init__(self):
        super().__init__('gesture_recognition_node')
        p = self.declare_parameter
        p('camera_index', 0)
        p('frame_width', 640)
        p('frame_height', 480)
        p('mirror', True)
        p('model_path', '')
        p('num_hands', 1)
        p('min_detection_confidence', 0.6)
        p('min_score', 0.6)
        p('stable_frames', 6)
        p('stop_frames', 3)
        p('none_frames', 8)
        p('wave_enabled', True)
        p('wave_window', 1.5)
        p('wave_amplitude', 0.06)
        p('wave_swings', 3)
        p('gesture_topic', '/detected_gesture')
        p('state_topic', '/behavior_state')
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.mirror = bool(g('mirror'))
        self.camera_index = int(g('camera_index'))
        self.width, self.height = int(g('frame_width')), int(g('frame_height'))

        self.pub = self.create_publisher(String, g('gesture_topic'), 10)
        self.robot_state = 'no /behavior_state yet'
        self.create_subscription(String, g('state_topic'), self._on_state, 10)

        self.recognizer = self._create_recognizer(
            self._resolve_model(g('model_path')), int(g('num_hands')),
            float(g('min_detection_confidence')))

        stabilizer = GestureStabilizer(
            min_score=g('min_score'), stable_frames=g('stable_frames'),
            fast_frames=g('stop_frames'), none_frames=g('none_frames'))
        wave = WaveDetector(g('wave_window'), g('wave_amplitude'),
                            g('wave_swings')) if g('wave_enabled') else None
        self.filter = GestureFilter(stabilizer, wave)

        self.cap = None
        self._open_camera()
        self._last_ts_ms = 0
        self._fps = 0.0
        self._last_published = None

    # ---------------------------------------------------------------- setup
    def _resolve_model(self, path):
        candidates = [path] if path else []
        try:
            share = get_package_share_directory(PKG)
            candidates.append(os.path.join(share, 'models', 'gesture_recognizer.task'))
        except Exception:  # noqa: BLE001
            pass
        for c in candidates:
            if c and os.path.isfile(os.path.expanduser(c)):
                self.get_logger().info(f'MediaPipe model: {c}')
                return os.path.expanduser(c)
        raise FileNotFoundError(
            'gesture_recognizer.task not found. Run scripts/download_model.sh in the '
            'package source, rebuild with colcon, or pass -p model_path:=/abs/path.task')

    def _create_recognizer(self, model_path, num_hands, min_conf):
        options = vision.GestureRecognizerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=model_path),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=num_hands,
            min_hand_detection_confidence=min_conf,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5)
        return vision.GestureRecognizer.create_from_options(options)

    def _open_camera(self):
        if self.cap is not None:
            self.cap.release()
        cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap = cv2.VideoCapture(self.camera_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap = cap
        if cap.isOpened():
            self.get_logger().info(f'Webcam /dev/video{self.camera_index} opened')
        else:
            self.get_logger().error(f'Cannot open webcam index {self.camera_index}')

    def _on_state(self, msg):
        self.robot_state = msg.data

    # ----------------------------------------------------------------- loop
    def run(self):
        last_t = time.monotonic()
        last_reopen = 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            ok, frame = (self.cap.read() if self.cap is not None else (False, None))
            now = time.monotonic()
            if not ok or frame is None:
                # Publish nothing -> robot_behavior_node watchdog stops the robot.
                self.get_logger().warn('Webcam frame lost - not publishing',
                                       throttle_duration_sec=2.0)
                if now - last_reopen > 2.0:
                    last_reopen = now
                    self._open_camera()
                if self._show_lost():
                    break
                time.sleep(0.05)
                continue

            if self.mirror:
                frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            ts = max(self._last_ts_ms + 1, int(now * 1000))
            self._last_ts_ms = ts
            result = self.recognizer.recognize_for_video(mp_image, ts)

            raw_label, raw_score, landmarks = 'None', 0.0, None
            if result.gestures:
                top = result.gestures[0][0]
                raw_label, raw_score = top.category_name or 'None', float(top.score)
            if result.hand_landmarks:
                landmarks = result.hand_landmarks[0]
            palm_x = (sum(landmarks[i].x for i in PALM_IDS) / len(PALM_IDS)
                      if landmarks else None)

            out = self.filter.update(now, raw_label, raw_score, palm_x)
            self.pub.publish(String(data=out))
            if out != self._last_published:
                self.get_logger().info(f'gesture -> {out}  (raw {raw_label} {raw_score:.2f})')
                self._last_published = out

            dt = now - last_t
            last_t = now
            if dt > 0:
                self._fps = 0.9 * self._fps + 0.1 * (1.0 / dt)

            self._draw(frame, landmarks, raw_label, raw_score, out)
            cv2.imshow(WINDOW, frame)
            if self._should_quit():
                break

    # -------------------------------------------------------------- drawing
    def _should_quit(self):
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q')):
            return True
        try:
            return cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1
        except cv2.error:
            return True

    def _show_lost(self):
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        cv2.putText(img, 'WEBCAM SIGNAL LOST', (40, self.height // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(img, 'robot is stopped by the watchdog', (40, self.height // 2 + 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow(WINDOW, img)
        return self._should_quit()

    @staticmethod
    def _text(img, txt, org, scale=0.6, colour=(255, 255, 255), thick=1):
        cv2.putText(img, txt, org, cv2.FONT_HERSHEY_SIMPLEX, scale, colour,
                    thick, cv2.LINE_AA)

    def _draw(self, img, landmarks, raw_label, raw_score, out):
        h, w = img.shape[:2]
        if landmarks:
            pts = [(int(lm.x * w), int(lm.y * h)) for lm in landmarks]
            for a, b in HAND_CONNECTIONS:
                cv2.line(img, pts[a], pts[b], (255, 255, 255), 2, cv2.LINE_AA)
            for x, y in pts:
                cv2.circle(img, (x, y), 4, (0, 140, 255), -1, cv2.LINE_AA)

        # dark panels (top: status, bottom-left: legend)
        legend_h = 18 * len(LEGEND) + 10
        overlay = img.copy()
        cv2.rectangle(overlay, (0, 0), (w, 118), (0, 0, 0), -1)
        cv2.rectangle(overlay, (0, h - legend_h), (300, h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, img, 0.5, 0, img)

        conf_col = (0, 220, 0) if raw_score >= self.filter.stabilizer.min_score else (0, 165, 255)
        self._text(img, f'Gesture: {raw_label}  ({raw_score * 100:.0f}%)', (10, 28),
                   0.8, conf_col, 2)
        # confidence bar
        cv2.rectangle(img, (w - 170, 12), (w - 20, 30), (90, 90, 90), 1)
        cv2.rectangle(img, (w - 170, 12), (w - 170 + int(150 * raw_score), 30), conf_col, -1)

        stab = self.filter.stabilizer
        self._text(img, f'Published: {out}', (10, 58), 0.7,
                   (0, 255, 255) if out == WAVE else (255, 255, 255), 2)
        bar_w = int(120 * stab.progress)
        cv2.rectangle(img, (w - 170, 44), (w - 50, 58), (90, 90, 90), 1)
        cv2.rectangle(img, (w - 170, 44), (w - 170 + bar_w, 58), (255, 200, 0), -1)
        self._text(img, 'stable', (w - 45, 56), 0.4)

        if self.filter.wave is not None:
            sw = self.filter.wave_swings
            need = self.filter.wave.min_swings
            wave_txt = 'Wave: DETECTED' if out == WAVE else f'Wave: {sw}/{need} swings'
            if raw_label == OPEN_PALM or out == WAVE:
                self._text(img, wave_txt, (w - 230, 84), 0.55, (0, 255, 255))

        state_key = self.robot_state.split(' ', 1)[0]
        colour = STATE_COLOURS.get(state_key, (200, 200, 200))
        self._text(img, f'Robot: {self.robot_state}'[:80], (10, 88), 0.55, colour, 1)
        self._text(img, f'FPS {self._fps:4.1f}', (10, 110), 0.45, (200, 200, 200))
        cv2.rectangle(img, (0, 0), (w - 1, h - 1), colour, 4)

        # legend (bottom-left)
        y0 = h - 12 - 18 * (len(LEGEND) - 1)
        for i, (gname, action) in enumerate(LEGEND):
            col = (0, 255, 255) if gname == out else (230, 230, 230)
            self._text(img, f'{gname:<11} -> {action}', (10, y0 + 18 * i), 0.45, col)

    def destroy_node(self):
        try:
            if self.cap is not None:
                self.cap.release()
            self.recognizer.close()
            cv2.destroyAllWindows()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GestureRecognitionNode()
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
