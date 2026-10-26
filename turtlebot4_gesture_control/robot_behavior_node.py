#!/usr/bin/env python3
"""robot_behavior_node

Subscribes /detected_gesture (std_msgs/String), runs the BehaviorController
state machine and publishes geometry_msgs/Twist on /cmd_vel at a fixed rate.

Safety layers
  * Thumb_Up confirmation before any selected behaviour moves the robot
  * edge trigger + per-gesture cooldown (anti-noise)
  * Open_Palm stop / Closed_Fist cancel are never rate-limited
  * watchdog: no gesture message for `signal_timeout` s -> stop, back to IDLE
  * optional lidar guard: obstacle closer than `obstacle_stop_distance`
    in the direction of travel -> pause
  * zero Twist on every stop and on shutdown; velocity clamped
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .behavior_logic import (CIRCLE, FORWARD_BACKWARD, GREETING,
                             BehaviorController, State, circle_plan,
                             forward_backward_plan, greeting_plan,
                             plan_duration)

try:
    import tf2_ros
except ImportError:  # pragma: no cover
    tf2_ros = None


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RobotBehaviorNode(Node):

    def __init__(self):
        super().__init__('robot_behavior_node')
        p = self.declare_parameter
        p('gesture_topic', '/detected_gesture')
        p('cmd_vel_topic', '/cmd_vel')
        p('state_topic', '/behavior_state')
        p('scan_topic', '/scan')
        p('base_frame', 'base_link')
        p('control_rate', 20.0)
        p('signal_timeout', 1.0)
        p('cooldown', 1.0)
        p('wave_requires_confirm', False)
        p('max_linear', 0.3)
        p('max_angular', 1.5)
        # greeting
        p('greet_angular_speed', 1.2)
        p('greet_angle', 0.6)
        p('greet_repeats', 2)
        # forward-backward
        p('fb_speed', 0.2)
        p('fb_distance', 0.4)
        p('fb_repeats', 4)
        p('fb_pause', 0.4)
        # circle
        p('circle_speed', 0.2)
        p('circle_radius', 0.5)
        p('circle_laps', 1.0)
        p('circle_direction', 1)
        # lidar guard (0 disables)
        p('obstacle_stop_distance', 0.30)
        p('obstacle_ignore_below', 0.18)
        p('obstacle_half_angle_deg', 35.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        plans = {
            GREETING: greeting_plan(g('greet_angular_speed'), g('greet_angle'),
                                    g('greet_repeats')),
            FORWARD_BACKWARD: forward_backward_plan(g('fb_speed'), g('fb_distance'),
                                                    g('fb_repeats'), g('fb_pause')),
            CIRCLE: circle_plan(g('circle_speed'), g('circle_radius'),
                                g('circle_laps'), g('circle_direction')),
        }
        self.ctrl = BehaviorController(plans, cooldown=g('cooldown'),
                                       wave_requires_confirm=g('wave_requires_confirm'))
        for name, plan in plans.items():
            self.get_logger().info(f'plan {name}: {len(plan)} segments, '
                                   f'{plan_duration(plan):.1f} s')

        self.max_lin = float(g('max_linear'))
        self.max_ang = float(g('max_angular'))
        self.signal_timeout = float(g('signal_timeout'))
        self.base_frame = g('base_frame')
        self.obs_dist = float(g('obstacle_stop_distance'))
        self.obs_ignore = float(g('obstacle_ignore_below'))
        self.obs_half = math.radians(float(g('obstacle_half_angle_deg')))

        self.cmd_pub = self.create_publisher(Twist, g('cmd_vel_topic'), 10)
        self.state_pub = self.create_publisher(String, g('state_topic'), 10)
        self.create_subscription(String, g('gesture_topic'), self._on_gesture, 10)

        self.scan = None
        self.scan_yaw = None
        if self.obs_dist > 0.0:
            self.create_subscription(LaserScan, g('scan_topic'), self._on_scan,
                                     qos_profile_sensor_data)
            if tf2_ros is not None:
                self.tf_buffer = tf2_ros.Buffer()
                self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.last_msg_time = None
        self.signal_ok = False
        self.zero_ticks = 0
        self.last_tick = None
        self.tick = 0
        rate = float(g('control_rate'))
        self.timer = self.create_timer(1.0 / rate, self._on_timer)
        self.get_logger().info('robot_behavior_node ready - waiting for gestures')

    # ------------------------------------------------------------ callbacks
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_gesture(self, msg):
        now = self._now()
        self.last_msg_time = now
        if not self.signal_ok:
            self.signal_ok = True
            self.get_logger().info('gesture signal OK')
        was_moving = self.ctrl.is_moving
        self.ctrl.on_gesture(msg.data.strip(), now)
        self._flush_events()
        if was_moving and not self.ctrl.is_moving:
            self._stop_now()

    def _on_scan(self, msg):
        self.scan = msg
        if self.scan_yaw is None:
            self.scan_yaw = 0.0
            if tf2_ros is not None:
                try:
                    tf = self.tf_buffer.lookup_transform(
                        self.base_frame, msg.header.frame_id, Time())
                    self.scan_yaw = _yaw_from_quat(tf.transform.rotation)
                    self.get_logger().info(
                        f'lidar frame {msg.header.frame_id}: yaw '
                        f'{math.degrees(self.scan_yaw):.0f} deg w.r.t. {self.base_frame}')
                except Exception:  # noqa: BLE001  (TF not ready yet -> retry)
                    self.scan_yaw = None

    def _obstacle_ahead(self, linear):
        """Closest valid range in the direction of travel, or None."""
        if self.obs_dist <= 0.0 or self.scan is None or abs(linear) < 1e-3:
            return None
        s = self.scan
        heading = 0.0 if linear > 0 else math.pi
        offset = self.scan_yaw or 0.0
        closest = None
        for i, r in enumerate(s.ranges):
            if not math.isfinite(r) or r < max(self.obs_ignore, s.range_min) or r > s.range_max:
                continue
            ang = s.angle_min + i * s.angle_increment + offset
            diff = math.atan2(math.sin(ang - heading), math.cos(ang - heading))
            if abs(diff) <= self.obs_half and (closest is None or r < closest):
                closest = r
        if closest is not None and closest < self.obs_dist:
            return closest
        return None

    def _on_timer(self):
        now = self._now()
        dt = 0.0 if self.last_tick is None else max(0.0, now - self.last_tick)
        self.last_tick = now
        self.tick += 1

        # watchdog: control signal lost
        if self.last_msg_time is not None and now - self.last_msg_time > self.signal_timeout:
            if self.signal_ok:
                self.signal_ok = False
                self.get_logger().warn(
                    f'no /detected_gesture for {self.signal_timeout:.1f} s -> STOP')
                was_moving = self.ctrl.is_moving
                self.ctrl.on_signal_lost()
                self._flush_events()
                if was_moving:
                    self._stop_now()

        if self.ctrl.state == State.EXECUTING:
            seg = self.ctrl.current_segment
            dist = self._obstacle_ahead(seg.linear if seg else 0.0)
            if dist is not None:
                self.ctrl.on_obstacle(dist)
                self._flush_events()
                self._stop_now()

        if self.ctrl.state == State.EXECUTING:
            v, w = self.ctrl.step(dt)
            self._flush_events()
            if self.ctrl.state == State.EXECUTING:
                self._publish(v, w)
            else:
                self._stop_now()          # plan finished
        elif self.zero_ticks > 0:
            self.zero_ticks -= 1
            self._publish(0.0, 0.0)

        if self.tick % 4 == 0:
            text = self.ctrl.status_text()
            if not self.signal_ok:
                text += ' | NO GESTURE SIGNAL'
            self.state_pub.publish(String(data=text))

    # -------------------------------------------------------------- helpers
    def _flush_events(self):
        for ev in self.ctrl.drain_events():
            self.get_logger().info(ev)

    def _publish(self, v, w):
        t = Twist()
        t.linear.x = max(-self.max_lin, min(self.max_lin, float(v)))
        t.angular.z = max(-self.max_ang, min(self.max_ang, float(w)))
        self.cmd_pub.publish(t)

    def _stop_now(self):
        """Publish zero now and keep publishing zeros for ~0.5 s, then go quiet
        so other tools (teleop, ros2 topic pub) can still use /cmd_vel."""
        self._publish(0.0, 0.0)
        self.zero_ticks = 10

    def destroy_node(self):
        try:
            for _ in range(3):
                self._publish(0.0, 0.0)
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotBehaviorNode()
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
