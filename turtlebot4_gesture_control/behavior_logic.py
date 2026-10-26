"""Behaviour plans and the gesture -> behaviour state machine (pure Python).

States
    IDLE       robot stopped, nothing selected
    SELECTED   a behaviour is chosen and waits for Thumb_Up to confirm
    EXECUTING  the behaviour's velocity plan is being played
    PAUSED     stopped by Open_Palm (or an obstacle); Thumb_Up resumes,
               Closed_Fist cancels

Gesture -> action (default mapping)
    Wave         greet       (IDLE only; runs right away, or needs Thumb_Up
                              if wave_requires_confirm=True)
    Pointing_Up  select FORWARD_BACKWARD
    Victory      select CIRCLE
    Thumb_Up     confirm / resume
    Open_Palm    stop  (EXECUTING -> PAUSED)
    Closed_Fist  cancel (any -> IDLE)

Anti-noise: actions fire only on a *change* of the received label (edge
trigger) and a non-safety action is ignored if the same gesture was accepted
less than `cooldown` seconds ago. Stop / cancel are never rate-limited.
"""

import math
from dataclasses import dataclass
from enum import Enum


@dataclass(frozen=True)
class Segment:
    duration: float   # seconds
    linear: float     # m/s
    angular: float    # rad/s
    label: str = ''


def greeting_plan(angular_speed=1.2, angle=0.6, repeats=2, pause=0.2):
    """Turn left `angle`, right 2*angle, back to centre - like shaking head."""
    w = abs(angular_speed)
    t = abs(angle) / w
    plan = []
    for i in range(int(repeats)):
        plan += [
            Segment(t, 0.0, +w, f'greet {i + 1}: left'),
            Segment(2 * t, 0.0, -w, f'greet {i + 1}: right'),
            Segment(t, 0.0, +w, f'greet {i + 1}: centre'),
        ]
        if pause > 0:
            plan.append(Segment(pause, 0.0, 0.0, 'pause'))
    return plan


def forward_backward_plan(speed=0.2, distance=0.4, repeats=4, pause=0.4):
    """Forward `distance`, back `distance`, `repeats` times (net displacement 0)."""
    v = abs(speed)
    t = abs(distance) / v
    plan = []
    for i in range(int(repeats)):
        plan.append(Segment(t, +v, 0.0, f'cycle {i + 1}/{repeats}: forward'))
        if pause > 0:
            plan.append(Segment(pause, 0.0, 0.0, 'pause'))
        plan.append(Segment(t, -v, 0.0, f'cycle {i + 1}/{repeats}: backward'))
        if pause > 0:
            plan.append(Segment(pause, 0.0, 0.0, 'pause'))
    return plan


def circle_plan(speed=0.2, radius=0.5, laps=1.0, direction=1):
    """Constant v and w = v / r for `laps` full turns (counter-clockwise if direction>0)."""
    v = abs(speed)
    w = math.copysign(v / abs(radius), direction)
    t = 2.0 * math.pi * abs(radius) * float(laps) / v
    return [Segment(t, v, w, f'circle r={radius:.2f} m')]


def plan_duration(plan):
    return sum(s.duration for s in plan)


class State(Enum):
    IDLE = 'IDLE'
    SELECTED = 'SELECTED'
    EXECUTING = 'EXECUTING'
    PAUSED = 'PAUSED'


GREETING = 'GREETING'
FORWARD_BACKWARD = 'FORWARD_BACKWARD'
CIRCLE = 'CIRCLE'

DEFAULT_GESTURE_MAP = {
    'Wave': ('greet', GREETING),
    'Pointing_Up': ('select', FORWARD_BACKWARD),
    'Victory': ('select', CIRCLE),
    'Thumb_Up': ('confirm', None),
    'Open_Palm': ('stop', None),
    'Closed_Fist': ('cancel', None),
}

SAFETY_ACTIONS = ('stop', 'cancel')


class BehaviorController:
    """Finite-state machine; call on_gesture() for each received label and
    step(dt) at a fixed rate to get the (linear, angular) command."""

    def __init__(self, plans, gesture_map=None, cooldown=1.0,
                 wave_requires_confirm=False):
        self.plans = dict(plans)
        self.gesture_map = dict(gesture_map or DEFAULT_GESTURE_MAP)
        self.cooldown = float(cooldown)
        self.wave_requires_confirm = bool(wave_requires_confirm)

        self.state = State.IDLE
        self.selected = None
        self.message = 'Waiting for a gesture'
        self._last_label = None
        self._last_accept = {}
        self._seg_idx = 0
        self._seg_elapsed = 0.0
        self._done_time = 0.0
        self.events = []   # human-readable log lines, drained by the node

    # ------------------------------------------------------------ helpers
    def _log(self, text):
        self.message = text
        self.events.append(text)

    def drain_events(self):
        ev, self.events = self.events, []
        return ev

    def _start(self, name, reset=True):
        self.selected = name
        if reset:
            self._seg_idx = 0
            self._seg_elapsed = 0.0
            self._done_time = 0.0
        self.state = State.EXECUTING

    def _to_idle(self, text):
        self.state = State.IDLE
        self.selected = None
        self._seg_idx = 0
        self._seg_elapsed = 0.0
        self._done_time = 0.0
        self._log(text)

    @property
    def is_moving(self):
        return self.state == State.EXECUTING

    @property
    def progress(self):
        if not self.selected or self.selected not in self.plans:
            return 0.0
        total = plan_duration(self.plans[self.selected])
        return 0.0 if total <= 0 else min(1.0, self._done_time / total)

    @property
    def current_segment(self):
        if self.state not in (State.EXECUTING, State.PAUSED) or not self.selected:
            return None
        plan = self.plans[self.selected]
        return plan[self._seg_idx] if self._seg_idx < len(plan) else None

    def status_text(self):
        """ASCII only - drawn with cv2.putText on the webcam window."""
        if self.state == State.IDLE:
            return 'IDLE | ' + self.message
        if self.state == State.SELECTED:
            return f'SELECTED {self.selected} | show Thumb_Up to confirm'
        pct = int(round(self.progress * 100))
        seg = self.current_segment
        seg_txt = f' | {seg.label}' if seg and seg.label else ''
        if self.state == State.EXECUTING:
            return f'EXECUTING {self.selected} {pct}%{seg_txt}'
        return f'PAUSED {self.selected} {pct}% | Thumb_Up resume, Fist cancel | {self.message}'

    # ------------------------------------------------------------- inputs
    def on_gesture(self, label, now):
        """Feed one received label. Returns the action taken or None."""
        if label == self._last_label:
            return None                      # edge trigger: only on change
        self._last_label = label

        entry = self.gesture_map.get(label)
        if entry is None:
            return None                      # None / unused gestures
        action, arg = entry

        if action not in SAFETY_ACTIONS:
            last = self._last_accept.get(label)
            if last is not None and now - last < self.cooldown:
                self.events.append(f'ignored {label} (cooldown)')
                return None
        self._last_accept[label] = now

        handler = getattr(self, '_on_' + action)
        return action if handler(label, arg) else None

    def _on_stop(self, label, _):
        if self.state == State.EXECUTING:
            self.state = State.PAUSED
            self._log(f'{label}: STOP - {self.selected} paused')
            return True
        self.events.append(f'{label}: STOP (robot already stopped)')
        return True

    def _on_cancel(self, label, _):
        if self.state == State.IDLE:
            self.events.append(f'{label}: nothing to cancel')
            return True
        self._to_idle(f'{label}: cancelled {self.selected}')
        return True

    def _on_select(self, label, name):
        if self.state == State.EXECUTING:
            self.events.append(f'ignored {label}: busy (Open_Palm stop / Fist cancel first)')
            return False
        self.selected = name
        self.state = State.SELECTED
        self._seg_idx = 0
        self._seg_elapsed = 0.0
        self._done_time = 0.0
        self._log(f'{label}: selected {name}, waiting for Thumb_Up')
        return True

    def _on_confirm(self, label, _):
        if self.state == State.SELECTED:
            self._start(self.selected, reset=True)
            self._log(f'{label}: confirmed, executing {self.selected}')
            return True
        if self.state == State.PAUSED:
            self._start(self.selected, reset=False)
            self._log(f'{label}: resumed {self.selected}')
            return True
        self.events.append(f'ignored {label}: no behaviour selected')
        return False

    def _on_greet(self, label, name):
        if self.state != State.IDLE:
            self.events.append(f'ignored {label}: robot not idle')
            return False
        if self.wave_requires_confirm:
            return self._on_select(label, name)
        self._start(name, reset=True)
        self._log(f'{label}: hello! executing {name}')
        return True

    def on_signal_lost(self):
        """Gesture stream stopped (camera/node died): stop and reset."""
        self._last_label = None
        if self.state in (State.EXECUTING, State.PAUSED, State.SELECTED):
            self._to_idle('SIGNAL LOST - stopped, back to IDLE')
            return True
        self.message = 'SIGNAL LOST'
        return False

    def on_obstacle(self, distance):
        if self.state == State.EXECUTING:
            self.state = State.PAUSED
            self._log(f'OBSTACLE at {distance:.2f} m - paused')
            return True
        return False

    # -------------------------------------------------------------- output
    def step(self, dt):
        """Advance the active plan by dt seconds -> (linear, angular)."""
        if self.state != State.EXECUTING:
            return 0.0, 0.0
        plan = self.plans[self.selected]
        dt = max(0.0, float(dt))
        self._seg_elapsed += dt
        self._done_time += dt
        while self._seg_idx < len(plan) and self._seg_elapsed >= plan[self._seg_idx].duration:
            self._seg_elapsed -= plan[self._seg_idx].duration
            self._seg_idx += 1
        if self._seg_idx >= len(plan):
            self._to_idle(f'{self.selected} finished')
            return 0.0, 0.0
        seg = plan[self._seg_idx]
        return seg.linear, seg.angular
