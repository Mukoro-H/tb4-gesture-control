"""Unit tests for the ROS-independent logic. Run: python3 -m pytest test/"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from turtlebot4_gesture_control.behavior_logic import (  # noqa: E402
    CIRCLE, FORWARD_BACKWARD, GREETING, BehaviorController, State,
    circle_plan, forward_backward_plan, greeting_plan, plan_duration)
from turtlebot4_gesture_control.gesture_logic import (  # noqa: E402
    NONE, WAVE, GestureFilter, GestureStabilizer, WaveDetector)


# ------------------------------------------------------------ gesture filter
def test_stabilizer_needs_consecutive_frames():
    s = GestureStabilizer(min_score=0.6, stable_frames=5, fast_frames=2)
    for _ in range(4):
        assert s.update('Victory', 0.9) == NONE
    assert s.update('Victory', 0.9) == 'Victory'


def test_stabilizer_rejects_flicker_and_low_score():
    s = GestureStabilizer(stable_frames=3)
    for lab in ['Victory', 'Thumb_Up', 'Victory', 'Thumb_Up', 'Victory']:
        assert s.update(lab, 0.9) == NONE
    for _ in range(10):
        assert s.update('Victory', 0.3) == NONE   # below min_score


def test_stop_gesture_is_faster():
    s = GestureStabilizer(stable_frames=6, fast_frames=2)
    s.update('Open_Palm', 0.9)
    assert s.update('Open_Palm', 0.9) == 'Open_Palm'


def test_wave_detector_counts_swings():
    w = WaveDetector(window_s=2.0, min_amplitude=0.06, min_swings=3)
    xs = [0.5, 0.55, 0.6, 0.55, 0.5, 0.45, 0.5, 0.56, 0.6]
    fired = [w.update(i * 0.05, x) for i, x in enumerate(xs)]
    assert any(fired)


def test_wave_detector_ignores_slow_drift_and_jitter():
    w = WaveDetector(window_s=1.5, min_amplitude=0.06, min_swings=3)
    # one-way drift across the frame
    assert not any(w.update(i * 0.05, 0.2 + i * 0.02) for i in range(30))
    w.reset()
    # small jitter
    assert not any(w.update(i * 0.05, 0.5 + (0.01 if i % 2 else -0.01)) for i in range(40))


def _wave_frames(t0, n=24, fps=20):
    out = []
    for i in range(n):
        t = t0 + i / fps
        out.append((t, 0.5 + 0.12 * math.sin(2 * math.pi * 2.0 * (t - t0))))
    return out


def test_filter_wave_is_latched_until_palm_disappears():
    f = GestureFilter(GestureStabilizer(stable_frames=4, fast_frames=2, none_frames=3),
                      WaveDetector(window_s=1.5, min_amplitude=0.06, min_swings=3))
    outs = [f.update(t, 'Open_Palm', 0.9, x) for t, x in _wave_frames(0.0)]
    assert outs[0] == NONE
    assert 'Open_Palm' in outs           # palm seen before wave recognised
    assert WAVE in outs
    first_wave = outs.index(WAVE)
    assert all(o == WAVE for o in outs[first_wave:])   # never back to Open_Palm
    # still palm afterwards: still latched
    assert f.update(10.0, 'Open_Palm', 0.9, 0.5) == WAVE
    # hand gone -> None, latch released
    outs = [f.update(11 + i * 0.05, 'None', 0.0) for i in range(5)]
    assert outs[-1] == NONE and not f.wave_latched


# --------------------------------------------------------------- plans
def test_plans_are_balanced():
    fb = forward_backward_plan(0.2, 0.4, 4, 0.4)
    disp = sum(s.linear * s.duration for s in fb)
    assert abs(disp) < 1e-9
    assert sum(1 for s in fb if s.linear > 0) == 4
    g = greeting_plan(1.2, 0.6, 2)
    assert abs(sum(s.angular * s.duration for s in g)) < 1e-9
    c = circle_plan(0.2, 0.5, 1)
    assert abs(c[0].angular * c[0].duration - 2 * math.pi) < 1e-9
    assert abs(c[0].linear / c[0].angular - 0.5) < 1e-9


def _ctrl(**kw):
    plans = {GREETING: greeting_plan(), FORWARD_BACKWARD: forward_backward_plan(),
             CIRCLE: circle_plan()}
    return BehaviorController(plans, **kw)


def _run(c, seconds, dt=0.05):
    cmds = []
    for _ in range(int(round(seconds / dt))):
        cmds.append(c.step(dt))
    return cmds


# ------------------------------------------------------------ state machine
def test_requires_confirmation():
    c = _ctrl()
    c.on_gesture('Pointing_Up', 0.0)
    assert c.state == State.SELECTED
    assert _run(c, 1.0)[-1] == (0.0, 0.0)      # nothing moves before Thumb_Up
    c.on_gesture('None', 0.5)
    c.on_gesture('Thumb_Up', 1.0)
    assert c.state == State.EXECUTING
    assert _run(c, 0.2)[-1][0] > 0


def test_thumb_up_without_selection_is_ignored():
    c = _ctrl()
    assert c.on_gesture('Thumb_Up', 0.0) is None
    assert c.state == State.IDLE


def test_edge_trigger_and_cooldown():
    c = _ctrl(cooldown=1.0)
    c.on_gesture('Victory', 0.0)
    assert c.on_gesture('Victory', 0.1) is None          # same label repeated
    c.on_gesture('None', 0.2)
    assert c.on_gesture('Victory', 0.4) is None          # flicker within cooldown
    c.on_gesture('None', 1.5)
    assert c.on_gesture('Victory', 1.6) == 'select'


def test_stop_pauses_and_thumb_up_resumes():
    c = _ctrl()
    c.on_gesture('Victory', 0.0)
    c.on_gesture('Thumb_Up', 2.0)
    _run(c, 3.0)
    p = c.progress
    c.on_gesture('Open_Palm', 5.0)
    assert c.state == State.PAUSED
    assert c.step(0.05) == (0.0, 0.0)
    _run(c, 2.0)
    assert c.progress == p                                # frozen while paused
    c.on_gesture('Thumb_Up', 7.0)
    assert c.state == State.EXECUTING
    _run(c, 0.5)
    assert c.progress > p


def test_fist_cancels_everything():
    c = _ctrl()
    c.on_gesture('Pointing_Up', 0.0)
    c.on_gesture('Thumb_Up', 2.0)
    _run(c, 1.0)
    c.on_gesture('Closed_Fist', 3.0)
    assert c.state == State.IDLE and c.selected is None
    c.on_gesture('Thumb_Up', 5.0)
    assert c.state == State.IDLE


def test_select_ignored_while_executing():
    c = _ctrl()
    c.on_gesture('Pointing_Up', 0.0)
    c.on_gesture('Thumb_Up', 2.0)
    c.on_gesture('Victory', 4.0)
    assert c.state == State.EXECUTING and c.selected == FORWARD_BACKWARD


def test_wave_greets_and_finishes():
    c = _ctrl()
    c.on_gesture('Open_Palm', 0.0)
    c.on_gesture('Wave', 0.5)
    assert c.state == State.EXECUTING and c.selected == GREETING
    _run(c, plan_duration(greeting_plan()) + 0.2)
    assert c.state == State.IDLE


def test_wave_can_require_confirm():
    c = _ctrl(wave_requires_confirm=True)
    c.on_gesture('Wave', 0.0)
    assert c.state == State.SELECTED and c.selected == GREETING


def test_signal_lost_stops_robot():
    c = _ctrl()
    c.on_gesture('Victory', 0.0)
    c.on_gesture('Thumb_Up', 2.0)
    _run(c, 1.0)
    assert c.on_signal_lost()
    assert c.state == State.IDLE
    assert c.step(0.05) == (0.0, 0.0)


def test_behaviour_runs_to_completion():
    c = _ctrl()
    c.on_gesture('Pointing_Up', 0.0)
    c.on_gesture('Thumb_Up', 2.0)
    total = plan_duration(forward_backward_plan())
    cmds = _run(c, total + 0.5)
    assert c.state == State.IDLE
    disp = sum(v * 0.05 for v, _ in cmds)
    assert abs(disp) < 0.03
