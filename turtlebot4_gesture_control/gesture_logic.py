"""Post-processing of MediaPipe gesture results (pure Python, no ROS deps).

Three pieces, chained in GestureFilter:

1. GestureStabilizer  - a raw label only becomes "stable" after it has been
   seen in N consecutive frames with score >= min_score. Safety gestures
   (Open_Palm, Closed_Fist) need fewer frames so the robot stops faster.
2. WaveDetector       - MediaPipe has no "Wave" class, so a wave is detected
   as an Open_Palm whose palm centre swings left/right several times
   (zig-zag with a minimum amplitude) inside a short time window.
3. Wave latch         - once a wave is detected the published label stays
   "Wave" until the hand stops showing an open palm. This prevents the
   still-open palm right after the wave from being read as a STOP command
   that would cancel the greeting the wave just started.
"""

from collections import deque

NONE = 'None'
WAVE = 'Wave'
OPEN_PALM = 'Open_Palm'
CLOSED_FIST = 'Closed_Fist'

# Canned classes of the MediaPipe Gesture Recognizer model + our Wave.
CANNED_GESTURES = (
    'None', 'Closed_Fist', 'Open_Palm', 'Pointing_Up',
    'Thumb_Down', 'Thumb_Up', 'Victory', 'ILoveYou',
)


class GestureStabilizer:
    """Consecutive-frame voting filter."""

    def __init__(self, min_score=0.6, stable_frames=6, fast_frames=3,
                 fast_gestures=(OPEN_PALM, CLOSED_FIST), none_frames=8):
        self.min_score = float(min_score)
        self.stable_frames = max(1, int(stable_frames))
        self.fast_frames = max(1, int(fast_frames))
        self.none_frames = max(1, int(none_frames))
        self.fast_gestures = set(fast_gestures)
        self.stable = NONE
        self._candidate = NONE
        self._count = 0

    def required_frames(self, label):
        if label == NONE:
            return self.none_frames
        if label in self.fast_gestures:
            return self.fast_frames
        return self.stable_frames

    def update(self, label, score):
        if not label or label == NONE or score < self.min_score:
            label = NONE
        if label == self._candidate:
            self._count += 1
        else:
            self._candidate = label
            self._count = 1
        if self._count >= self.required_frames(label):
            self.stable = label
        return self.stable

    @property
    def candidate(self):
        return self._candidate

    @property
    def progress(self):
        """0..1 progress of the current candidate towards becoming stable."""
        need = self.required_frames(self._candidate)
        return min(1.0, self._count / need)

    def reset(self):
        self.stable = NONE
        self._candidate = NONE
        self._count = 0


class WaveDetector:
    """Detects left-right oscillation of the palm centre (normalised x)."""

    def __init__(self, window_s=1.5, min_amplitude=0.06, min_swings=3):
        self.window_s = float(window_s)
        self.min_amplitude = float(min_amplitude)
        self.min_swings = max(2, int(min_swings))
        self._hist = deque()
        self.swings = 0

    def reset(self):
        self._hist.clear()
        self.swings = 0

    def _count_swings(self):
        """Count strokes of at least min_amplitude in alternating direction."""
        xs = [x for _, x in self._hist]
        if len(xs) < 2:
            return 0
        a = self.min_amplitude
        direction = 0
        lo = hi = ref = xs[0]
        swings = 0
        for x in xs[1:]:
            if direction == 0:
                lo, hi = min(lo, x), max(hi, x)
                if x - lo >= a:
                    direction, ref, swings = 1, x, 1
                elif hi - x >= a:
                    direction, ref, swings = -1, x, 1
            elif direction == 1:
                if x > ref:
                    ref = x
                elif ref - x >= a:
                    direction, ref = -1, x
                    swings += 1
            else:
                if x < ref:
                    ref = x
                elif x - ref >= a:
                    direction, ref = 1, x
                    swings += 1
        return swings

    def update(self, t, x):
        """Add a sample; returns True once when a wave is recognised."""
        self._hist.append((t, x))
        while self._hist and t - self._hist[0][0] > self.window_s:
            self._hist.popleft()
        self.swings = self._count_swings()
        if self.swings >= self.min_swings:
            self.reset()
            return True
        return False


class GestureFilter:
    """Raw MediaPipe output -> label to publish on /detected_gesture."""

    def __init__(self, stabilizer, wave_detector=None):
        self.stabilizer = stabilizer
        self.wave = wave_detector
        self.wave_latched = False

    def update(self, t, raw_label, raw_score, palm_x=None):
        stable = self.stabilizer.update(raw_label, raw_score)

        if self.wave is not None:
            palm_ok = (raw_label == OPEN_PALM
                       and raw_score >= self.stabilizer.min_score
                       and palm_x is not None)
            if palm_ok:
                if self.wave.update(t, palm_x) and stable == OPEN_PALM:
                    self.wave_latched = True
            elif stable != OPEN_PALM:
                # Hand is no longer an open palm: forget the trajectory.
                self.wave.reset()

        if self.wave_latched and stable != OPEN_PALM:
            self.wave_latched = False

        return WAVE if self.wave_latched else stable

    @property
    def wave_swings(self):
        return self.wave.swings if self.wave is not None else 0
