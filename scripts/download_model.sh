#!/usr/bin/env bash
# Download the MediaPipe Gesture Recognizer model bundle into ../models/
set -euo pipefail
DIR="$(cd "$(dirname "$0")/.." && pwd)/models"
URL="https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/latest/gesture_recognizer.task"
mkdir -p "$DIR"
if command -v wget >/dev/null 2>&1; then
  wget -q --show-progress -O "$DIR/gesture_recognizer.task" "$URL"
else
  curl -L --fail -o "$DIR/gesture_recognizer.task" "$URL"
fi
echo "Saved: $DIR/gesture_recognizer.task ($(du -h "$DIR/gesture_recognizer.task" | cut -f1))"
echo "Now rebuild:  cd ~/tb4_gesture_ws && colcon build --symlink-install"
