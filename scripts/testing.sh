#!/bin/bash
set -e

# ===== PATH SETUP =====
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
RESULTS_DIR="$WORKSPACE_DIR/results"

# ===== WORLDS =====
WORLDS=(
  quadcopter_easy_world1.sdf
  quadcopter_easy_world2.sdf
  quadcopter_easy_world3.sdf
  quadcopter_medium_world1.sdf
  quadcopter_medium_world2.sdf
  quadcopter_medium_world3.sdf
  quadcopter_hard_world1.sdf
  quadcopter_hard_world2.sdf
  quadcopter_hard_world3.sdf
)

RUN_ID=0

cd "$SCRIPT_DIR"

for WORLD in "${WORLDS[@]}"; do
  echo ""
  echo "========================================"
  echo "Running: $WORLD"
  echo "========================================"

  # ----- difficulty -----
  if [[ $WORLD == *easy* ]]; then
    DIFF="easy"
  elif [[ $WORLD == *medium* ]]; then
    DIFF="medium"
  elif [[ $WORLD == *hard* ]]; then
    DIFF="hard"
  else
    DIFF="unknown"
  fi

  # ----- snapshot results BEFORE -----
  BEFORE_FILES=$(ls "$RESULTS_DIR" 2>/dev/null || true)

  # ----- start simulator -----
  echo "Starting simulator..."
  ./run_sim.sh "$WORLD" &
  sleep 3

  # wait for gazebo service
  echo "Waiting for Gazebo..."
  until ros2 service list | grep -q "/world/default/control"; do
    sleep 1
  done

  # unpause
  echo "Unpausing simulator..."
  ros2 service call /world/default/control gz_msgs/srv/WorldControl "{pause: false}" > /dev/null

  sleep 1

  # ----- run system -----
  echo "Starting system..."
  ./run.sh

  echo "Mission complete."

  # ----- snapshot AFTER -----
  AFTER_FILES=$(ls "$RESULTS_DIR" 2>/dev/null || true)

  NEW_FILES=$(comm -13 <(echo "$BEFORE_FILES" | sort) <(echo "$AFTER_FILES" | sort))

  TS=$(date +%H%M%S)

  # ----- rename new result files -----
  for FILE in $NEW_FILES; do
    FULL_PATH="$RESULTS_DIR/$FILE"

    if [ -f "$FULL_PATH" ]; then
      BASE="${FILE%.*}"
      EXT="${FILE##*.}"

      NEW_NAME="${BASE}_${DIFF}_run${RUN_ID}_${TS}.${EXT}"
      mv "$FULL_PATH" "$RESULTS_DIR/$NEW_NAME"

      echo "Saved: $NEW_NAME"
    fi
  done

  # ----- kill simulator -----
  echo "Stopping simulator..."
  pkill -9 -f "gz sim" || true
  pkill -9 -f gazebo || true
  pkill -9 -f ros_gz_bridge || true

  sleep 3
  ((RUN_ID++))

done

echo ""
echo "========================================"
echo "ALL RUNS COMPLETE ✅"
echo "========================================"