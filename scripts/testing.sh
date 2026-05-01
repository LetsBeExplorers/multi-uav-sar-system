#!/bin/bash
set -e

# ===== CONFIG =====

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

OUTPUT_FILE="results.csv"   # <-- CHANGE if needed
RUN_ID=0

# ===== MAIN LOOP =====

for WORLD in "${WORLDS[@]}"; do
  echo ""
  echo "========================================"
  echo "Running: $WORLD"
  echo "========================================"

  # ----- Extract difficulty -----
  if [[ $WORLD == *easy* ]]; then
    DIFF="easy"
  elif [[ $WORLD == *medium* ]]; then
    DIFF="medium"
  elif [[ $WORLD == *hard* ]]; then
    DIFF="hard"
  else
    DIFF="unknown"
  fi

  # ----- Start simulator -----
  echo "Starting simulator..."
  ./run_sim.sh "$WORLD" &
  SIM_PID=$!

  # ----- Wait for sim to come up -----
  sleep 3

  echo "Waiting for Gazebo control service..."
  until ros2 service list | grep -q "/world/default/control"; do
    sleep 1
  done

  # ----- Unpause sim -----
  echo "Unpausing simulator..."
  ros2 service call /world/default/control gz_msgs/srv/WorldControl "{pause: false}" > /dev/null

  sleep 1

  # ----- Run system (this blocks until mission ends) -----
  echo "Starting system..."
  ./run.sh

  echo "Mission complete."

  # ----- Save results -----
  if [ -f "$OUTPUT_FILE" ]; then
    TS=$(date +%H%M%S)
    NEW_NAME="results_${DIFF}_run${RUN_ID}_${TS}.csv"
    mv "$OUTPUT_FILE" "$NEW_NAME"
    echo "Saved: $NEW_NAME"
  else
    echo "WARNING: $OUTPUT_FILE not found"
  fi

  # ----- Kill simulator cleanly -----
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