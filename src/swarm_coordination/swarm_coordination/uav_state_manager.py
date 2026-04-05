// FSM NODE - uav_state_manager.py
SUBSCRIPTIONS:
  /mission/start        → MISSION_START event
  /mission/stop         → MISSION_STOP event
  /{uav_id}/fsm/event   → all other FSMEvent.msg events

PUBLICATIONS:
  /uav/state            → UAVState.msg on every transition
  /gcs/target_detected  → target location on TARGET_LOCK entry

// UAVState.msg
string uav_id
string state
string previous_state
float64 timestamp

// FSMEvent.msg
string uav_id
string event
float64 timestamp
float64 value      // optional payload e.g. coverage ratio

STATE MACHINE:
  current_state = IDLE
  previous_state = None

  transition(new_state):
    previous_state = current_state
    current_state = new_state
    publish_state_change()
    if new_state == TARGET_LOCK:
      publish → /gcs/target_detected

  publish_state_change():
    publish → /uav/state
    message: UAVState.msg

  handle_event(event):

    // TIER 1 - always wins
    if event == CRITICAL_FAILURE:
      transition(EMERGENCY_STOP)
      return

    // TIER 2 - constraint overrides
    if event in [LOW_BATTERY, COLLISION_RISK]:
      transition(RETURNING)
      return
    if event in [PATH_FAILED, COMMS_LOSS]:
      transition(RECOVERY)  // stub - exits to RETURNING on failure, previous_state on success
      return

    // TIER 3 - mission commands
    if event == MISSION_STOP:
      transition(RETURNING)
      return
    if event == MISSION_START:
      if current_state == IDLE:
        transition(SEARCHING)
      return

    // TIER 4 - coverage and detection
    if event == REGION_COMPLETE:
      if event.value < threshold:
        transition(REFINING)
      else:
        transition(ASSISTING)
      return
    if event == REFINEMENT_COMPLETE:
      transition(ASSISTING)
      return
    if event == ASSIST_COMPLETE or ALL_DRONES_DONE:
      transition(RETURNING)
      return
    if event == DETECTION_EVENT:
      transition(VERIFYING)
      return
    if event == CONFIRMED_TARGET:
      transition(TARGET_LOCK)
      return
    if event == FALSE_POSITIVE:
      transition(previous_state)
      return
    if event == TARGET_LOST:
      transition(RETURNING)
      return

// SWARM COORDINATOR - separate node, reactive
SUBSCRIPTIONS:
  /uav/state  → pause waypoint generation if VERIFYING, resume otherwise