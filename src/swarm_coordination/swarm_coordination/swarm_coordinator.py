// SWARM COORDINATOR - swarm_coordinator.py
// Spatial planner - owns region assignment, waypoint generation, coverage tracking
// Takes direction from uav_state_manager via /uav/state

PARAMETERS:
  uav_id        // e.g. "x1"
  num_uavs      // total agents
  area_bounds   // [xmin, xmax, ymin, ymax]
  rows          // lawnmower row density
  threshold     // coverage completeness threshold e.g. 0.95

SUBSCRIPTIONS:
  /uav/state                      → on_fsm_state_change()
  /{uav_id}/nav/reached_waypoint  → on_waypoint_reached()
  /{uav_id}/nav/waypoint_count    → on_waypoint_count()
  /mission/coverage               → on_coverage_update()   // ← add this

PUBLICATIONS:
  /{uav_id}/nav/waypoints   → PoseArray (waypoints to navigation)
  /{uav_id}/fsm/event       → FSMEvent.msg (coverage events to FSM)
  /mission/status           → String (progress updates to dashboard)

STATE:
  current_mode = None       // mirrors FSM state, set by on_fsm_state_change
  is_paused = False         // true when FSM is in VERIFYING
  coverage_map = {}         // uav_id → coverage ratio, updated from /mission/coverage
  visited_waypoints = 0
  num_waypoints = 0
  x_start = 0.0
  x_end = 0.0
  start_time = None
  run_id = None

INITIALIZATION:
  compute uav_index from uav_id and num_uavs
  compute assigned region:
    slice_width = (xmax - xmin) / num_uavs
    x_start = xmin + uav_index * slice_width
    x_end = xmin + (uav_index + 1) * slice_width
  init logging

// ==============================
// FSM Reaction
// ==============================

on_fsm_state_change(UAVState msg):
  current_mode = msg.state

  if current_mode == VERIFYING:
    is_paused = True
    return

  is_paused = False

  if current_mode == SEARCHING:
    reset coverage metrics
    start_time = now
    publish_search_waypoints()

  elif current_mode == REFINING:
    publish_refinement_waypoints()

  elif current_mode == ASSISTING:
    publish_assistive_waypoints()

  elif current_mode == RETURNING or IDLE:
    // stop generating waypoints, nothing to do
    return

// ==============================
// Waypoint Generation
// ==============================

publish_search_waypoints():
  // standard lawnmower over assigned x slice, full y range
  poses = generate_lawnmower_waypoints(
    x_start, x_end, ymin, ymax, rows
  )
  publish → /{uav_id}/nav/waypoints

publish_refinement_waypoints():
  // higher density pass or offset sweep over same region
  // only if coverage < threshold
  poses = generate_lawnmower_waypoints(
    x_start, x_end, ymin, ymax, rows * 2  // denser
  )
  publish → /{uav_id}/nav/waypoints

publish_assistive_waypoints():
  // find least covered region excluding self
  target_id = uav_id with lowest coverage_map value, excluding self
  target_index = parse index from target_id
  
  slice_width = (xmax - xmin) / num_uavs
  target_x_start = xmin + target_index * slice_width
  target_x_end = xmin + (target_index + 1) * slice_width
  
  poses = generate_lawnmower_waypoints(
    target_x_start, target_x_end, ymin, ymax, rows
  )
  publish → /{uav_id}/nav/waypoints

// ==============================
// Coverage Tracking
// ==============================

on_waypoint_reached():
  if is_paused:
    return   // don't update coverage while verifying

  visited_waypoints += 1
  coverage = visited_waypoints / num_waypoints

  // publish progress to dashboard every 10 waypoints
  if visited_waypoints % 10 == 0:
    publish → /mission/status
    message: "[{uav_id}] PROGRESS: {visited_waypoints}/{num_waypoints}"

  // check if we should tell FSM to transition
  check_coverage_events(coverage)

on_waypoint_count(msg):
  num_waypoints += msg.data

check_coverage_events(coverage):
  if current_mode == SEARCHING and all waypoints complete:
    publish → /{uav_id}/fsm/event
    event: REGION_COMPLETE, value: coverage

  elif current_mode == REFINING and all waypoints complete:
    publish → /{uav_id}/fsm/event
    event: REFINEMENT_COMPLETE, value: coverage

  elif current_mode == ASSISTING and all waypoints complete:
    if all(v >= threshold for v in coverage_map.values()):
      publish → /{uav_id}/fsm/event
      event: ALL_DRONES_DONE
    else:
      publish → /{uav_id}/fsm/event
      event: ASSIST_COMPLETE

on_coverage_update(MissionCoverage msg):
  for i, uav_id in enumerate(msg.uav_ids):
    coverage_map[uav_id] = msg.coverage_ratios[i]

// ==============================
// Logging
// ==============================

log_metrics():
  // same as current implementation
  // stop logging at 100% coverage