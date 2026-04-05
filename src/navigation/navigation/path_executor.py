// PATH EXECUTOR - path_executor.py
// Executes A* planned paths, tracks execution metrics
// Publishes HOME_REACHED to FSM when home position reached

PARAMETERS:
  uav_id
  speed             // movement speed e.g. 2.0
  waypoint_threshold // distance to consider waypoint reached e.g. 0.2
  lookahead         // cells to look ahead on path e.g. 5

SUBSCRIPTIONS:
  /{uav_id}/nav/planned_path  → on_path_received()
  /{uav_id}/state/pose        → on_pose_update()
  /mission/stop               → on_stop()

PUBLICATIONS:
  /{uav_id}/platform/cmd_vel          → Twist
  /{uav_id}/nav/reached_coverage_waypoint  → Empty
  /{uav_id}/fsm/event                 → FSMEvent.msg (HOME_REACHED)

STATE:
  current_path = []
  current_index = 0
  uav_x = 0.0
  uav_y = 0.0
  home_x = None
  home_y = None
  is_returning = False
  path_cells_total = 0
  path_cells_traversed = 0

INITIALIZATION:
  init logging
  create timer at 0.1s → move_step()

// ==============================
// Pose Tracking
// ==============================

on_pose_update(msg):
  uav_x = msg.pose.pose.position.x
  uav_y = msg.pose.pose.position.y

  // capture home position once
  if home_x is None:
    home_x = uav_x
    home_y = uav_y

// ==============================
// Path Reception
// ==============================

on_path_received(msg):
  current_path = [pose.pose for pose in msg.poses]
  current_index = 0
  path_cells_total += len(current_path)

// ==============================
// Execution
// ==============================

move_step():
  if current_path is empty:
    stop()
    return

  target_index = min(current_index + lookahead, len(current_path) - 1)
  target = current_path[target_index]

  dx = target.x - uav_x
  dy = target.y - uav_y
  dist = sqrt(dx² + dy²)

  if dist > 0:
    publish → /{uav_id}/platform/cmd_vel
    cmd: dx/dist * speed, dy/dist * speed

  if dist < waypoint_threshold:
    current_index += 1
    path_cells_traversed += 1
    publish → /{uav_id}/nav/reached_coverage_waypoint

    if current_index >= len(current_path):
      stop()

      if is_returning:
        // reached home
        is_returning = False
        publish → /{uav_id}/fsm/event
        event: HOME_REACHED
        log_metrics()
      
      current_path = []
      current_index = 0

stop():
  publish → /{uav_id}/platform/cmd_vel
  cmd: zero twist

// ==============================
// Stop Handler
// ==============================

on_stop():
  current_path = []
  current_index = 0
  stop()

// ==============================
// Logging
// ==============================

log_metrics():
  completion_ratio = path_cells_traversed / path_cells_total if path_cells_total > 0 else 0.0
  write to CSV:
    uav_id, timestamp, path_cells_total, path_cells_traversed, completion_ratio