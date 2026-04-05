// A* NAVIGATION NODE - astar_navigation_node.py
// Path planning - queries world model for grid, plans collision-free paths
// Replans when path is invalidated

PARAMETERS:
  uav_id
  replan_check_rate   // how often to check path validity e.g. 0.5 hz

SUBSCRIPTIONS:
  /{uav_id}/nav/waypoints   → on_waypoint_received()   // single Pose msg, from coordinator
  /{uav_id}/state/pose      → on_pose_update()

SERVICES (client):
  /{uav_id}/world_model/get_grid  // call when planning or checking validity

PUBLICATIONS:
  /{uav_id}/nav/planned_path  → Path msg
  /{uav_id}/fsm/event         → FSMEvent.msg (PATH_FAILED)
  /mission/status             → String (metrics)

STATE:
  current_pose = None
  current_goal = None
  current_path = None
  initial_plan_count = 0
  replan_count = 0
  path_failed_count = 0

// ==============================
// Pose Tracking
// ==============================

on_pose_update(msg):
  current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

// ==============================
// Waypoint Reception
// ==============================

on_waypoint_received(msg):
  if current_pose is None:
    return

  current_goal = (msg.pose.position.x, msg.pose.position.y)
  plan()

// ==============================
// Planning
// ==============================

plan():
  grid = call /{uav_id}/world_model/get_grid service
  
  if grid is None:
    return  // service unavailable, wait

  start = world_to_grid(current_pose)
  goal = world_to_grid(current_goal)
  path = astar(start, goal, grid)

  if path is None:
    path_failed_count += 1
    publish → /{uav_id}/fsm/event
    event: PATH_FAILED
    log_metrics()
    return

  // track whether this is initial plan or replan
  if current_path is None:
    initial_plan_count += 1   // first time planning to this goal
  // replan_count already incremented in check_path_validity before calling plan()

  current_path = path
  publish → /{uav_id}/nav/planned_path
  log_metrics()

// ==============================
// Path Validity Check
// ==============================

// runs on timer at replan_check_rate
check_path_validity():
  if current_path is None:
    return
  if current_goal is None:
    return

  grid = call /{uav_id}/world_model/get_grid service

  for each cell in current_path:
    if grid[cell] == occupied:
      current_path = None
      replan_count += 1
      plan()
      return

// ==============================
// Core Algorithm
// ==============================

astar(start, goal, grid):
  // same as current implementation
  // just takes grid as parameter instead of using self.grid

// ==============================
// Helpers
// ==============================

world_to_grid(world_x, world_y):
  // same conversion as world model
  // maybe worth sharing as a utility function later

build_path_msg(cells):
  // same as current implementation

// ==============================
// Metrics Logging
// ==============================

log_metrics():
  publish → /mission/status
  message: "[{uav_id}] REPLANS: {replan_count} FAILURES: {path_failed_count}"