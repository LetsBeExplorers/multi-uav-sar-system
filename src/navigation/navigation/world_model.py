// WORLD MODEL - world_model.py
// Owns occupancy grid, static and dynamic obstacle management
// Serves grid to A* on request via service

PARAMETERS:
  uav_id
  num_uavs
  grid_width         // cells
  grid_height        // cells
  resolution         // meters per cell
  origin_x           // world coords of grid origin
  origin_y
  inflation_radius   // cells to inflate around obstacles
  static_obstacles   // list of (x,y) in world coords, loaded from params

SUBSCRIPTIONS:
  /{other_uav_id}/state/pose  → on_dynamic_obstacle_update()
  /{uav_id}/state/pose        → on_own_pose_update()
  // one subscription per other UAV
  // e.g. x1 subscribes to /x2/state/pose and /x3/state/pose

SERVICES:
  /{uav_id}/world_model/get_grid  → get_occupancy_grid()

STATE:
  grid = 2D array of 0s    // 0 = free, 1 = occupied
  static_grid = 2D array   // separate record of static obstacles only
  dynamic_obstacles = {}   // world coords
  unknown_obstacles = {}   // FUTURE: (x,y) → timestamp
  own_pose = None          // current position of this UAV

INITIALIZATION:
  initialize grid as all free
  load static obstacles from params
  for each static obstacle:
    mark_occupied(x, y)
    static_grid[gx][gy] = 1   // record as static
  setup subscriptions for all other UAV odom topics
  // STUB: when real sensor pipeline is ready,
  // replace static_obstacles param with sensor callback
  // interface stays the same - just calls mark_occupied()

// ==============================
// Grid Management
// ==============================

world_to_grid(world_x, world_y):
  grid_x = int((world_x - origin_x) / resolution)
  grid_y = int((world_y - origin_y) / resolution)
  return (grid_x, grid_y)

mark_occupied(world_x, world_y):
  gx, gy = world_to_grid(world_x, world_y)
  
  // mark cell and inflated neighborhood
  for each cell in neighborhood(gx, gy, inflation_radius):
    if in bounds:
      grid[cell] = 1

mark_free(world_x, world_y):
  gx, gy = world_to_grid(world_x, world_y)
  if in bounds:
    grid[gx][gy] = 0
  // note: don't clear inflated cells, only clear center
  // inflation is conservative - better to be safe

on_own_pose_update(msg):
  own_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

// ==============================
// Dynamic Obstacles
// ==============================

on_dynamic_obstacle_update(uav_id, Odometry msg):
  x = msg.pose.pose.position.x
  y = msg.pose.pose.position.y

  // clear previous position of this UAV
  if uav_id in dynamic_obstacles:
    old_x, old_y = dynamic_obstacles[uav_id]
    clear_dynamic_obstacle(old_x, old_y)

  // mark new position
  mark_occupied(x, y)
  dynamic_obstacles[uav_id] = (x, y)

clear_dynamic_obstacle(world_x, world_y):
  gx, gy = world_to_grid(world_x, world_y)
  
  for each cell in neighborhood(gx, gy, inflation_radius):
    if in bounds and static_grid[cell] == 0:  // only clear if not static
      grid[cell] = 0

on_unknown_obstacle_detected(x, y):
  mark_occupied(x, y)
  // FUTURE: add to unknown_obstacles with timestamp
  // FUTURE: clear if not seen for N seconds

// ==============================
// Service Handler
// ==============================

get_occupancy_grid(request):
  // called by A* when it needs to plan
  response = GetOccupancyGrid()
  response.grid = grid
  response.width = grid_width
  response.height = grid_height
  response.resolution = resolution
  response.origin_x = origin_x
  response.origin_y = origin_y
  return response