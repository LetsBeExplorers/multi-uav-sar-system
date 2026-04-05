// MISSION MANAGER - mission_manager.py
// Operator interface and mission aggregator
// Owns start/stop commands, dashboard, and coverage aggregation

PARAMETERS:
  num_uavs
  threshold    // coverage completeness threshold, shared with coordinator

SUBSCRIPTIONS:
  /uav/state              → on_uav_state_change()
  /mission/status         → on_status() // progress strings for dashboard

PUBLICATIONS:
  /mission/start          → Empty
  /mission/stop           → Empty
  /mission/coverage       → MissionCoverage.msg

STATE:
  mission_state = IDLE                             // overall mission state
  uav_states = {id: "IDLE" for id in uav_ids}      // uav_id → current FSM state
  uav_coverage = {id: 0.0 for id in uav_ids}       // uav_id → coverage ratio
  uav_ids = [f"x{i+1}" for i in range(num_uavs)]   // ordered list of uav ids

// ==============================
// State Tracking
// ==============================

on_uav_state_change(UAVState msg):
  uav_states[msg.uav_id] = msg.state
  
  if all uav_states == IDLE:
    mission_state = COMPLETE

  refresh_dashboard()

on_status(String msg):
  if "PROGRESS" in msg:
    uav_id = parse_uav_id(msg)
    visited, total = parse_progress(msg)
    if total > 0:
      uav_coverage[uav_id] = visited / total
      publish_coverage()  // publish immediately on update

  refresh_dashboard()

// ==============================
// Coverage Aggregation
// ==============================

publish_coverage():
  msg = MissionCoverage()
  msg.uav_ids = uav_ids
  msg.coverage_ratios = [uav_coverage.get(id, 0.0) for id in uav_ids]
  msg.all_complete = all(r >= threshold for r in msg.coverage_ratios)
  msg.timestamp = now
  publish → /mission/coverage

// ==============================
// Dashboard
// ==============================

refresh_dashboard():
  clear screen
  print mission_state
  for each uav_id:
    print uav_id, uav_states[uav_id], uav_coverage[uav_id]
  print available commands

// ==============================
// Operator Interface
// ==============================

send_start():
  if mission_state != IDLE:
    return
  mission_state = RUNNING
  uav_states = {id: "IDLE" for id in uav_ids}
  uav_coverage = {id: 0.0 for id in uav_ids}
  publish → /mission/start

send_stop():
  if mission_state == IDLE:
    return
  mission_state = STOPPED
  publish → /mission/stop

// FUTURE: send_mission_update() 
//   would support mid-flight replanning and region reassignment
//   requires MISSION_UPDATE event in FSM and coordinator reset logic