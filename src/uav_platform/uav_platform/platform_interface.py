// PLATFORM INTERFACE - platform_interface.py
// Safety layer between autonomy stack and hardware drivers
// Responsibilities:
//   - Clamp velocity commands to safe limits
//   - Monitor hardware health and report issues to FSM
//   - Single point of safety enforcement regardless of platform
// NOT responsible for:
//   - State translation (that's the driver's job)
//   - Platform-specific communication (that's the driver's job)

// DriverHealth.msg
string status      // OK, COMMS_LOSS, LOW_BATTERY etc.
float64 battery    // percentage
float64 timestamp

PARAMETERS:
  uav_name
  max_linear_speed
  max_vertical_speed
  takeoff_velocity_threshold
  landing_velocity_threshold
  low_battery_threshold    // percentage e.g. 20%

SUBSCRIPTIONS:
  /{uav_name}/platform/cmd_vel  → process_command()
  /{uav_name}/driver/health     → on_health_update()  // driver reports hardware issues

PUBLICATIONS:
  /{uav_name}/driver/cmd_vel    → Twist (safe clamped commands)
  /{uav_name}/fsm/event         → FSMEvent.msg (LOW_BATTERY, COMMS_LOSS)

STATE:
  flight_state = GROUNDED   // GROUNDED or AIRBORNE
  // hardware: read from Tello SDK
  // sim: not available, stub only

// ==============================
// Command Processing
// ==============================

process_command(msg):
  safe_msg = Twist()

  // clamp horizontal
  safe_msg.linear.x = clamp(msg.linear.x, -max_linear, max_linear)
  safe_msg.linear.y = clamp(msg.linear.y, -max_linear, max_linear)

  // clamp vertical
  safe_msg.linear.z = clamp(msg.linear.z, -max_vertical, max_vertical)

  // pass through yaw
  safe_msg.angular.z = msg.angular.z

  update_flight_state(safe_msg.linear.z)
  publish → /{uav_name}/driver/cmd_vel

// ==============================
// Flight State
// ==============================

update_flight_state(vz):
  if vz > takeoff_threshold and flight_state != AIRBORNE:
    flight_state = AIRBORNE

  elif vz < landing_threshold and flight_state != GROUNDED:
    flight_state = GROUNDED

on_health_update(msg):
  if msg.status == COMMS_LOSS:
    publish → /{uav_name}/fsm/event
    event: COMMS_LOSS

  if msg.battery < low_battery_threshold:
    publish → /{uav_name}/fsm/event
    event: LOW_BATTERY