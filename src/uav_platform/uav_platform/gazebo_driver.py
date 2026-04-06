// GAZEBO DRIVER - gazebo_driver.py
// Sim-specific driver
// Translates between Gazebo and ROS system

PARAMETERS:
  uav_name

SUBSCRIPTIONS:
  /{uav_name}/driver/cmd_vel      → forward_command()
  /model/{uav_name}/odometry      → forward_state()

PUBLICATIONS:
  /model/{uav_name}/cmd_vel       → Twist
  /{uav_name}/state/pose          → Odometry
  /{uav_name}/driver/health       → DriverHealth.msg

// ==============================
// Command Passthrough
// ==============================
forward_command(msg):
  publish → /model/{uav_name}/cmd_vel

// ==============================
// State Reporting
// ==============================
forward_state(msg):
  publish → /{uav_name}/state/pose
  publish_health()

// ==============================
// Health Reporting
// ==============================
// called from forward_state since they're naturally coupled

publish_health():
  health.status = "OK"
  health.battery = 100.0  // stubbed in sim
  health.timestamp = now
  publish → /{uav_name}/driver/health