import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class GazeboDriver(Node):

    def __init__(self):
        super().__init__('gazebo_driver')

        # Parameters
        self.declare_parameter('uav_name', 'x1')
        uav_name = self.get_parameter('uav_name').value

        # Topics
        self.cmd_in_topic = f'/{uav_name}/driver/cmd_vel'
        self.gz_cmd_topic = f'/model/{uav_name}/cmd_vel'
        self.gz_odom_topic = f'/model/{uav_name}/odometry'
        self.state_topic = f'/{uav_name}/state/odom'

        # Publishers: Gazebo and Platform
        self.cmd_pub = self.create_publisher(Twist, self.gz_cmd_topic, 10)
        self.state_pub = self.create_publisher(Odometry, self.state_topic, 10)

        # Subscribers from Gazebo and Platform Interface
        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_in_topic,
            self.forward_command,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.gz_odom_topic,
            self.forward_state,
            10
        )

        # Debug message
        self.get_logger().debug(f"GazeboDriver ready for UAV: {uav_name}")

    def forward_command(self, msg):
        self.cmd_pub.publish(msg)

    def forward_state(self, msg):
        self.state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GazeboDriver()
    rclpy.spin(node)
    rclpy.shutdown()