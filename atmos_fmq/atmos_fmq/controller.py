import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# from atmos_fmq_msgs.msg import RobotState
from geometry_msgs.msg import PoseStamped

class MsgHandlerController(Node):
    def __init__(self):
        super().__init__('msg_handler_robot')

        fmq_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Get namespace
        self.namespace = self.declare_parameter('namespace', '').value
        self.namespace_prefix = f'/{self.namespace}' if self.namespace else ''

        self.control_pub = self.create_publisher(PoseStamped,  f'{self.namespace_prefix}/fmq/control', fmq_qos_profile)
        self.state_pub = self.create_publisher(PoseStamped,  f'{self.namespace_prefix}/robot_pose', 10)

        self.setpoint_sub = self.create_subscription(PoseStamped,  f'{self.namespace_prefix}/px4_mpc/setpoint_pose', self.setpoint_callback, 10)
        self.state_sub = self.create_subscription(PoseStamped,  f'{self.namespace_prefix}/fmq/state', self.state_callback, fmq_qos_profile)

    def setpoint_callback(self, msg: PoseStamped):
        self.control_pub.publish(msg)

    def state_callback(self, msg: PoseStamped):
        self.state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MsgHandlerController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
