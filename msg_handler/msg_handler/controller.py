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
        


        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.control_pub = self.create_publisher(PoseStamped, 'fmq/control', qos_profile)
        self.state_pub = self.create_publisher(PoseStamped, 'robot_pose', qos_profile)

        self.setpoint_sub = self.create_subscription(PoseStamped, '/px4_mpc/setpoint_pose', self.setpoint_callback, qos_profile)
        self.state_sub = self.create_subscription(PoseStamped, 'fmq/state', self.state_callback, qos_profile)

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