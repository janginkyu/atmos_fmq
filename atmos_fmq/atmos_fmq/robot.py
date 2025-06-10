import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# from atmos_fmq_msgs.msg import RobotState
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleLocalPosition


class MsgHandlerRobot(Node):
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

        self.state_pub = self.create_publisher(PoseStamped, f'{self.namespace_prefix}/fmq/state', fmq_qos_profile)
        self.setpoint_pub = self.create_publisher(PoseStamped, f'{self.namespace_prefix}/px4_mpc/setpoint_pose', 10)

        self.local_position_sub = self.create_subscription(VehicleLocalPosition, f'{self.namespace_prefix}/fmu/out/vehicle_local_position', self.local_position_callback, fmq_qos_profile)
        self.control_sub = self.create_subscription(PoseStamped, f'{self.namespace_prefix}/fmq/control', self.control_callback, fmq_qos_profile)
        self.last_pub = 0.0

    def local_position_callback(self, msg: VehicleLocalPosition):
        pub_msg = PoseStamped()
        pub_msg.pose.position.x = msg.x
        pub_msg.pose.position.y = msg.y
        pub_msg.pose.position.z = msg.z
        pub_msg.pose.orientation.w = np.cos(msg.heading / 2)
        pub_msg.pose.orientation.x = 0.0
        pub_msg.pose.orientation.y = 0.0
        pub_msg.pose.orientation.z = np.sin(msg.heading / 2)
        pub_msg.header.frame_id = 'map'
        pub_msg.header.stamp = self.get_clock().now().to_msg()

        this_time = self.get_clock().now().nanoseconds / 1000000000
        if this_time >= self.last_pub + 0.1:
            self.last_pub = this_time
            self.state_pub.publish(pub_msg)

    def control_callback(self, msg: PoseStamped):
        self.setpoint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MsgHandlerRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
