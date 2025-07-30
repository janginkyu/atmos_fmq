import rclpy

from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from rclpy.node import Node
from rclpy.time import Time, Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import numpy as np

from geometry_msgs.msg import PoseStamped, TwistStamped


class PublishSetpoints(Node):
    def __init__(self):
        super().__init__('publish_setpoints')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Get namespace names
        self.namespaces = self.declare_parameter('namespaces', ['']).value

        r = 0.5
        theta = 2.0 * np.pi / 3.0
        self.initial_poses = {
            'pop': [r, -2.0, 0.0],
            'crackle': [r * np.cos(theta), -2.0 + r * np.sin(theta), 0.0],
            'snap': [r * np.cos(2.0 * theta), -2.0 + r * np.sin(2.0 * theta), 0.0],
        }

        self.publishers_pose = {}
        self.publishers_twist = {}

        for ns in self.namespaces:
            msg_prefix = '' if ns == '' else f'/{ns}'
            self.publishers_pose[ns] = self.create_publisher(PoseStamped, f'{msg_prefix}/setpoint_pose', qos_profile)
            self.publishers_twist[ns] = self.create_publisher(TwistStamped, f'{msg_prefix}/setpoint_twist', qos_profile)

            if ns not in self.initial_poses.keys():
                self.initial_poses[ns] = np.array([1.0, 1.0, 0.0])

        self.pub_setpoints_timer = self.create_timer(0.02, self.publish_setpoints)
        
        self.start_time = self.get_clock().now()
        self.omega = 0.2
        self.center = [0.0, -2.0]

    def publish_setpoints(self):
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1.0e9 - 5.0
        if t >= 0:
            for (ns, pose) in self.initial_poses.items():
                a = (pose[0] - self.center[0])
                b = (pose[1] - self.center[1])

                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'world'
                pose_msg.pose.position.x = self.center[0] + a * np.cos(t * self.omega) - b * np.sin(t * self.omega)
                pose_msg.pose.position.y = self.center[1] + a * np.sin(t * self.omega) + b * np.cos(t * self.omega)
                pose_msg.pose.position.z = 0.0
                pose_msg.pose.orientation.w = np.cos((pose[2] + t * self.omega) / 2)
                pose_msg.pose.orientation.x = 0.0
                pose_msg.pose.orientation.y = 0.0
                pose_msg.pose.orientation.z = np.sin((pose[2] + t * self.omega) / 2)
                self.publishers_pose[ns].publish(pose_msg)

                twist_msg = TwistStamped()
                twist_msg.header.stamp = self.get_clock().now().to_msg()
                twist_msg.header.frame_id = 'world'
                twist_msg.twist.linear.x = (-a * np.sin(t * self.omega) - b * np.cos(t * self.omega)) * self.omega
                twist_msg.twist.linear.y = (a * np.cos(t * self.omega) - b * np.sin(t * self.omega)) * self.omega
                twist_msg.twist.linear.z = 0.0
                twist_msg.twist.angular.x = 0.0
                twist_msg.twist.angular.y = 0.0
                twist_msg.twist.angular.z = self.omega
                self.publishers_twist[ns].publish(twist_msg)

        else:
            for (ns, pose) in self.initial_poses.items():
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'world'
                pose_msg.pose.position.x = pose[0]
                pose_msg.pose.position.y = pose[1]
                pose_msg.pose.position.z = 0.0
                pose_msg.pose.orientation.w = np.cos(pose[2] / 2)
                pose_msg.pose.orientation.x = 0.0
                pose_msg.pose.orientation.y = 0.0
                pose_msg.pose.orientation.z = np.sin(pose[2] / 2)
                self.publishers_pose[ns].publish(pose_msg)

                twist_msg = TwistStamped()
                twist_msg.header.stamp = self.get_clock().now().to_msg()
                twist_msg.header.frame_id = 'world'
                twist_msg.twist.linear.x = 0.0
                twist_msg.twist.linear.y = 0.0
                twist_msg.twist.linear.z = 0.0
                twist_msg.twist.angular.x = 0.0
                twist_msg.twist.angular.y = 0.0
                twist_msg.twist.angular.z = 0.0
                self.publishers_twist[ns].publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PublishSetpoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
