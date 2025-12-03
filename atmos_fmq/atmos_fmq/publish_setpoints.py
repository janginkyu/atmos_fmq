import rclpy

from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from rclpy.node import Node
from rclpy.time import Time, Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_srvs.srv import SetBool
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
            # 'crackle': [r * np.cos(theta), -2.0 + r * np.sin(theta), 0.0],
            # 'snap': [r * np.cos(2.0 * theta), -2.0 + r * np.sin(2.0 * theta), 0.0],
        }

        self.R_traj = 0.5; self.omega_traj = 2.0*np.pi*0.1/3
        self.init_delay_service_ = self.create_service(SetBool,"/delay_call", self.delay_on)
        self.init_delay = False

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
        self.omega = 0.0 # 0.2
        self.center = [0.0, -2.0]
    
    def delay_on(self, req:SetBool.Request, res:SetBool.Response):
        if not self.init_delay:
            self.init_delay = True
            self.get_logger().info("Delay On")
        else:
            self.init_delay = False
            self.get_logger().info("Delay Off")
            res.success = True
        return res
    
    def publish_setpoints(self):
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1.0e9 # - 5.0
        if t >= 0:
            for (ns, pose) in self.initial_poses.items():
                a = (pose[0] - self.center[0])
                b = (pose[1] - self.center[1])
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'world'
                if not self.init_delay:
                    self.start_time = self.get_clock().now()
                    pose_msg.pose.position.x = 0.5 # 1.5 #self.center[0] + a * np.cos(t * self.omega) - b * np.sin(t * self.omega)
                    pose_msg.pose.position.y = 2.0 # 1.62 #self.center[1] + a * np.sin(t * self.omega) + b * np.cos(t * self.omega)
                    pose_msg.pose.position.z = 0.0
                    
                    yaw_ = pose[2]
                    pose_msg.pose.orientation.w = np.cos((yaw_) / 2)
                    pose_msg.pose.orientation.x = 0.0
                    pose_msg.pose.orientation.y = 0.0
                    pose_msg.pose.orientation.z = np.sin((yaw_) / 2)
                    self.publishers_pose[ns].publish(pose_msg)

                    twist_msg = TwistStamped()
                    twist_msg.header.stamp = self.get_clock().now().to_msg()
                    twist_msg.header.frame_id = 'world'
                    twist_msg.twist.linear.x = 0.0 #(-a * np.sin(t * self.omega) - b * np.cos(t * self.omega)) * self.omega
                    twist_msg.twist.linear.y = 0.0 #(a * np.cos(t * self.omega) - b * np.sin(t * self.omega)) * self.omega
                    twist_msg.twist.linear.z = 0.0
                    twist_msg.twist.angular.x = 0.0
                    twist_msg.twist.angular.y = 0.0
                    twist_msg.twist.angular.z = self.omega
                    self.publishers_twist[ns].publish(twist_msg)

                else :
                    pose_msg.pose.position.x = 0.0 + self.R_traj * np.cos(self.omega_traj*t) # 1.5 #self.center[0] + a * np.cos(t * self.omega) - b * np.sin(t * self.omega)
                    pose_msg.pose.position.y = 2.0 + self.R_traj * np.sin(self.omega_traj*t) # 1.62 #self.center[1] + a * np.sin(t * self.omega) + b * np.cos(t * self.omega)
                    pose_msg.pose.position.z = 0.0
                    yaw_ = pose[2] + t * self.omega_traj
                    yaw_rate_ = self.omega_traj
                    pose_msg.pose.orientation.w = np.cos((yaw_) / 2)
                    pose_msg.pose.orientation.x = 0.0
                    pose_msg.pose.orientation.y = 0.0
                    pose_msg.pose.orientation.z = np.sin((yaw_) / 2)
                    self.publishers_pose[ns].publish(pose_msg)

                    twist_msg = TwistStamped()
                    twist_msg.header.stamp = self.get_clock().now().to_msg()
                    twist_msg.header.frame_id = 'world'
                    twist_msg.twist.linear.x = -self.omega_traj*self.R_traj * np.sin(self.omega_traj*t) #(-a * np.sin(t * self.omega) - b * np.cos(t * self.omega)) * self.omega
                    twist_msg.twist.linear.y =  self.omega_traj*self.R_traj * np.cos(self.omega_traj*t) #(a * np.cos(t * self.omega) - b * np.sin(t * self.omega)) * self.omega
                    twist_msg.twist.linear.z = 0.0
                    twist_msg.twist.angular.x = 0.0
                    twist_msg.twist.angular.y = 0.0
                    twist_msg.twist.angular.z = yaw_rate_
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
