import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int16
import numpy as np
import random
import time
import copy
from collections import deque

from px4_msgs.msg import VehicleAngularVelocity, VehicleAttitude, VehicleLocalPosition # subscribed by controller
from px4_msgs.msg import OffboardControlMode, VehicleThrustSetpoint, VehicleTorqueSetpoint # published by controller
from atmos_fmq_msgs.msg import DelayRobotState, DelayWrenchControl, MultiDelayRobotState, MultiDelayWrenchControl

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


'''
/fmu/out/vehicle_angular_velocity
/fmu/out/vehicle_attitude
/fmu/out/vehicle_local_position
/fmu/in/offboard_control_mode
/fmu/in/vehicle_thrust_setpoint
/fmu/in/vehicle_torque_setpoint
'''

class DelayWrapper(Node):
    def __init__(self, UseDelay=True):
        super().__init__('delay_wrapper')
        self.use_delay_ = UseDelay
        self.topic_msg_dict_ = {
            '/fmq/state': (MultiDelayRobotState, '/fmq/state/delayed'),
            '/fmq/control/undelayed': (MultiDelayWrenchControl, '/fmq/control'),
        }

        self.publishers_ = {}
        self.subscribers_ = {}
        self.msg_queues = {}

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # 각 토픽에 대한 publisher/subscriber 등록
        for topic_name_, (msg_type_, delay_pub_topic_) in self.topic_msg_dict_.items():

            # publisher: /msg1 등
            self.publishers_[delay_pub_topic_] = self.create_publisher(msg_type_, delay_pub_topic_, qos_profile)
            self.msg_queues[delay_pub_topic_] = deque()
            # subscriber: /prefix/msg1 등
            self.subscribers_[delay_pub_topic_] = self.create_subscription(
                msg_type_, topic_name_,
                lambda msg, t=delay_pub_topic_: self.delay_callback(msg, t),
                qos_profile
            )

        self.create_timer(0.01, self.delayed_publish) 

    def delay_callback(self, msg, topic_name):
        if np.random.rand() < 0.2 and self.use_delay_:
            return
        delay_ms = np.clip(np.random.rand()*300, 100, 300)
        # delay_ms = 0
        delay_sec = delay_ms / 1000.0
        now = self.get_clock().now().nanoseconds / 1e9
        release_time = (now + delay_sec) if self.use_delay_ else now
        self.msg_queues[topic_name].append((copy.deepcopy(msg), release_time))


    def delayed_publish(self):
        now = self.get_clock().now().nanoseconds / 1e9
        for topic_name, queue in self.msg_queues.items():
            while queue and queue[0][1] <= now:
                msg, _ = queue.popleft()
                self.publishers_[topic_name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DelayWrapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

