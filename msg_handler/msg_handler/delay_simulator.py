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
            # '/fmu/out/vehicle_angular_velocity': VehicleAngularVelocity,
            # '/fmu/out/vehicle_attitude': VehicleAttitude,
            # '/fmu/out/vehicle_local_position': VehicleLocalPosition,
            # '/prefix/fmu/in/offboard_control_mode': OffboardControlMode,
            # '/prefix/fmu/in/vehicle_thrust_setpoint': VehicleThrustSetpoint,
            # '/prefix/fmu/in/vehicle_torque_setpoint': VehicleTorqueSetpoint,
            '/prefix/px4_mpc/setpoint_pose': PoseStamped,
        }
        # self.topic_msg_dict_ = {
        #     '/prefix/msg1': PoseStamped,
        #     '/prefix/msg2': PoseStamped,
        # }

        self.publishers_ = {}
        self.subscribers_ = {}
        self.msg_queues = {}


        qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0
        )

        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0
        )


        # 각 토픽에 대한 publisher/subscriber 등록
        for topic_name_, msg_type_ in self.topic_msg_dict_.items():
            if topic_name_.startswith('/prefix'):
                delay_pub_topic_ = topic_name_.replace('/prefix', '')
            else:
                delay_pub_topic_ = '/prefix' + topic_name_

            # publisher: /msg1 등
            self.publishers_[delay_pub_topic_] = self.create_publisher(msg_type_, delay_pub_topic_, qos_profile_pub)
            self.msg_queues[delay_pub_topic_] = deque()
            # subscriber: /prefix/msg1 등
            self.subscribers_[delay_pub_topic_] = self.create_subscription(
                msg_type_, topic_name_,
                lambda msg, t=delay_pub_topic_: self.delay_callback(msg, t),
                qos_profile_sub
            )

        self.create_timer(0.01, self.delayed_publish) 

    def delay_callback(self, msg, topic_name):
        if np.random.rand() < 0.3 and self.use_delay_:
            return
        delay_ms = np.clip(np.random.randn()*300, 100, 1000)
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