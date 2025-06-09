import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from atmos_fmq_msgs.msg import RobotState
from atmos_fmq_msgs.msg import WrenchControl
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude, VehicleAngularVelocity, VehicleStatus
from px4_msgs.msg import ActuatorMotors, VehicleThrustSetpoint, VehicleTorqueSetpoint, OffboardControlMode


class MsgHandlerRobot(Node):
    def __init__(self):
        super().__init__('msg_handler_robot')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # QoS profiles for PX4
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

        # Control mode:
        self.mode = 'wrench'

        # Initialize variables
        self.lpe = None
        self.attitude = None
        self.angular_velocity = None
        self.status = None
        self.received_ctl = None

        # Get namespace
        self.namespace = self.declare_parameter('namespace', '').value
        self.namespace_prefix = f'/{self.namespace}' if self.namespace else ''

        # Send out State
        self.fmq_state_pub = self.create_publisher(RobotState,  f'{self.namespace_prefix}/fmq/state', qos_profile)
        self.local_position_sub = self.create_subscription(VehicleLocalPosition,  f'{self.namespace_prefix}/fmu/out/vehicle_local_position', self.local_position_callback, qos_profile_sub)
        self.attitude_sub = self.create_subscription(VehicleAttitude,  f'{self.namespace_prefix}/fmu/out/vehicle_attitude', self.attitude_callback, qos_profile_sub)
        self.angular_velocity_sub = self.create_subscription(VehicleAngularVelocity,  f'{self.namespace_prefix}/fmu/out/vehicle_angular_velocity', self.angular_velocity_callback, qos_profile_sub)
        self.status_sub = self.create_subscription(VehicleStatus,  f'{self.namespace_prefix}/fmu/out/vehicle_status', self.status_callback, qos_profile_sub)

        # Handle Controls
        self.fmq_control_sub = self.create_subscription(WrenchControl,  f'{self.namespace_prefix}/fmq/control', self.control_callback, qos_profile)
        self.publisher_direct_actuator = self.create_publisher(ActuatorMotors,  f'{self.namespace_prefix}/fmu/in/actuator_motors', qos_profile_pub)
        self.publisher_thrust_setpoint = self.create_publisher(VehicleThrustSetpoint,  f'{self.namespace_prefix}/fmu/in/vehicle_thrust_setpoint', qos_profile_pub)
        self.publisher_torque_setpoint = self.create_publisher(VehicleTorqueSetpoint,  f'{self.namespace_prefix}/fmu/in/vehicle_torque_setpoint', qos_profile_pub)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode,  f'{self.namespace_prefix}/fmu/in/offboard_control_mode', qos_profile_pub)

        # Create timers
        self.timer_control = self.create_timer(0.1, self.robot_ctl_callback)
        self.timer_state = self.create_timer(0.05, self.robot_state_callback)

        self.last_pub = 0.0

    def local_position_callback(self, msg: VehicleLocalPosition):
        self.lpe = msg

    def attitude_callback(self, msg: VehicleAttitude):
        self.attitude = msg

    def angular_velocity_callback(self, msg: VehicleAngularVelocity):
        self.angular_velocity = msg

    def status_callback(self, msg: VehicleStatus):
        self.status = msg

    def control_callback(self, msg: WrenchControl):
        self.received_ctl.publish(msg)

    def robot_state_callback(self):
        if self.lpe is None or self.attitude is None or self.angular_velocity is None or self.status is None:
            self.get_logger().warn(self.namespace_prefix + ': Not all state variables are set, skipping state publication')
            return
        state = RobotState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.vehicle_local_position = self.lpe
        state.vehicle_attitude = self.attitude
        state.vehicle_angular_velocity = self.angular_velocity
        state.vehicle_status = self.status
        self.fmq_state_pub.publish(state)

    def robot_ctl_callback(self):
        if self.received_ctl is None:
            self.get_logger().warn(self.namespace_prefix + 'No control message received, skipping setpoint publication')
            return
        ctl_torque = self.received_ctl.vehile_torque_setpoint
        ctl_thrust = self.received_ctl.vehile_thrust_setpoint
        ctl_offboard_mode = self.received_ctl.offboard_control_mode

        # Publish mode:
        self.publisher_offboard_mode.publish(ctl_offboard_mode)
        self.publisher_thrust_setpoint.publish(ctl_thrust)
        self.publisher_torque_setpoint.publish(ctl_torque)


def main(args=None):
    rclpy.init(args=args)
    node = MsgHandlerRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
