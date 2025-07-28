import rclpy

from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from rclpy.node import Node
from rclpy.time import Time, Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from atmos_fmq_msgs.msg import DelayRobotState, MultiDelayRobotState, MultiDelayWrenchControl
from px4_msgs.msg import VehicleTorqueSetpoint, VehicleThrustSetpoint, OffboardControlMode, VehicleAngularVelocity, VehicleAttitude, VehicleLocalPosition

from copy import deepcopy
from functools import partial

class ControlFeeder(Node):
    def __init__(self):
        super().__init__('control_feeder')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Timers
        self.pub_to_robot_timer = self.create_timer(0.01, self.publish_to_robots)
        self.pub_to_ctrl_timer = self.create_timer(0.1, self.publish_to_ctrler)

        # FleetMQ messages pub/sub
        self.state_pub = self.create_publisher(MultiDelayRobotState, '/fmq/state', qos_profile)
        self.control_sub = self.create_subscription(MultiDelayWrenchControl, '/fmq/control', self.control_callback, qos_profile)

        # Get namespace names
        self.namespaces = self.declare_parameter('namespaces', ['']).value

        # Create pub/sub and internal states for each namespace
        self.thrust_setpoint_pubs: dict[str, Publisher] = {}
        self.torque_setpoint_pubs: dict[str, Publisher] = {}
        self.offboard_control_mode_pubs: dict[str, Publisher] = {}

        self.angular_velocity_subs: dict[str, Subscription] = {}
        self.attitude_subs: dict[str, Subscription] = {}
        self.local_position_subs: dict[str, Subscription] = {}

        self.latest_thrust_setpoints: dict[str, VehicleThrustSetpoint] = {}
        self.latest_torque_setpoints: dict[str, VehicleTorqueSetpoint] = {}
        self.latest_offboard_control_modes: dict[str, OffboardControlMode] = {}
        self.latest_control_ids: dict[str, int] = {}
        self.rec_times: dict[str, list[int, Time, Duration]] = {}
        self.control_arrival_times: dict[str, Time] = {}

        self.control_on: dict[str, bool] = {}
        self.angular_velocity_on: dict[str, bool] = {}
        self.attitude_on: dict[str, bool] = {}
        self.local_position_on: dict[str, bool] = {}

        self.delay_robot_state_msgs: dict[str, DelayRobotState] = {}
        
        for ns in self.namespaces:
            msg_prefix = '' if ns == '' else f'/{ns}'
            self.thrust_setpoint_pubs[ns] = self.create_publisher(VehicleThrustSetpoint, f'{msg_prefix}/fmu/in/vehicle_thrust_setpoint', qos_profile)
            self.torque_setpoint_pubs[ns] = self.create_publisher(VehicleTorqueSetpoint, f'{msg_prefix}/fmu/in/vehicle_torque_setpoint', qos_profile)
            self.offboard_control_mode_pubs[ns] = self.create_publisher(OffboardControlMode, f'{msg_prefix}/fmu/in/offboard_control_mode', qos_profile)

            self.angular_velocity_subs[ns] = self.create_subscription(
                VehicleAngularVelocity,
                f'{msg_prefix}/fmu/out/vehicle_angular_velocity',
                partial(self.angular_velocity_callback, namespace=ns),
                qos_profile
            )
            self.attitude_subs[ns] = self.create_subscription(
                VehicleAttitude,
                f'{msg_prefix}/fmu/out/vehicle_attitude',
                partial(self.attitude_callback, namespace=ns),
                qos_profile
            )
            self.local_position_subs[ns] = self.create_subscription(
                VehicleLocalPosition,
                f'{msg_prefix}/fmu/out/vehicle_local_position',
                partial(self.local_position_callback, namespace=ns),
                qos_profile
            )

            self.latest_thrust_setpoints[ns] = VehicleThrustSetpoint()
            self.latest_torque_setpoints[ns] = VehicleTorqueSetpoint()
            self.latest_offboard_control_modes[ns] = OffboardControlMode()
            self.latest_control_ids[ns] = int(0)
            self.rec_times[ns] = []

            self.control_on[ns] = False
            self.angular_velocity_on[ns] = False
            self.attitude_on[ns] = False
            self.local_position_on[ns] = False
            self.delay_robot_state_msgs[ns] = DelayRobotState()
            self.delay_robot_state_msgs[ns].robot_name = ns

            self.get_logger().info(f'Initialized pub/sub for robot name = \'{ns}\'')

    def publish_to_robots(self):
        for ns in self.namespaces:
            if self.control_on[ns]:
                self.thrust_setpoint_pubs[ns].publish(self.latest_thrust_setpoints[ns])
                self.torque_setpoint_pubs[ns].publish(self.latest_torque_setpoints[ns])
                self.offboard_control_mode_pubs[ns].publish(self.latest_offboard_control_modes[ns])

    def publish_to_ctrler(self):
        msg = MultiDelayRobotState()
        msg.robot_states = []
        for ns in self.namespaces:
            cur_time = self.get_clock().now()
            if self.control_on[ns] and self.angular_velocity_on[ns] and self.attitude_on[ns] and self.local_position_on[ns]:
                self.delay_robot_state_msgs[ns].header.stamp = cur_time.to_msg()
                self.delay_robot_state_msgs[ns].latest_ctrl_id = self.latest_control_ids[ns]
                self.delay_robot_state_msgs[ns].sec_since_latest_ctrl = (cur_time - self.control_arrival_times[ns]).nanoseconds / 1e9
                self.delay_robot_state_msgs[ns].transmission_delays = [dur.to_msg() for _, _, dur in self.rec_times[ns]]
                msg.robot_states.append(deepcopy(self.delay_robot_state_msgs[ns]))
        self.state_pub.publish(msg)

    def control_callback(self, msg: MultiDelayWrenchControl):
        for control in msg.wrench_controls:
            ns = control.robot_name
            if ns not in self.namespaces:
                self.get_logger().log(f'Unknown robot name: {ns}')
                continue

            if not self.control_on[ns]:
                self.control_on[ns] = True

            last_time = control.state_timestamp

            self.latest_offboard_control_modes[ns] = control.offboard_control_mode
            self.latest_thrust_setpoints[ns] = control.vehicle_thrust_setpoint
            self.latest_torque_setpoints[ns] = control.vehicle_torque_setpoint
            self.latest_control_ids[ns] = control.id
            self.control_arrival_times[ns] = self.get_clock().now()

            while True:
                if len(self.rec_times[ns]) == 0:
                    break
                if self.rec_times[ns][0][1] < Time.from_msg(last_time):
                    self.rec_times[ns].pop(0)
                else:
                    break

            if len(self.rec_times[ns]) == 0:
                self.rec_times[ns].append((control.id, self.control_arrival_times[ns], self.control_arrival_times[ns] - Time.from_msg(control.header.stamp)))
            elif len(self.rec_times[ns]) > 0:
                if self.rec_times[ns][-1][0] < control.id:
                    id_prev = self.rec_times[ns][-1][0]
                    for id in range(id_prev + 1, control.id):
                        self.rec_times[ns].append((id, self.control_arrival_times[ns], Duration(seconds=1e8))) # append big time to represent drops
                    self.rec_times[ns].append((control.id, self.control_arrival_times[ns], self.control_arrival_times[ns] - Time.from_msg(control.header.stamp)))

    def angular_velocity_callback(self, msg: VehicleAngularVelocity, namespace):
        if not self.angular_velocity_on[namespace]:
            self.angular_velocity_on[namespace] = True
        self.delay_robot_state_msgs[namespace].vehicle_angular_velocity = deepcopy(msg)

    def attitude_callback(self, msg: VehicleAttitude, namespace):
        if not self.attitude_on[namespace]:
            self.attitude_on[namespace] = True
        self.delay_robot_state_msgs[namespace].vehicle_attitude = deepcopy(msg)

    def local_position_callback(self, msg: VehicleLocalPosition, namespace):
        if not self.local_position_on[namespace]:
            self.local_position_on[namespace] = True
        self.delay_robot_state_msgs[namespace].vehicle_local_position = deepcopy(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControlFeeder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
