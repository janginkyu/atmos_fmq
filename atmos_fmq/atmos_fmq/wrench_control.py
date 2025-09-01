import rclpy
import numpy as np
from qpsolvers import solve_qp

from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.time import Time, Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, TwistStamped
from atmos_fmq_msgs.msg import DelayRobotState, DelayWrenchControl, MultiDelayRobotState, MultiDelayWrenchControl
from px4_msgs.msg import OffboardControlMode, VehicleThrustSetpoint, VehicleTorqueSetpoint

import random
from functools import partial

def wrap_to_pi(x_):
    x = x_
    while True:
        if x > np.pi:
            x -= np.pi * 2.0
        elif x < -np.pi:
            x += np.pi * 2.0
        else:
            break
    return x

class SpacecraftModel():
    def __init__(self):
        self.dt = 0.01
        self.mass = 16.8
        self.inertia = 0.1594
        self.max_force = 1.5
        self.max_torque = 0.5

        # delay model
        self.delay_min = 0.1 # minimum delay in seconds, one-way
        self.delay_max = 0.5 # maximum delay in seconds, one-way

        # lqr model
        self.P = np.array([
                [0.8476,    0.0000,    0.0000,    3.5418,    0.0000,    0.0000],
                [0.0000,    0.8476,   -0.0000,    0.0000,    3.5418,   -0.0000],
                [0.0000,   -0.0000,    0.1737,    0.0000,   -0.0000,    0.1008],
                [3.5418,    0.0000,    0.0000,   30.0183,    0.0000,    0.0000],
                [0.0000,    3.5418,   -0.0000,    0.0000,   30.0183,   -0.0000],
                [0.0000,   -0.0000,    0.1008,    0.0000,   -0.0000,    0.1751]])

    def f(self, x):
        return np.array([x[3], x[4], x[5], 0.0, 0.0, 0.0])

    def g(self, x):
        return np.array([
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [np.cos(x[2]) / self.mass, -np.sin(x[2]) / self.mass, 0.0],
            [np.sin(x[2]) / self.mass, np.cos(x[2]) / self.mass, 0.0],
            [0.0, 0.0, 1.0 / self.inertia]
        ])

    # inputs should be a list of (input, duration) tuples
    def predict(self, inputs, x0):
        x = x0.copy()
        for duration, u in inputs:
            steps = int(duration / self.dt) + 1
            dt = duration / steps
            for _ in range(steps):
                # do prediction for the spacecraft model
                # u[0] is forward force, u[1] is lateral force, u[2] is angular acceleration
                x += (self.f(x) + self.g(x) @ u) * dt
                x[2] = wrap_to_pi(x[2])
        return x

    def lyapunov(self, x, x0):
        # Lyapunov function: V(x) = 0.5 * ((x - x0)^T * P * (x - x0))
        # where P is a positive definite matrix
        # solve LQR in matlab to obtain P
        dx = x - x0
        dx[2] = np.sin(wrap_to_pi(dx[2]) / 2) * 2
        grad = np.dot(self.P, dx)
        grad[2] *= np.cos(dx[2] / 2)
        return 0.5 * np.dot(dx, np.dot(self.P, dx)), grad

    def control(self, x, x0):
        kx = 0.47
        kxdot = 4.02
        ky = 0.47
        kydot = 4.02
        ktheta = 0.08
        kw = 0.13
        dx = x - x0
        dx[2] = wrap_to_pi(dx[2])
        vxdot_des = -kx * dx[0] - kxdot * dx[3]
        vydot_des = -ky * dx[1] - kydot * dx[4]
        alpha_des = -ktheta * dx[2] - kw * dx[5]

        u = np.zeros(3)
        u[0] = vxdot_des * np.cos(x[2]) + vydot_des * np.sin(x[2])
        u[1] = -vxdot_des * np.sin(x[2]) + vydot_des * np.cos(x[2])
        u[2] = alpha_des

        u[0] = np.clip(u[0], -self.max_force, self.max_force)
        u[1] = np.clip(u[1], -self.max_force, self.max_force)
        u[2] = np.clip(u[2], -self.max_torque, self.max_torque)

        return u
    
class ControllerInstance():
    def __init__(self, ns: str='', x=np.zeros(6), ctrl_dt=0.1):
        self.ns = ns
        self.x0 = x.copy() # desired state, set to initial state at start
        self.x0[2] = wrap_to_pi(self.x0[2])
        self.x = x.copy() # state from the robot
        self.x[2] = wrap_to_pi(self.x[2])
        self.last_msg = None
        self.model = SpacecraftModel()
        self.last_setpoint_stamp = None
        self.ctrl_id = 0
        self.pending_inputs = []  # list of (id, time, input) tuples
        self.delay_history = []
        self.ctrl_dt = ctrl_dt
    
    def handle_state_msg(self, msg: DelayRobotState):
        self.last_msg = msg
        self.x[0] = msg.vehicle_local_position.x
        self.x[1] = -msg.vehicle_local_position.y
        # self.x[2] = 2.0 * np.arctan2(-msg.vehicle_attitude.q[3], -msg.vehicle_attitude.q[0])
        self.x[2] = -msg.vehicle_local_position.heading
        self.x[3] = msg.vehicle_local_position.vx
        self.x[4] = -msg.vehicle_local_position.vy
        self.x[5] = -msg.vehicle_angular_velocity.xyz[2]

        while len(self.pending_inputs) > 0 and self.pending_inputs[0][0] < msg.latest_ctrl_id:
            self.pending_inputs.pop(0)

        for dur in msg.transmission_delays:
            self.delay_history.append(dur)
            if len(self.delay_history) > 100:
                self.delay_history.pop(0)

    def generate_control(self, stamp: Time, control_on: bool) -> tuple[DelayWrenchControl, PoseStamped, TwistStamped]:
        # do prediction here
        x_for_control = np.zeros(6)
        if self.last_msg is None:
            u = np.zeros(3)
        else:
            # do prediction
            worst_lyap_val = -np.inf
            Aineq = [
            ]
            bineq = [
            ]
            print(self.ns, self.x0, self.x)
            n_preds = 10
            for _ in range(n_preds):
                x0 = self.x0.copy()
                # delay_profile = [np.clip(self.ctrl_dt + random.uniform(self.model.delay_min, self.model.delay_max) - random.uniform(self.model.delay_min, self.model.delay_max), 0.0, np.inf) for _ in range(len(self.pending_inputs))]

                if len(self.delay_history) >= len(self.pending_inputs):
                    td_profile = [Duration.from_msg(random.choice(self.delay_history)).nanoseconds / 1e9 for _ in range(len(self.pending_inputs))]
                else:
                    td_profile = [random.uniform(self.model.delay_min, self.model.delay_max) for _ in range(len(self.pending_inputs))]
                td_profile.insert(0, -self.last_msg.sec_since_latest_ctrl)
                delay_profile = [np.clip(td_profile[i + 1] - td_profile[i] + self.ctrl_dt,
                                         0,
                                        self.model.delay_max + self.ctrl_dt 
                                        #  np.inf if i < len(td_profile) - 2 else self.model.delay_max + self.ctrl_dt
                                        ) for i in range(len(td_profile) - 1)]

                inputs = [(delay, pending_input[2]) for delay, pending_input in zip(delay_profile, self.pending_inputs)]

                x_pred = self.model.predict(inputs, self.x)

                lyap_val, lyap_grad = self.model.lyapunov(x_pred, x0)

                LfV = np.dot(lyap_grad, self.model.f(x_pred))
                LfG = list(lyap_grad.T @ self.model.g(x_pred))
                LfG.append(-1.0)
                bineq.append(-LfV - 1.0 * lyap_val)
                Aineq.append(LfG)

                x_for_control += x_pred / n_preds
                # lyap_grad
                if lyap_val > worst_lyap_val:
                    worst_lyap_val = lyap_val

            P = np.array([
                [1.0, 0, 0, 0],
                [0, 1.0, 0, 0],
                [0, 0, 30.0, 0],
                [0, 0, 0, 0.0]
            ])
            q = np.array([0.0, 0.0, 0.0, 1.0])
            Aineq = np.array(Aineq)
            bineq = np.array(bineq)
            lb = np.array([-self.model.max_force, -self.model.max_force, -self.model.max_torque, 0.0])
            ub = np.array([self.model.max_force, self.model.max_force, self.model.max_torque, np.inf])
            # sol = solvers.qp(Q, p, Aineq, bineq)
            if control_on:
                sol = solve_qp(P=P, q=q, G=Aineq, h=bineq, lb=lb, ub=ub, solver='cvxopt')
                u = sol[0:3]
            else:
                u = np.zeros(3)
            # u = self.model.control(x_for_control, self.x0)
            # u = self.model.control(self.x, self.x0)

        pose_pred_msg = PoseStamped()
        pose_pred_msg.header.stamp = stamp.to_msg()
        pose_pred_msg.header.frame_id = 'world'
        pose_pred_msg.pose.position.x = x_for_control[0]
        pose_pred_msg.pose.position.y = x_for_control[1]
        pose_pred_msg.pose.position.z = 0.0
        pose_pred_msg.pose.orientation.w = np.cos(x_for_control[2] / 2)
        pose_pred_msg.pose.orientation.x = 0.0
        pose_pred_msg.pose.orientation.y = 0.0
        pose_pred_msg.pose.orientation.z = np.sin(x_for_control[2] / 2)

        twist_pred_msg = TwistStamped()
        twist_pred_msg.header.stamp = stamp.to_msg()
        twist_pred_msg.header.frame_id = 'world'
        twist_pred_msg.twist.linear.x = x_for_control[3]
        twist_pred_msg.twist.linear.y = x_for_control[4]
        twist_pred_msg.twist.linear.z = 0.0
        twist_pred_msg.twist.angular.x = 0.0
        twist_pred_msg.twist.angular.y = 0.0
        twist_pred_msg.twist.angular.z = x_for_control[5]

        px4_timestamp = int(stamp.nanoseconds / 1e3)
        control_msg = DelayWrenchControl()
        control_msg.header.stamp = stamp.to_msg()
        control_msg.header.frame_id = ''
        control_msg.id = self.ctrl_id
        control_msg.state_timestamp = self.last_msg.header.stamp if self.last_msg is not None else Time().to_msg()
        control_msg.offboard_control_mode = OffboardControlMode()
        control_msg.offboard_control_mode.timestamp = px4_timestamp
        control_msg.offboard_control_mode.position = False
        control_msg.offboard_control_mode.velocity = False
        control_msg.offboard_control_mode.acceleration = False
        control_msg.offboard_control_mode.attitude = False
        control_msg.offboard_control_mode.body_rate = False
        control_msg.offboard_control_mode.direct_actuator = False
        control_msg.offboard_control_mode.thrust_and_torque = True
        
        control_msg.vehicle_thrust_setpoint = VehicleThrustSetpoint()
        control_msg.vehicle_torque_setpoint = VehicleTorqueSetpoint()
        control_msg.vehicle_thrust_setpoint.timestamp = px4_timestamp
        control_msg.vehicle_thrust_setpoint.xyz = [u[0], -u[1], 0.0]
        control_msg.vehicle_torque_setpoint.timestamp = px4_timestamp
        control_msg.vehicle_torque_setpoint.xyz = [0.0, 0.0, -u[2]]

        control_msg.robot_name = self.ns

        self.pending_inputs.append((self.ctrl_id, stamp, u))
        self.ctrl_id += 1

        return control_msg, pose_pred_msg, twist_pred_msg

    def register_setpoint(self, pose_sp: PoseStamped, twist_sp: TwistStamped, no_twist=True):
        if pose_sp is not None:
            self.x0[0] = pose_sp.pose.position.x
            self.x0[1] = pose_sp.pose.position.y
            self.x0[2] = 2.0 * np.arctan2(pose_sp.pose.orientation.z, pose_sp.pose.orientation.w)
        else:
            pass # keep pose unchanged

        if twist_sp is not None:
            self.x0[3] = twist_sp.twist.linear.x
            self.x0[4] = twist_sp.twist.linear.y
            self.x0[5] = twist_sp.twist.angular.z
        elif no_twist:
            self.x0[3] = 0.0
            self.x0[4] = 0.0
            self.x0[5] = 0.0
        else:
            pass # keep twist unchanged


class MultiWrenchControl(Node):
    def __init__(self):
        super().__init__('multi_wrench_control')

        self.namespaces = self.declare_parameter('namespaces', ['']).value
        self.simulated_delay = self.declare_parameter('simulated_delay', False).value
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.ctrl_dt = 0.1

        # add predicted pose and twist publishers (for referece generation)
        self.pose_pub = {}
        self.twist_pub = {}
        for ns in self.namespaces:
            self.pose_pub[ns] = self.create_publisher(PoseStamped, f'/{ns}/predicted_pose', qos_profile)
            self.twist_pub[ns] = self.create_publisher(TwistStamped, f'/{ns}/predicted_twist', qos_profile)

        self.control_on = {}
        for ns in self.namespaces:
            self.control_on[ns] = False
        self.control_on_sub = [
            self.create_subscription(Bool, f'/{ns}/control_on', partial(self.control_on_callback, namespace=ns), qos_profile)
        ]

        self.control_pub = self.create_publisher(
            MultiDelayWrenchControl,
            '/fmq/control/undelayed' if self.simulated_delay else '/fmq/control',
            qos_profile
        )
        self.state_sub = self.create_subscription(
            MultiDelayRobotState,
            '/fmq/state/delayed' if self.simulated_delay else '/fmq/state',
            self.state_callback, qos_profile
        )
        self.pub_timer = self.create_timer(self.ctrl_dt, self.publish_control_msgs)

        self.setpoint_subs = []
        self.robots: dict[str, ControllerInstance] = {}
        self.has_twist: dict[str, bool] = {}
        for ns in self.namespaces:
            self.robots[ns] = ControllerInstance(ns=ns, x=np.zeros(6))
            msg_prefix = '' if ns == '' else f'/{ns}'
            self.setpoint_subs.append(self.create_subscription(
                PoseStamped,
                f'{msg_prefix}/setpoint_pose',
                partial(self.pose_setpoint_callback, namespace=ns),
                qos_profile
            ))
            self.setpoint_subs.append(self.create_subscription(
                TwistStamped,
                f'{msg_prefix}/setpoint_twist',
                partial(self.twist_setpoint_callback, namespace=ns),
                qos_profile
            ))
            self.has_twist[ns] = False
        self.get_clock().now()

    def state_callback(self, msg: MultiDelayRobotState):
        for robot_state in msg.robot_states:
            self.robots[robot_state.robot_name].handle_state_msg(robot_state)

    def publish_control_msgs(self):
        msg = MultiDelayWrenchControl()

        for (ns, inst) in self.robots.items():
            wrench_control, pose, twist = inst.generate_control(self.get_clock().now(), self.control_on[ns])
            msg.wrench_controls.append(wrench_control)
            self.pose_pub[ns].publish(pose)
            self.twist_pub[ns].publish(twist)

        self.control_pub.publish(msg)

    def pose_setpoint_callback(self, msg: PoseStamped, namespace: str):
        if self.robots[namespace] is not None:
            self.robots[namespace].register_setpoint(pose_sp=msg, twist_sp=None, no_twist=not self.has_twist[namespace])

    def twist_setpoint_callback(self, msg: TwistStamped, namespace: str):
        if self.robots[namespace] is not None:
            self.has_twist[namespace] = True
            self.robots[namespace].register_setpoint(pose_sp=None, twist_sp=msg, no_twist=False)

    def control_on_callback(self, msg: Bool, namespace: str):
        self.control_on[namespace] = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = MultiWrenchControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
