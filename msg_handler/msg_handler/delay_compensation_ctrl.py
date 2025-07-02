import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# from atmos_fmq_msgs.msg import RobotState
from geometry_msgs.msg import PoseStamped
from atmos_fmq_msgs.msg import DelayRobotState, DelayWrenchControl
from px4_msgs.msg import OffboardControlMode, VehicleThrustSetpoint, VehicleTorqueSetpoint
from px4_msgs.msg import VehicleAngularVelocity, VehicleAttitude, VehicleLocalPosition
from sensor_msgs.msg import Joy

import random

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
        self.state = np.zeros(6) # [x, y, theta, vx, vy, w]
        self.dt = 0.01
        self.mass = 16.8
        self.inertia = 0.1594
        self.max_force = 1.5
        self.max_torque = 0.5

        # delay model
        self.delay_min = 0.05 # minimum delay in seconds, one-way
        self.delay_max = 0.15 # maximum delay in seconds, one-way
    
    # inputs should be a list of (input, duration) tuples
    def predict(self, inputs, x0):
        x = x0.copy()
        for duration, u in inputs:
            steps = int(duration / self.dt) + 1
            dt = duration / steps
            for _ in range(steps):
                # do prediction for the spacecraft model
                # u[0] is forward force, u[1] is lateral force, u[2] is angular acceleration
                x[0] += x[3] * dt
                x[1] += x[4] * dt
                x[2] += x[5] * dt
                x[3] += (u[0] * dt * np.cos(x[2]) - u[1] * dt * np.sin(x[2])) / self.mass  # forward velocity
                x[4] += (u[0] * dt * np.sin(x[2]) + u[1] * dt * np.cos(x[2])) / self.mass  # lateral velocity
                x[5] += (u[2] * dt) / self.inertia  # angular acceleration
        return x

    def lyapunov(self, x, x0):
        # Lyapunov function: V(x) = 0.5 * ((x - x0)^T * P * (x - x0))
        # where P is a positive definite matrix
        # solve LQR in matlab to obtain P
        P = np.array([
                [0.8476,    0.0000,    0.0000,    3.5418,    0.0000,    0.0000],
                [0.0000,    0.8476,   -0.0000,    0.0000,    3.5418,   -0.0000],
                [0.0000,   -0.0000,    0.1737,    0.0000,   -0.0000,    0.1008],
                [3.5418,    0.0000,    0.0000,   30.0183,    0.0000,    0.0000],
                [0.0000,    3.5418,   -0.0000,    0.0000,   30.0183,   -0.0000],
                [0.0000,   -0.0000,    0.1008,    0.0000,   -0.0000,    0.1751]])
        dx = x - x0
        dx[2] = wrap_to_pi(dx[2])
        return 0.5 * np.dot(dx, np.dot(P, dx))

    def control(self, x, x0):
        kx = 0.47
        kxdot = 4.02
        ky = 0.47
        kydot = 4.02
        # ktheta = 0.16
        ktheta = 0.16
        kw = 0.27
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

        # print(u)
        return u

class DelayCompensationController(Node):
    def __init__(self):
        super().__init__('delay_compensation_controller')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.x0 = np.zeros(6) # desired state
        self.x = np.zeros(6) # state from the robot
        self.last_msg = None
        self.model = SpacecraftModel()
        self.last_joy_stamp = None
        self.ctrl_id = 0
        self.pending_inputs = []  # list of (id, time, input) tuples
        self.ctrl_dt = 0.1

        self.control_pub = self.create_publisher(DelayWrenchControl, 'fmq/control', qos_profile)
        self.state_sub = self.create_subscription(DelayRobotState, 'fmq/state', self.state_callback, qos_profile)
        self.pub_timer = self.create_timer(self.ctrl_dt, self.publish_control)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 1)


    def state_callback(self, msg: DelayRobotState):
        self.last_msg = msg
        self.x[0] = msg.vehicle_local_position.x
        self.x[1] = -msg.vehicle_local_position.y
        # self.x[2] = 2.0 * np.arctan2(-msg.vehicle_attitude.q[3], -msg.vehicle_attitude.q[0])
        self.x[2] = -msg.vehicle_local_position.heading
        self.x[3] = msg.vehicle_local_position.vx
        self.x[4] = -msg.vehicle_local_position.vy
        self.x[5] = -msg.vehicle_angular_velocity.xyz[2]

        if len(self.pending_inputs) > 0:
            while self.pending_inputs[0][0] < msg.latest_ctrl_id:
                self.pending_inputs.pop(0)

        """
        msg has the following fields:
            msg.header.stamp # timestamp (in rclpy.Time)
            msg.latest_ctrl_id # ID of the latest control message
            msg.sec_since_latest_ctrl # seconds since the latest control message when the robot state was recorded
            msg.vehicle_angular_velocity # px4_msgs.msg.VehicleAngularVelocity
            msg.vehicle_attitude # px4_msgs.msg.VehicleAttitude
            msg.vehicle_local_position # px4_msgs.msg.VehicleLocalPosition
        """

    def publish_control(self):
        # do prediction here

        if self.last_msg is None:
            u = np.zeros(3)
        else:
            # do prediction
            worst_lyap_val = -np.inf
            x_for_control = self.x
            for i in range(10):
                x0 = self.x0.copy()
                delay_profile = [np.clip(self.ctrl_dt + random.uniform(self.model.delay_min, self.model.delay_max) - random.uniform(self.model.delay_min, self.model.delay_max), 0.0, np.inf) for _ in range(len(self.pending_inputs))]
                if len(delay_profile) > 0:
                    delay_profile[0] = np.clip(delay_profile[0], self.last_msg.sec_since_latest_ctrl, np.inf)
                
                inputs = [(delay, pending_input[2]) for delay, pending_input in zip(delay_profile, self.pending_inputs)]
                x_pred = self.model.predict(inputs, self.x)

                lyap_val = self.model.lyapunov(x_pred, x0)
                if lyap_val > worst_lyap_val:
                    x_for_control = x_pred
                    worst_lyap_val = lyap_val
            u = self.model.control(x_for_control, self.x0)

        px4_timestamp = int(self.get_clock().now().nanoseconds / 1e3)
        control_msg = DelayWrenchControl()
        control_msg.header.stamp = self.get_clock().now().to_msg()
        control_msg.header.frame_id = ''
        control_msg.id = self.ctrl_id
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

        self.control_pub.publish(control_msg)

        self.pending_inputs.append((self.ctrl_id, self.get_clock().now(), u))

        self.ctrl_id += 1

    # joystick input --> desired state
    def joy_callback(self, msg: Joy):
        t = Time.from_msg(msg.header.stamp)
        if self.last_joy_stamp is None:
            dt = 0.0
        else:
            dt = (t - self.last_joy_stamp).nanoseconds / 1e9
        self.last_joy_stamp = t

        max_vx = 0.2
        max_vy = 0.2
        max_w = 0.3
        v = np.array([-msg.axes[0] * max_vx, msg.axes[1] * max_vy, (msg.axes[5] - msg.axes[2]) * max_w / 2.0])

        self.x0[0] += v[0] * dt
        self.x0[1] += v[1] * dt
        self.x0[2] += v[2] * dt
        self.x0[3] = v[0]
        self.x0[4] = v[1]
        self.x0[5] = v[2]

        print(self.x0)

def main(args=None):
    rclpy.init(args=args)
    node = DelayCompensationController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()