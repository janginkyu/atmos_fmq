import rclpy

from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from rclpy.node import Node
from rclpy.time import Time, Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import numpy as np
import casadi as ca

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, TwistStamped


def zoh_dyn(x, u, dt):
    mass = 16.8
    inertia = 0.1594
    vx = x[2]
    vy = x[3]
    ax = u[0] / mass
    ay = u[1] / mass
    return x + dt * ca.vcat([vx + 0.5 * dt * ax, vy + 0.5 * dt * ay, ax, ay])


class Docking(Node):
    def __init__(self):
        super().__init__('docking_setpoints')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Get namespace names
        self.namespace = self.declare_parameter('namespace', '').value
        self.namespace=  'snap'

        self.docking_target_pose = np.array([1.0, -0.2, np.pi/2]) # x, y, yaw
        self.docking_target_vel = np.array([0.0, 0.01, 0.0]) # vx, vy, yaw_rate
        self.parking_pose = np.array([1.0, -1.0, np.pi/2]) # x, y, yaw

        self.pose_pub = self.create_publisher(PoseStamped, f'/{self.namespace}/setpoint_pose', qos_profile)
        self.twist_pub = self.create_publisher(TwistStamped, f'/{self.namespace}/setpoint_twist', qos_profile)
        self.control_on_pub = self.create_publisher(Bool, f'/{self.namespace}/control_on', qos_profile)
        self.pose_sub = self.create_subscription(PoseStamped, f'/{self.namespace}/predicted_pose', self.pose_callback, qos_profile)
        self.twist_sub = self.create_subscription(TwistStamped, f'/{self.namespace}/predicted_twist', self.twist_callback, qos_profile)

        self.mpc_timer = self.create_timer(1.0, self.compute_traj)
        self.pub_timer = self.create_timer(0.1, self.publish_setpoints)

        self.x_pred = np.zeros(6) # x, y, yaw, vx, vy, yaw_rate

        self.has_pose = False
        self.has_twist = False

        self.docking_success = False
        self.traj = None
        self.traj_start_time = None

        self.max_force = 1.5
        self.max_torque = 0.5

        # mpc settings
        self.dt_mpc = 0.5
        self.N = 20
        self.horizon = self.dt_mpc * self.N

        _5s = int(5.0 / self.dt_mpc)
        self.umin = np.tile([-self.max_force, -self.max_force], (self.N, 1))
        self.umax = np.tile([self.max_force, self.max_force], (self.N, 1))

        self.umin[-_5s:, :] = self.umin[-_5s:, :] * np.linspace(1, 0, _5s).reshape(-1, 1) ** 2
        self.umax[-_5s:, :] = self.umax[-_5s:, :] * np.linspace(1, 0, _5s).reshape(-1, 1) ** 2

        # cost weights
        self.R_seq = np.tile(np.diag([1.0, 1.0]), (self.N, 1, 1))
        ramp = np.linspace(1.0, 2.0, self.N).reshape(-1, 1, 1) ** 2
        self.R_seq = self.R_seq * ramp

        self.mpc_solver = ca.Opti()
        self.X = self.mpc_solver.variable(4, self.N + 1) # x, y, vx, vy
        self.U = self.mpc_solver.variable(2, self.N)     # ax, ay
        self.X0_param = self.mpc_solver.parameter(4)
        
        self.mpc_solver.subject_to(self.X[:, 0] == self.X0_param)
        self.mpc_solver.subject_to(self.X[:, -1] == ca.vcat([self.docking_target_pose[0], self.docking_target_pose[1], self.docking_target_vel[0], self.docking_target_vel[1]]))
        for k in range(self.N):
            x_next = zoh_dyn(self.X[:, k], self.U[:, k], self.dt_mpc)
            self.mpc_solver.subject_to(self.X[:, k + 1] == x_next)
            self.mpc_solver.subject_to(self.umin[k, :] <= self.U[:, k])
            self.mpc_solver.subject_to(self.U[:, k] <= self.umax[k, :])

        # Cost: sum u^T R_k u (only input)
        self.J = 0
        for k in range(self.N):
            uk = self.U[:, k]
            Rk = ca.DM(self.R_seq[k])
            stage = ca.mtimes([uk.T, Rk, uk])
            self.J = self.J + stage * self.dt_mpc
        self.mpc_solver.minimize(self.J)

        # solver options
        self.ipopt_options = {
            "print_level": 0, "max_iter": 2000, "tol": 1e-8,
            "dual_inf_tol": 1e-8, "constr_viol_tol": 1e-8,
            "compl_inf_tol": 1e-8, "acceptable_tol": 1e-6,
        }
        self.mpc_solver.solver("ipopt", {"print_time": False}, self.ipopt_options)

    def pose_callback(self, msg: PoseStamped):
        self.has_pose = True
        self.x_pred[0] = msg.pose.position.x
        self.x_pred[1] = msg.pose.position.y

        # yaw from quaternion
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.x_pred[2] = np.arctan2(siny_cosp, cosy_cosp)

    def twist_callback(self, msg: TwistStamped):
        self.has_twist = True
        self.x_pred[3] = msg.twist.linear.x
        self.x_pred[4] = msg.twist.linear.y
        self.x_pred[5] = msg.twist.angular.z

    def compute_traj(self):
        if not (self.has_pose and self.has_twist):
            self.traj = None
            return

        if self.traj is not None:
            return
    
        if self.docking_success:
            return

        # do not start solving mpc if too far away from parking pose
        if np.linalg.norm(self.x_pred[0:3] - np.array(self.parking_pose))**2 + np.linalg.norm(self.x_pred[4:6])**2 > 0.2**2:
            return

        # extract indices 0, 1, 3, 4 from x_pred
        x0 = np.array([self.x_pred[0], self.x_pred[1], self.x_pred[3], self.x_pred[4]])
        self.mpc_solver.set_value(self.X0_param, x0[:4])

        self.mpc_solver.set_initial(self.X, np.tile(x0, (self.N + 1, 1)).T)
        self.mpc_solver.set_initial(self.U, np.zeros((2, self.N)))

        try:
            sol = self.mpc_solver.solve()
            x_sol = sol.value(self.X).T
            self.traj = x_sol
            self.traj_start_time = self.get_clock().now()
            self.get_logger().info('Found feasible docking trajectory')
        except RuntimeError as e:
            # infeasible
            self.traj = None


    """
    TODO
    if there is no currently run trajectory:
        compute trajectory from current state to docking target
        
        if feasible:
            start executing the trajectory
        else:
            publish parking state setpoint
    
    if there is a currently running trajectory:
        if close enough to the end of the trajectory (in terms of time):
            see if applying u = [0, 0, 0] will make the robot reach the docking target
            if yes, for sure:
                no control anymore, publish control_on = False
            else:
                this means the robot is not going to reach the target with zero input, publish parking state setpoint

        else:
            recompute the trajectory from current predicted statte to the docking target with remaining time

            if feasible:
                start executing the trajectory
            else:
                publish parking state setpoint
    """
        

    def publish_setpoints(self):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.position.x = self.parking_pose[0]
        pose_msg.pose.position.y = self.parking_pose[1]
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.w = np.cos(self.parking_pose[2] / 2)
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = np.sin(self.parking_pose[2] / 2)

        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'world'
        twist_msg.twist.linear.x = 0.0
        twist_msg.twist.linear.y = 0.0
        twist_msg.twist.linear.z = 0.0
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0

        # check if docking succeeded
        if self.traj is not None:
            t_now = self.get_clock().now()
            t_elapsed = (t_now - self.traj_start_time).nanoseconds / 1.0e9
            if t_elapsed >= self.horizon:
                # reached the end of the trajectory
                self.docking_success = True
                self.traj = None
                self.traj_start_time = None

        # publish the right setpoints
        if not (self.has_pose and self.has_twist):
            self.control_on_pub.publish(Bool(data=False))
            self.pose_pub.publish(pose_msg)
            self.twist_pub.publish(twist_msg)
            return

        if self.docking_success:
            # docking succeeded, no control
            self.control_on_pub.publish(Bool(data=False))

            pose_msg.pose.position.x = self.docking_target_pose[0]
            pose_msg.pose.position.y = self.docking_target_pose[1]
            pose_msg.pose.orientation.w = np.cos(self.docking_target_pose[2] / 2)
            pose_msg.pose.orientation.z = np.sin(self.docking_target_pose[2] / 2)
            self.pose_pub.publish(pose_msg)

            twist_msg.twist.linear.x = self.docking_target_vel[0]
            twist_msg.twist.linear.y = self.docking_target_vel[1]
            twist_msg.twist.angular.z = self.docking_target_vel[2]
            self.twist_pub.publish(twist_msg)
            return

        if self.traj is None:
            # no feasible trajectory, move to parking pose
            self.control_on_pub.publish(Bool(data=True))
            self.pose_pub.publish(pose_msg)
            self.twist_pub.publish(twist_msg)
            return

        else:
            # linear interpolate the trajectory and publish the current pose.
            # attitude is the same to target attitude
            self.control_on_pub.publish(Bool(data=True))
            t_now = self.get_clock().now()
            t_elapsed = (t_now - self.traj_start_time).nanoseconds / 1.0e9

            # find the segment
            idx = int(t_elapsed / self.dt_mpc)
            if idx >= self.N:
                idx = self.N - 1
            t_segment = t_elapsed - idx * self.dt_mpc
            ratio = t_segment / self.dt_mpc

            x0 = self.traj[idx]
            x1 = self.traj[idx + 1]
            x_interp = (1 - ratio) * x0 + ratio * x1

            pose_msg.pose.position.x = x_interp[0]
            pose_msg.pose.position.y = x_interp[1]
            pose_msg.pose.orientation.w = np.cos(self.docking_target_pose[2] / 2)
            pose_msg.pose.orientation.z = np.sin(self.docking_target_pose[2] / 2)
            self.pose_pub.publish(pose_msg)

            twist_msg = TwistStamped()
            twist_msg.twist.linear.x = x_interp[2]
            twist_msg.twist.linear.y = x_interp[3]
            self.twist_pub.publish(twist_msg)

            return


def main(args=None):
    rclpy.init(args=args)
    node = Docking()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
