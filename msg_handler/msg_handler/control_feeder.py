from atmos_fmq_msgs.msg import DelayRobotState, DelayWrenchControl
import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from rclpy.qos  import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import VehicleTorqueSetpoint, VehicleThrustSetpoint, OffboardControlMode, VehicleAngularVelocity, VehicleAttitude, VehicleLocalPosition
from copy import deepcopy

class ControlFeeder(Node):
    def __init__(self):
        super().__init__('control_feeder')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        
        self._ctrl_on_ = False
        self._Vehicle_ang_vel_on_, self._Vehicle_att_on_, self._Vehicle_loc_pos_on_ = False, False, False

        self._pub_to_robot_timer_ = self.create_timer(0.01, self._PublishTopicsToRobots)
        self._pub_to_ctrl_timer_ = self.create_timer(0.1, self._PublishTopicsToCtrller)

        self._Ctrl_sub_ = self.create_subscription(DelayWrenchControl, 'fmq/control', self._CtrlUpdateCallback, qos_profile)
        
        self._DelayRobotState_msg_ = DelayRobotState()

        self._Vehicle_ang_vel_sub_ = self.create_subscription(VehicleAngularVelocity, '/fmu/out/vehicle_angular_velocity', self._VehicleAngularVelocityCallback, qos_profile)
        self._Vehicle_att_sub_ = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self._VehicleAttitudeCallback, qos_profile)
        self._Vehicle_loc_pos_sub_ = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self._VehicleLocalPositionCallback, qos_profile)
        
        self.rec_times = [] # (id, time, duration) tuples
        
        self._latest_ctrl_id = int(0)

        self._State_pub_ = self.create_publisher(DelayRobotState, 'fmq/state', qos_profile)
        
        self._latest_offboardCm_= OffboardControlMode()

        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile)
        
        self._latest_ThrustSp_= VehicleThrustSetpoint()
        self.publisher_thrust_setpoint = self.create_publisher(
            VehicleThrustSetpoint,
            '/fmu/in/vehicle_thrust_setpoint',
            qos_profile)
        
        self._latest_TorqueSp_= VehicleTorqueSetpoint()
        self.publisher_torque_setpoint = self.create_publisher(
            VehicleTorqueSetpoint,
            '/fmu/in/vehicle_torque_setpoint',
            qos_profile)
        
    def _CtrlUpdateCallback(self, msg:DelayWrenchControl):
        if self._ctrl_on_ == False:
            self._ctrl_on_ = True

        last_time = msg.state_timestamp

        self._latest_offboardCm_ = msg.offboard_control_mode
        self._latest_ThrustSp_ = msg.vehicle_thrust_setpoint
        self._latest_TorqueSp_ = msg.vehicle_torque_setpoint
        self._latest_ctrl_id = msg.id
        self._ctrl_arrival_time = self.get_clock().now()

        while True:
            if len(self.rec_times) == 0:
                break
            if self.rec_times[0][1] < Time.from_msg(last_time):
                self.rec_times.pop(0)
            else:
                break

        if len(self.rec_times) == 0:
            self.rec_times.append((msg.id, self._ctrl_arrival_time, self._ctrl_arrival_time - Time.from_msg(msg.header.stamp)))
        elif len(self.rec_times) > 0:
            if self.rec_times[-1][0] < msg.id:
                id_prev = self.rec_times[-1][0]
                for id in range(id_prev + 1, msg.id):
                    self.rec_times.append((id, self._ctrl_arrival_time, Duration(seconds=1e8))) # append big time to represent drops
                self.rec_times.append((msg.id, self._ctrl_arrival_time, self._ctrl_arrival_time - Time.from_msg(msg.header.stamp)))
    
    def _VehicleAngularVelocityCallback(self, msg:VehicleAngularVelocity):
        if not self._Vehicle_ang_vel_on_:
            self._Vehicle_ang_vel_on_ = True
        self._DelayRobotState_msg_.vehicle_angular_velocity = deepcopy(msg)

    def _VehicleAttitudeCallback(self, msg:VehicleAttitude):
        if not self._Vehicle_att_on_:
            self._Vehicle_att_on_ = True
        self._DelayRobotState_msg_.vehicle_attitude = deepcopy(msg)

    def _VehicleLocalPositionCallback(self, msg:VehicleLocalPosition):
        if not self._Vehicle_loc_pos_on_: 
            self._Vehicle_loc_pos_on_ = True
        self._DelayRobotState_msg_.vehicle_local_position = deepcopy(msg)

    def _PublishTopicsToRobots(self):
        if self._ctrl_on_:
            self.publisher_offboard_mode.publish(self._latest_offboardCm_)
            self.publisher_thrust_setpoint.publish(self._latest_ThrustSp_)
            self.publisher_torque_setpoint.publish(self._latest_TorqueSp_)

    def _PublishTopicsToCtrller(self):
        cur_time_ = self.get_clock().now()
        if self._ctrl_on_ and self._Vehicle_ang_vel_on_ and self._Vehicle_att_on_ and self._Vehicle_loc_pos_on_:
            self._DelayRobotState_msg_.header.stamp = cur_time_.to_msg()
            self._DelayRobotState_msg_.latest_ctrl_id = self._latest_ctrl_id            
            self._DelayRobotState_msg_.sec_since_latest_ctrl = (cur_time_ - self._ctrl_arrival_time).nanoseconds / 1e9

            self._DelayRobotState_msg_.transmission_delays = [dur.to_msg() for _, _, dur in self.rec_times]

            self._State_pub_.publish(self._DelayRobotState_msg_)

def main(args = None):
    rclpy.init(args=args)
    delaywrapper_ = ControlFeeder()
    try:
        rclpy.spin(delaywrapper_)
        delaywrapper_.destroy_node()
    except KeyboardInterrupt:
        delaywrapper_.get_logger().info('stopped')
    finally:
        delaywrapper_.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()

