from atmos_fmq_msgs.msg import DelayRobotState, DelayWrenchControl
import rclpy
from rclpy.node import Node
from rclpy.qos  import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import VehicleTorqueSetpoint, VehicleThrustSetpoint, OffboardControlMode, VehicleAngularVelocity, VehicleAttitude, VehicleLocalPosition



class DelayWrapper(Node):
    def __init__(self):
        super().__init__('Delay_Siheung_Gwanak_Wrapper')


def main(args = None):
    rclpy.init(args=args)
    delaywrapper_ = DelayWrapper()
    try:
        rclpy.spin(delaywrapper_)
        delaywrapper_.destroy_node()
    except KeyboardInterrupt:
        delaywrapper_.get_logger().info('Delay Wrapper stopped')
    finally:
        delaywrapper_.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()