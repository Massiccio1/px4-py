import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import  OffboardControlMode


import logging


class Converter(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('opti_to_px4')

        self.ocm=OffboardControlMode()
        self.ready=True
        self.calls=0
        
        
        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
            
        # Create subscribers
        self.vehicle_odometry_subscriber = self.create_subscription(
            OffboardControlMode, '/fmu/in/offboard_control_mode', self.ocm_callback, qos_profile)


        self.timer = self.create_timer(1.0, self.timer_callback)

        logging.debug("init complete")

    def ocm_callback(self, msg):
        """Callback function for vehicle odometry"""
        logging.debug("hb recived")

        self.calls = self.calls + 1

    def timer_callback(self):
        print("Hz: "+str(self.calls))
        self.calls = 0


def main(args=None) -> None:
    print('Starting optitrack to px4 node...')
    rclpy.init(args=args)
    converter = Converter()
    rclpy.spin(converter)
    converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
