import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition
from rclpy import qos




class Converter(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('publisher_of_vlp')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.vlpp= self.create_publisher(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', qos_profile)


        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vlp = VehicleLocalPosition()

        self.timer = self.create_timer(0.1, self.timer_callback)

         


    def publish_vlp(self):
        """Publish the mocap odometry"""
        
        print("publishing mocap")

        self.vlp.timestamp+=1

        self.vlpp.publish(self.vlp)
        

    def timer_callback(self) -> None:
        self.publish_vlp()
            
    def status_callback(self) -> None:
        print("---------------------")



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