
"""
odometry subscriber
"""


import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry

import time





class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('odo_relay')
        
        self.count = 0
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.odometry_publisher= self.create_publisher(
            VehicleOdometry, '/fmu/in/vehicle_mocap_odometry', qos_profile)
        self.v_odometry_publisher= self.create_publisher(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry', qos_profile)
        # Create subscribers
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        

    
    def vehicle_odometry_callback(self, msg):
        
        new_msg = VehicleOdometry()     
           
        new_msg.timestamp = self.now()
        new_msg.position=msg.position
        new_msg.q = msg.q
        new_msg.pose_frame = msg.pose_frame
        
        
        self.odometry_publisher.publish(msg)
        self.v_odometry_publisher.publish(msg)
        
        print(f"relayed message {self.count}")
        self.count+=1
        
    def now(self):
        return int(self.get_clock().now().nanoseconds / 1000)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()