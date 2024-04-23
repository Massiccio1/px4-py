import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry, OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitudeSetpoint
from geometry_msgs.msg import PoseStamped
from mocap_interfaces.msg import RigidBodies, RigidBody
from rclpy import qos


import config
import random
import logging

logging.basicConfig(level=config.log)

class Converter(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('test_msgs')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
            
        # Create subscribers
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.optitrack_subscriber= self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.vlp, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
                
        logging.debug("init complete")

    def vehicle_odometry_callback(self, msg):
        """Callback function for vehicle odometry"""
        logging.debug("recived odometry")


    def vlp(self, msg):
        """Callback function for optitrack pose"""
        logging.debug("recived vlp")
        print(msg)


    def publish_mocap_odometry(self):
        """Publish the mocap odometry"""
        
        logging.debug("publishing mocap")
        
        msg = VehicleOdometry()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.timestamp_sample = int(self.get_clock().now().nanoseconds / 1000)
        
        xyz = self.pose.pose.position
        p = [xyz.x, xyz.z, -xyz.y] #z with - for NED
        
        qt=self.pose.pose.orientation
        #qt=self.pose.pose.orientation
        
        q=[
            qt.w,
            qt.x,
            qt.z,
            -qt.y
        ]
        
        msg.pose_frame=1 #NED frame
        msg.velocity_frame=1 #NED frame
        msg.position=p
        msg.q=q
        # msg.velocity = self.odo.velocity
        # msg.angular_velocity = self.odo.angular_velocity
        # msg.position_variance = self.odo.position_variance
        # msg.orientation_variance = self.odo.orientation_variance
        # msg.velocity_variance = self.odo.velocity_variance
        msg.quality = 1


        self.odometry_publisher.publish(msg)
        self.v_odometry_publisher.publish(msg)
        
        self.pubs = self.pubs + 1

    def timer_callback(self) -> None:
        pass
            
    def status_callback(self) -> None:
        logging.info("---------------------")



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