import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry, OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitudeSetpoint
from geometry_msgs.msg import PoseStamped
from rclpy import qos

import numpy as np
import config
import random
import logging

logging.basicConfig(level=config.log)

class Converter(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('opti_to_px4')

        self.odo=VehicleOdometry()
        self.ready=True
        
        self.pubs=0
        self.sub_odo=0
        self.sub_pose=0
        
        
        # Configure QoS profile for publishing and subscribing
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

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        
        
        logging.debug("init complete")

    def vehicle_odometry_callback(self, msg):
        """Callback function for vehicle odometry"""
        logging.debug("recived odometry")
        
        new_msg=VehicleOdometry()
        new_msg.position=msg.position
        new_msg.q=msg.q
        new_msg.velocity=msg.velocity
        new_msg.angular_velocity=msg.angular_velocity
        
        #static
        # new_msg=VehicleOdometry()
        # new_msg.position=[0.0,0.0,0.0]
        # new_msg.q=[0.0,0.0,0.0,0.0]
        # new_msg.velocity=[0.0,0.0,0.0]
        # new_msg.angular_velocity=[0.0,0.0,0.0]
        
        
        self.odo = msg
        self.sub_odo = self.sub_odo + 1
        
        #self.odo.velocity_variance=[np.Nan,np.Nan,np.Nan]
        
        #self.odometry_publisher.publish(new_msg)
        #self.v_odometry_publisher.publish(new_msg)
        self.v_odometry_publisher.publish(self.odo)


    def optitrack_pose_callback(self, msg):
        """Callback function for optitrack pose"""
        logging.debug("recived pose")
        self.pose = msg
        self.sub_pose = self.sub_pose + 1



            
    def status_callback(self) -> None:
        logging.info("---------------------")
        logging.info("odometry msg recived: "+ str(self.sub_odo))
        logging.info("pose msg recived: "+ str(self.sub_pose))
        logging.info("odometry msg published: "+ str(self.pubs))
        self.sub_odo=0
        self.sub_pose=0
        self.pubs=0


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
