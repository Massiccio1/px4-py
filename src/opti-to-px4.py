import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry, OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitudeSetpoint
from geometry_msgs.msg import PoseStamped
from rclpy import qos


import config
import random
import logging

logging.basicConfig(level=config.log)

class Converter(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('opti_to_px4')

        self.odo=VehicleOdometry()
        self.pose=PoseStamped()
        self.ready=True
        
        self.pubs=0
        self.sub_odo=0
        self.sub_pose=0
        
        self.dt=config.opti_to_px4_dt
        
        
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
        self.optitrack_subscriber= self.create_subscription(
            PoseStamped, '/optiTrack/pose', self.optitrack_pose_callback, qos.qos_profile_sensor_data)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        

        # Create a timer to publish control commands
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
        self.timer_status = self.create_timer(1.0, self.status_callback)
        
        logging.debug("init complete")

    def vehicle_odometry_callback(self, msg):
        """Callback function for vehicle odometry"""
        logging.debug("recived odometry")

        self.odo = msg
        self.sub_odo = self.sub_odo + 1


    def optitrack_pose_callback(self, msg):
        """Callback function for optitrack pose"""
        logging.debug("recived pose")
        self.pose = msg
        self.sub_pose = self.sub_pose + 1



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
        if self.ready:
            self.publish_mocap_odometry()
            
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
