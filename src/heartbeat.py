import rclpy
import numpy as np
import json
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import TimesyncStatus, OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitudeSetpoint


import config
import random
import logging
import threading
import time


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')
        
        self.dt=0.1
        
        self.delta_time=0
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        
        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
            
        # Create subscribers
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_timesync = self.create_subscription(
            TimesyncStatus, '/fmu/out/timesync_status', self.vehicle_timesync_callback, qos_profile)
        
        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
    

        # Create a timer to publish control commands
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
        logging.debug("init complete")

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position
        
    def vehicle_timesync_callback(self, msg):
        """Callback function for vehicle_local_position topic subscriber."""
        self.delta_time=msg.timestamp-int(self.get_clock().now().nanoseconds / 1000)
        print("delta updated to: ",self.delta_time)

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0) #6 per offboard mode
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.ready=False
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        
            
    def commander_force_disarm_callback(self,msg):
        """comamnder force disarm"""
        logging.info("gforce disarm from ros callback")
        if not msg.arm:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_FLIGHTTERMINATION,
                param1=1.0
            )
        
        

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        #msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000) + self.delta_time
        #msg.timestamp = 0
        print("heartbeat sent with hb: ",msg.timestamp)

        self.offboard_control_mode_publisher.publish(msg)

        logging.info("heartbeat sent")
             
    def publish_position_setpoint(self, x: float, y: float, z: float, yaw=None):
        """Publish the trajectory setpoint."""
        
        """
        # NED local world frame
        float32[3] position # in meters
        float32[3] velocity # in meters/second
        float32[3] acceleration # in meters/second^2
        float32[3] jerk # in meters/second^3 (for logging only)

        float32 yaw # euler angle of desired attitude in radians -PI..+PI
        float32 yawspeed # angular velocity around NED frame z-axis in radians/second
        """
        if yaw==None:
            yaw = self.find_yaw(x,y,self.vehicle_local_position.x,self.vehicle_local_position.y)
        
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # (90 degree)
        msg.yaw = 0.0
        # msg.yaw = self.tic*1.0
        msg.yaw = yaw
        #msg.yawspeed = yawspeed
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        #self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")
    
    
    def find_yaw(self,x1,y1,x2,y2):
        return -np.arctan2(x2-x1,y2-y1)-np.pi/2

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)


    
    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.vehicle_start()
            # self.routine = 0

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

    def vehicle_start(self):
        self.engage_offboard_mode()
        self.base = [self.vehicle_local_position.x,
                     self.vehicle_local_position.y, 
                     self.vehicle_local_position.z
                     ]
        self.arm()



def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
