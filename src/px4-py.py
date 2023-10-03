import rclpy
import numpy as np
import json
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitudeSetpoint


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')
        self.test=0
        self.procedure_time=0
        self.dt = 0.01
        self.theta = 0.0
        self.radius = 2.0
        self.omega = 2
        self.ready = 0
        self.routine=0
        self.pose=0
        self.tic=0
        self.path =0
        self.path_points=[]
        self.path_index = -1
        self.spin=0
        self.base = [0.0,0.0,-1.0]
        self.updown=0

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)


        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -1.0

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

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
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float, yaw: float=0.0, yawspeed: float=1.0):
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
        
        
        
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # (90 degree)
        msg.yaw = 0.0
        # msg.yaw = self.tic*1.0
        msg.yaw = yaw
        msg.yawspeed = yawspeed
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        #self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")
        


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

    def read_path(self):
        try:
            f = open('/home/massimo/ros2_ws/src/px4-py/src/path.json')

            # returns JSON object as
            # a dictionary
            data = json.load(f)
            
            # Iterating through the 1
            # list
            list = []
            for i in data['points on path']:
                print(i)
                list.append((i["x"],i["y"],i["z"]))
            
            # Closing file
            f.close()
            return list
        except:

            return []

    def dist(self,t1,t2):
        print("p1: ",t1)
        print("p2: ",t2)
        return math.dist(t1, t2)
    
    def vlp_to_array(self, vlp:VehicleLocalPosition):
        #print(vlp)
        return [vlp.x,vlp.y,vlp.z]
        #return [0.0,0.0,-5.0]


    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()
        self.tic+=1

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.base = [self.vehicle_local_position.x,self.vehicle_local_position.y, self.vehicle_local_position.z ]
            self.arm()
            self.ready=0
            # self.routine = 0

        #if self.vehicle_local_position.z > self.takeoff_height and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
        if not self.ready and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            
            self.publish_position_setpoint(self.vehicle_local_position.x,self.vehicle_local_position.y,self.takeoff_height)
            
            # print("z position: ",self.vehicle_local_position.z)
            if self.vehicle_local_position.z < self.takeoff_height*0.95:
                #da qui decido cosa fare
                self.ready=1
                
                
                self.routine=0  
                self.path=1
                self.spin=0
                self.updown =0

                print("start")
            
        
        # if self.ok:
        #     self.ok=0
        #     self.pose=1
        
        if self.updown:
            self.land()
            self.disarm()
        
        
        
        if self.path:
            
            if self.path_index==-2:
                self.path=0 #path vuota o error, disattivo

            if self.path_index==-1: #non acora impostato
                self.path_points=self.read_path()
                #print(self.path_points)
                if self.path_points:
                    #non vuota
                    self.path_index=0
                else:
                    self.path_index=-2
               #  if path.isem    
            if self.path_index >=0:
                print("goto; ", self.base[0] + self.path_points[self.path_index][0], self.base[1] + self.path_points[self.path_index][1], self.base[2] + self.path_points[self.path_index][2])
                self.publish_position_setpoint(self.base[0] + self.path_points[self.path_index][0], self.base[1] + self.path_points[self.path_index][1], self.base[2] + self.path_points[self.path_index][2])
                dist= self.dist([self.base[0] + self.path_points[self.path_index][0], self.base[1] + self.path_points[self.path_index][1], self.base[2] + self.path_points[self.path_index][2]] ,self.vlp_to_array(self.vehicle_local_position))
                print("dist: ",dist)
                if dist < 0.2:
                    self.path_index+=1
                    if self.path_index==len(self.path_points):
                        self.path=0
                        self.land()
                        exit(0)
        if self.spin:
            self.publish_position_setpoint(self.vehicle_local_position.x,self.vehicle_local_position.y,self.vehicle_local_position.z,self.vehicle_local_position.heading+0.01)
            print("yaw: ",self.vehicle_local_position.heading+0.05)
       			
        if self.routine:
            self.traj_x = self.radius * np.cos(self.theta)
            self.traj_y = self.radius * np.sin(self.theta)
            # self.traj_x = 0.0
            # self.traj_y = 0.0
            #self.traj_z = -5.0 +  np.sin(self.theta*1.7)
            self.traj_z = self.takeoff_height

            self.theta = self.theta + self.omega * self.dt
            
            self.publish_position_setpoint(self.traj_x, self.traj_y, self.traj_z)

            #print("fatto ", int(self.get_clock().now().nanoseconds / 1000000))
            # print("current posistion ", self.vehicle_local_position.x)
            # print("current posistion ", self.vehicle_local_position.y)
            # print("current posistion ", self.vehicle_local_position.z)
        


        # elif self.vehicle_local_position.z <= self.takeoff_height:
        #     self.land()
        #     exit(0)

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

    def routine(self):
        print("todo")
        return 0
        

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
