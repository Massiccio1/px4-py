import rclpy
import numpy as np
import json
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitudeSetpoint
from commander_msg.msg import CommanderAll, CommanderArm, CommanderMode, CommanderAction
from std_msgs.msg import Float64MultiArray


import config
import random
import logging
import threading
import time

logging.basicConfig(level=config.log)

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')
        self.procedure_time=0
        self.takeoff_height = config.takeoff_height

        self.dt = config.dt
        self.theta = config.theta
        self.radius = config.radius
        self.omega = config.omega
        self.height=self.takeoff_height
        
        self.ready = False
        self.routine=False
        self.updown=False
        self.path =False
        self.spin=False
        self.goto=False
        
        self.path_points=config.path_points
        self.pose=0
        self.tic=0        
        self.base = [0.0,0.0,-1.0] 
        self.path_index = -1
        self.mode=0
        
        
       
        
        self.spin_rad=config.spin_rad

        
        self.gui = config.gui

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
        
        self.com_pub_all = self.create_publisher(
            CommanderAll, '/com/out/all', qos_profile)
        self.com_pub_mode = self.create_publisher(
            CommanderMode, '/com/out/mode', qos_profile)

            
        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        
        self.commander_arm_sub = self.create_subscription(
            CommanderArm, '/com/in/arm', self.commander_arm_callback, qos_profile)
        self.commander_arm_sub = self.create_subscription(
            CommanderArm, '/com/in/force_disarm', self.commander_force_disarm_callback, qos_profile)
        self.commander_mode_sub = self.create_subscription(
            CommanderMode, '/com/in/mode', self.commander_mode_callback, qos_profile)
        self.commander_action_sub = self.create_subscription(
            CommanderAction, '/com/in/action', self.commander_action_callback, qos_profile)
        self.gui_setpoint_publisher = self.create_subscription(
            TrajectorySetpoint, '/com/in/trajectory_setpoint', self.commander_trajectory_setpoint_callback, qos_profile)

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
        
    def commander_arm_callback(self,msg):
        """comamnder arm"""
        logging.info("arm from ros callback")
        if msg.arm:
            self.vehicle_start()
        else:
            self.disarm()
            
    def commander_force_disarm_callback(self,msg):
        """comamnder force disarm"""
        logging.info("gforce disarm from ros callback")
        if not msg.arm:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_FLIGHTTERMINATION,
                param1=1.0
            )
        
        
    def commander_mode_callback(self, msg):
        """comamnder mode callback"""
        logging.info("ros callback for mode")
        if msg.ready == False:
            self.publish_position_setpoint(self.vehicle_local_position.x,self.vehicle_local_position.y,self.vehicle_local_position.z, self.vehicle_local_position.heading)
        self.mode = msg.mode
        self.ready=msg.ready
        self.routine = msg.routine
        self.path=msg.path
        self.spin=msg.spin
        self.updown=msg.updown
        self.height=msg.height
        #self.theta=msg.theta
        self.omega=msg.omega
        self.radius=msg.radius
        self.spin_rad=msg.spin_speed
        self.path_points=self.decode_path_points(msg.points)
        if len(self.path_points)>0:
            self.path_index=msg.path_index
            logging.info("loaded path")
        else:
            self.path_index=-2
            logging.info("path invalid  ")

    def decode_path_points(self,points):
        # logging.debug("len msg points")
        # logging.debug(len(msg.points))
        path=[]
        for p in points:
            path.append([p.points[0],p.points[1],p.points[2]])
        #return []
        return path
        
    def commander_action_callback(self, action):
        """comamnder action callback"""
        logging.info("ros callback for action")
        ops = {
            "takeoff": self.takeoff,
            "land": self.land,
            "test": self.test
        }
        operation = ops.get(action.action, self.action_error)

        
        operation()
        
    def action_error(self):
        logging.debug("you choose an invalid action")

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

        #logging.info("heartbeat sent")
             
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
    
    def commander_trajectory_setpoint_callback(self,msg):
        logging.debug("recived manual command from gui")
        logging.debug(msg.position[0])
        logging.debug(msg.position[1])
        logging.debug(msg.position[2])
        self.publish_position_setpoint(float(msg.position[0]),float(msg.position[1]),float(msg.position[2]),msg.yaw)
        #self.publish_position_setpoint(0.0,0.0,-1.0,msg.yaw)
    
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

    def vehicle_start(self):
        self.engage_offboard_mode()
        self.base = [self.vehicle_local_position.x,
                     self.vehicle_local_position.y, 
                     self.vehicle_local_position.z
                     ]
        self.arm()
        self.ready=False
    
    def goto_func(self,xyz,yaw=None,speed=None,time=None):
        print("todo goto")
        # if yaw == None:
        #     yaw = self.vehicle_local_position.heading
        if speed == None:
            speed=5
        if time == None:
            time=5
        t=threading.Thread(target=self.goto_thread,args=(xyz,yaw,speed,time))
        t.start()
            
        
    def goto_thread(self,xyz,yaw,time_t,blank=None):
        print("todo goto thread")
        delta = np.array([xyz[0]-self.vehicle_local_position.x,  
                          xyz[1]-self.vehicle_local_position.y, 
                          xyz[2]-self.vehicle_local_position.z,
                          yaw-self.vehicle_local_position.heading
                         ])
        start=np.array([
            self.vehicle_local_position.x,
            self.vehicle_local_position.y,
            self.vehicle_local_position.z,
            self.vehicle_local_position.heading
        ])
        # minT=0
        # maxT=time
        # M= np.array([1, minT, pow(minT,2), pow(minT,3),
        #         0 , 1   , 2*minT,    3*pow(minT,2),
        #         1, maxT, pow(maxT,2), pow(maxT,3),
        #         0 , 1   ,2*maxT,    3*pow(maxT,2)]).reshape(4,4)
        # A=[]
        # for i in range(4):
        #     b = np.array([0,0,delta[i],0])  #s0,v0,sf,vf
        #     a = np.linalg.pinv(M)*b
        #     #start from zero and get to delta
        #     A.append(a)
        max_step=int(time_t/self.dt)
        for i in range(int(max_step)):
            logging.info("publish goto: ")
            logging.info(i)
            factor = i/max_step
            target = start+delta*factor
            # if self.ready:
            #     logging.info("[goto thread] not ready")
            #     return 0
            self.publish_position_setpoint(
                target[0],
                target[1],
                target[2],
                target[3],
                )
            
            time.sleep(self.dt)
        logging.info("end of path")
    
    def takeoff(self):
        logging.info("taking off")
        
        self.ready = False
        
        if self.vehicle_local_position.z > self.takeoff_height: #NED frame
        
            self.publish_position_setpoint(self.vehicle_local_position.x,self.vehicle_local_position.y,self.takeoff_height,self.vehicle_local_position.heading)

            if self.vehicle_local_position.z < self.takeoff_height*0.95:
                #da qui decido cosa fare
                self.ready=True
                
                self.routine = config.routine
                self.path= config.path
                self.spin= config.spin
                self.updown = config.updown
        
        else: 
            logging.info("takeoff rejected, vehicle aleady in air")

    def dist(self,t1,t2):
        # print("p1: ",t1)
        # print("p2: ",t2)
        return math.dist(t1, t2)
    
    def vlp_to_array(self, vlp:VehicleLocalPosition):
        #print(vlp)
        return [vlp.x,vlp.y,vlp.z]
        #return [0.0,0.0,-5.0]
    def publish_commander(self):
        """published commander params for gui"""
        msg = CommanderAll()
        
        msg.dt=self.dt
        msg.theta=self.theta
        msg.radius=self.radius
        msg.omega=self.omega
        
        msg.ready=self.ready
        msg.base= self.base
        msg.takeoff_height=self.takeoff_height
        
        msg.routine=self.routine
        msg.pose=self.pose
        msg.tic=self.tic
        msg.path=self.path
        msg.spin=self.spin
        msg.updown=self.updown
        
        msg.gui = self.gui
        msg.path_index=self.path_index
        #msg.path_points=self.path_points
                
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.testint=random.randint(0,50)
        msg.testbool=False
        self.com_pub_all.publish(msg)


    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()
        self.publish_commander()
        self.tic+=1

        if not self.gui and self.offboard_setpoint_counter == 10:
            self.vehicle_start()
            # self.routine = 0

        #if self.vehicle_local_position.z > self.takeoff_height and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
        if not self.ready and not self.gui and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            #if it's landing the state will be "landing"
            # print("z position: ",self.vehicle_local_position.z)
            self.takeoff()
            
        
        # if self.ok:
        #     self.ok=0
        #     self.pose=1
        
        if self.ready and self.updown:
            self.land()
            self.disarm()
        
        
        
        if self.ready and self.path:
            
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
            if self.ready and self.path_index >=0:
                #print("goto; ", self.base[0] + self.path_points[self.path_index][0], self.base[1] + self.path_points[self.path_index][1], self.base[2] + self.path_points[self.path_index][2])
                self.publish_position_setpoint(self.base[0] + self.path_points[self.path_index][0], self.base[1] + self.path_points[self.path_index][1], self.base[2] + self.path_points[self.path_index][2])
                dist= self.dist([self.base[0] + self.path_points[self.path_index][0], self.base[1] + self.path_points[self.path_index][1], self.base[2] + self.path_points[self.path_index][2]] ,self.vlp_to_array(self.vehicle_local_position))
                print("dist: ",dist)
                if dist < 0.2:
                    self.path_index+=1
                    if self.path_index==len(self.path_points):
                        self.path=0
                        logging.info("path finished")
        if self.ready and self.spin:
            self.publish_position_setpoint(self.vehicle_local_position.x,self.vehicle_local_position.y,self.height,self.vehicle_local_position.heading+self.spin_rad)
            logging.info("yaw: ")
            logging.info(self.vehicle_local_position.heading+self.spin_rad)
       			
        if self.ready and self.routine:
            self.traj_x = self.radius * np.cos(self.theta)
            self.traj_y = self.radius * np.sin(self.theta)
            # self.traj_x = 0.0
            # self.traj_y = 0.0
            #self.traj_z = -5.0 +  np.sin(self.theta*1.7)
            self.traj_z = self.height

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
    
    def test(self):
        target =np.array([
            self.vehicle_local_position.x+random.randint(-5,5),
            self.vehicle_local_position.y+random.randint(-5,5),
            self.vehicle_local_position.z-random.randint(-10,10)/10
        ])
        self.goto_func(target)
        

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
