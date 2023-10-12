#!/usr/bin/env python
"""
    Demo - Realtime Buttons

    Realtime buttons provide a way for you to get a continuous stream of button
    events for as long as a button is held down.

    This demo is using a timeout to determine that a button has been released.
    If your application doesn't care when a button is released and only needs to know
    that it's being held down, then  you can remove the timeout on the window read call.

    Note that your reaction latency will be the same as your timeout value.  In this demo
    the timeout is 100, so there will be 100ms between releasing a button and your program detecting
    this has happened.

    Copyright 2021 PySimpleGUI
"""

import PySimpleGUI as sg


import rclpy
import numpy as np
import json
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode,BatteryStatus, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition,FailsafeFlags, VehicleStatus, VehicleAttitudeSetpoint, SensorGps
from commander_msg.msg import CommanderAll, CommanderPathPoint ,CommanderArm, CommanderMode, CommanderAction

from random import randint
import logging
import threading, _thread

logging.basicConfig(level=logging.DEBUG)
GRAPH_SIZE = (400,200)
GRAPH_STEP_SIZE = 5


class GUI(Node):

    def __init__(self) -> None:
        
        super().__init__('commander_gui')
    # The Quit button is being placed in the bottom right corner and the colors are inverted, just for fun
    
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.offboard_control_mode_publisher = self.create_subscription(
            OffboardControlMode, '/fmu/in/offboard_control_mode', self.commander_heartbeat, qos_profile)
        self.gps_subscriber = self.create_subscription(
            SensorGps, '/fmu/out/vehicle_gps_position', self.gps_callback, qos_profile)
        self.failsafe_subscriber = self.create_subscription(
            FailsafeFlags, '/fmu/out/failsafe_flags', self.failsafe_callback, qos_profile)
        self.failsafe_subscriber = self.create_subscription(
            BatteryStatus, '/fmu/out/battery_status', self.battery_callback, qos_profile)
        self.setpoint_sub = self.create_subscription(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            self.trajectory_setpoint_callback,
            qos_profile)

        self.commander_arm_pub= self.create_publisher(
            CommanderArm, '/com/in/arm', qos_profile)
        self.commander_mode_pub= self.create_publisher(
            CommanderMode, '/com/in/mode', qos_profile)
        self.commander_action_pub= self.create_publisher(
            CommanderAction, '/com/in/action', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/com/in/trajectory_setpoint', qos_profile)
        
        
        self.exit=False
        self.last_heartbeat=0
        self.delta_hr=100000#alto per averlo staccato all'inizio
        self.last_pad=0#per il pempo tra trasmisioni
        
        # #busy loop
        # while True:
        #     adad=0
        
        class slider:
            max_range=np.pi
            min_range=-np.pi
            max_radius = 20
            max_omega = np.pi
            default = 0.0
            resolution=0.01
            orientation='h'
            size=(15,10)

        
        slid = slider()
        
        #ros topic list

        pad_1=[
            [sg.Text('Forward, Back, Move Left, Move Right')],
            [sg.Text("\nspeed"),sg.Slider((0,3), key="pad_1_speed",default_value=0.2, resolution=slid.resolution, orientation=slid.orientation, s=slid.size)],
            [sg.Text()],
            [sg.Text('           '),sg.RealtimeButton(sg.SYMBOL_UP, key='pad_forward'),sg.Text('           ')],
            [sg.RealtimeButton(sg.SYMBOL_LEFT, key='pad_left'),sg.Text('           '),sg.RealtimeButton(sg.SYMBOL_RIGHT, key='pad_right')],
            [sg.Text('           '),sg.RealtimeButton(sg.SYMBOL_DOWN, key='pad_back')],
            [sg.Text()]
        ]
        
        pad_2=[
            [sg.Text('Up, Down Turn Left, Turn Right')],
            [sg.Text("\nspeed"),sg.Slider((0,3), key="pad_2_speed",default_value=0.2, resolution=slid.resolution, orientation=slid.orientation, s=slid.size)],
            [sg.Text()],
            [sg.Text('           '),sg.RealtimeButton(sg.SYMBOL_UP, key='pad_up'),sg.Text('           ')],
            [sg.RealtimeButton(sg.SYMBOL_LEFT, key='pad_turn_left'),sg.Text('           '),sg.RealtimeButton(sg.SYMBOL_RIGHT, key='pad_turn_right')],
            [sg.Text('           '),sg.RealtimeButton(sg.SYMBOL_DOWN, key='pad_down')],
            [sg.Text()]
        ]
        
        essential=[
                [sg.StatusBar("drone: UNKNOWN", key="arm_text"),sg.StatusBar("commander: DISCONNECTED", key="commander_status")],
                [sg.Button('ARM', key="arm_button"),sg.Button('DISARM', key="disarm_button")],
                [sg.Button('TAKEOFF', key="takeoff_button"),sg.Button('LAND', key="land_button")]
        ]
        
        stop=[
            [sg.StatusBar('failsafe status: UNKNOWN', key="failsafe_text")],
            [sg.Button("STOP", key="btn_stop",size=(5,5))]
        ]
        
        mode = [
            [sg.Radio('routine', 1, key= "rd_routine"),sg.Radio('path',  1, key = "rd_path"),sg.Radio('spin', 1, key = "rd_spin"),sg.Radio('updown', 1, key = "rd_updown"),sg.Radio('None', 1, key="rd_none",default=True)],
            [sg.Button('Confirm', key="mode_button")],
            [sg.Text("\nheight"),sg.Slider((1.0,20.0), key="height",default_value=2.0, resolution=slid.resolution, orientation=slid.orientation, s=slid.size),
             sg.Text("\nspin speed"),sg.Slider((0,slid.max_range), key="spin_speed",default_value=1.0, resolution=slid.resolution, orientation=slid.orientation, s=slid.size)],
            [sg.Text("\nradius"),sg.Slider((0,slid.max_radius), key="radius",default_value=2.0, resolution=slid.resolution, orientation=slid.orientation, s=slid.size),
            sg.Text("\nomega"),sg.Slider((0,slid.max_omega), key="omega",default_value=1.0, resolution=slid.resolution, orientation=slid.orientation, s=slid.size)],
            [
                sg.Text("Chose a path file"),
                sg.Input(key='file_input'),
                sg.FileBrowse(file_types=(("JSON Files", "*.json"), ("ALL Files", "*.*"),),key="file_browser"),
                sg.Button("Load", key="file_load")
            ],
            [sg.Table([], ['X','Y','Z'], vertical_scroll_only=True,expand_x=True,expand_y=False,key="table_path",num_rows=10,visible=False)]
        ]
        
        test = [
            [sg.Text('My one-shot window.')],      
            [sg.InputText()],      
            [sg.Submit(), sg.Cancel()]
        ] 
        
        tab_lp = [
            [sg.Text('Local position will be displayed here', key="tab_lp")]
        ]
        tab_gps = [
            [sg.Text('GPS info will be displayed here', key="tab_gps")]
        ]
        tab_status = [
            [sg.Text('Vehicle status will be displayed here', key="tab_status")]
        ]
        tab_battery = [
            [sg.Text('Vehicle battery info will be displayed here', key="tab_battery")]
        ]
                
        pad_frame=[
            [sg.Text("\nmax taransmission rate [Hz]"),sg.Slider((1,100), key="pad_0_hz",default_value=10, resolution=1, orientation=slid.orientation, s=slid.size)],
            [sg.Frame("",pad_1),sg.Frame("",pad_2)]
        ]
        
        battery_layout=[
            [sg.Text("Battery")],
            [sg.ProgressBar(100, orientation='v',expand_x=True,s=(10,20), k='bar_battery')]
        ]

        graph_layout = [  
            [sg.Text('Speed:', size=(20,1)),
             sg.Slider((1,30), default_value=GRAPH_STEP_SIZE, orientation='h', key='graph_speed')],
            [sg.Text('Scale:', size=(20,1)),
             sg.Slider((1,200), default_value=GRAPH_STEP_SIZE, orientation='h', key='graph_scale')],
            [sg.Graph(GRAPH_SIZE, (0,0), GRAPH_SIZE, key='graph_1', background_color='lightblue'),],
            [sg.Graph(GRAPH_SIZE, (0,0), GRAPH_SIZE, key='graph_2', background_color='lightblue'),],
        ]
        
        tabs_l = sg.TabGroup([[sg.Tab('local position',tab_lp), sg.Tab('gps', tab_gps), sg.Tab('battery', tab_battery), sg.Tab("error graph", graph_layout),sg.Tab('status', tab_status)]],expand_y=True)
        tabs_r = sg.TabGroup([[sg.Tab('Auto',mode), sg.Tab('Manual', pad_frame)]])
        #right_frame= sg.Frame('Commands', [sg.Frame("core", essential), sg.Frame("Mode",mode)]    )
        right_layout= [
            [sg.Frame("", essential),sg.Frame("", stop),sg.Frame("", battery_layout)],
            [sg.Frame("",[[tabs_r]])],
            
        ]
        right_frame= sg.Frame('Commands', right_layout    )
        
        layout = [
                [tabs_l,right_frame]
            ]

        # window = sg.Window('Window Title', layout)    

        # event, values = window.read()    
        # window.close()
        
        # exit(0)

        #self.window = sg.Window('Commander GUI', sg.Frame('controller',essential))
        self.window = sg.Window('Commander GUI', layout)

        t=threading.Thread(target=self.window_thread)
        t.start()
        logging.info("thread for window started")

    def window_thread(self):
        
        x_graph = lastx = lasty = 0
        while True:
            
            # This is the code that reads and updates your window
            event, values = self.window.read(timeout=100)
            
            if event in (sg.WIN_CLOSED, 'Quit'):
                break
            
            #redraw graphs
            step_size = values['graph_speed']
            scale = values['graph_scale']
            y_graph=self.pose_error()*scale/100
            #y = randint(0,GRAPH_SIZE[1])*scale/100        # get random point for graph
            if x_graph < GRAPH_SIZE[0]:               # if still drawing initial width of graph
                self.window['graph_1'].DrawLine((lastx, lasty), (x_graph, y_graph), width=1)
            else:                               # finished drawing full graph width so move each time to make room
                self.window['graph_1'].Move(-step_size, 0)
                self.window['graph_1'].DrawLine((lastx, lasty), (x_graph, y_graph), width=1)
                self.window['graph_2'].Move(-step_size, 0)
                self.window['graph_2'].DrawLine((lastx, lasty), (x_graph, y_graph), width=1)
                x_graph -= step_size
            lastx, lasty = x_graph, y_graph
            x_graph += step_size
            
            
            ##rip python3.8
            #match event:
            
                # case sg.TIMEOUT_EVENT:
                #     logging.debug("event: ")
                #     logging.debug(event)
                #     break
                
            
                # elif event != sg.TIMEOUT_EVENT:
                #     # if not a timeout event, then it's a button that's being held down
                #     self.window['-STATUS-'].update(event)
            if event== sg.TIMEOUT_EVENT:
                dict_arm = {
                    0:"drone: UNKNOWN",
                    1:"drone: DISARMED",
                    2:"drone: ARMED"
                }
                dict_arm_color={
                    0: "gray",
                    1: "red",
                    2: "green"
                }
                dict_failsafe={
                    False:"failsafe: FALSE",
                    True: "failsafe: TRUE"
                }
                dict_color_failsafe={
                    False:"green",
                    True: "red"
                }
                
                
                com_stat="commander: DISCONNECTED"
                color_stat= "red"
                now = self.now()
                diff = (now-self.last_heartbeat)//1000#milliseconds
                
                # logging.info("new elta hr\n-------------------")
                # logging.info(self.last_heartbeat)
                # logging.info(now)
                # logging.info(diff)
                # logging.info(self.delta_hr)
                # logging.debug("difference")
                # logging.debug(diff)
                if diff<2000:#1.5 secondi arbitrari
                    com_stat="commander: CONNECTED"
                    color_stat = "green"
                try:
                    self.window["commander_status"].update(com_stat)
                    self.window["commander_status"].update(background_color=color_stat)
                    self.window["arm_text"].update(dict_arm[self.vehicle_status.arming_state])
                    self.window["arm_text"].update(background_color=dict_arm_color[self.vehicle_status.arming_state])
                    self.window["failsafe_text"].update(dict_failsafe[self.vehicle_status.failsafe])
                    self.window["failsafe_text"].update(background_color=dict_color_failsafe[self.vehicle_status.failsafe])
                    self.window["tab_lp"].update(self.parse_vlp())
                    self.window["tab_status"].update(self.parse_status())
                    self.window["tab_battery"].update(self.parse_battery())
                    self.window["bar_battery"].update(current_count=self.parse_battery_level())
                    self.window["tab_gps"].update(self.parse_gps())

                except:
                    #logging.error("no vehicle status yet")
                    asdasd=1
                
        
            elif event== "arm_button":
                logging.info("arm button pressed")
                self.arm()
                
            
            elif event== "disarm_button":
                logging.info("disarm button pressed")
                self.disarm()
                
                
            elif event== "disarm_button":
                logging.info("disarm button pressed")
                self.disarm()
                
            
            elif event== "takeoff_button":
                logging.info("takeoff button pressed")
                self.action("takeoff")
                
            
            elif event== "land_button":
                logging.info("land button pressed")
                self.action("land")
                
            
            elif event== "mode_button":
                logging.info("confirm mode button pressed with value: ",values)
                self.changeMode(values)
                
            
            elif event== "mode_button":
                logging.info("confirm mode button pressed with value: ",values)
                self.changeMode(values)
                
            elif event== "btn_stop":
                self.stop()
            
            #
            elif "pad" in event:
                #logging.info("pad pressed")
                self.manual(event,values)
                
            elif event== "file_load":
                path = self.load_path(values)
                if path:
                    #self.window["table_path"].update(num_rows=len(path))
                    self.window["table_path"].update(visible=True)
                    self.window["table_path"].update(values=path)
                    
            else:
                # A timeout signals that all buttons have been released so clear the status display
                logging.debug(event)
                
            

        logging.info("window closing")
        self.window.close()
        self.exit=True
        _thread.interrupt_main()
        exit(0)
    
    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        #logging.debug("recived vehicle position")
        #logging.debug(vehicle_local_position)
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        #logging.debug("recived vehicle status")

        self.vehicle_status = vehicle_status
    
    def gps_callback(self,msg):
        self.gps=msg
        
    def failsafe_callback(self,msg):
        self.failsafe=msg
        
    def battery_callback(self,msg):
        self.battery=msg
    
    def trajectory_setpoint_callback(self, msg):
        self.setpoint_position = msg.position
    
    def find_yaw(self,x1,y1,x2,y2):
        return -np.arctan2(x2-x1,y2-y1)-np.pi/2
    
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
            #calculate relative direction to turn
            yaw = self.find_yaw(x,y,self.vehicle_local_position.x,self.vehicle_local_position.y)
        
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        # msg.yaw = 1.57079  # (90 degree)
        # msg.yaw = 0.0
        # msg.yaw = self.tic*1.0
        msg.yaw = yaw
        #msg.yawspeed = yawspeed
        msg.timestamp = self.now()
        self.trajectory_setpoint_publisher.publish(msg)
        #self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")
    
    def commander_heartbeat(self,msg):
        new_time = msg.timestamp
        current = self.now()
        #print("new time:", current)
        self.delta_hr=current - new_time
        self.last_heartbeat = new_time
        #print("last hr: ", self.last_heartbeat)
        #logging.debug("hr delta")
        #logging.debug(self.delta_hr)
        #1 secondo arbitrario

    def arm(self):
        msg = CommanderArm()
        msg.timestamp = self.now()
        msg.arm=True
        try:
            self.commander_arm_pub.publish(msg)
        except Exception as e:
            print(e)
        logging.info("sent arm command")
        
    def disarm(self):
        msg = CommanderArm()
        msg.timestamp = self.now()
        msg.arm=False
        self.commander_arm_pub.publish(msg)
        logging.info("sent disarm command")
        
    def changeMode(self, values):
        msg = CommanderMode()
        msg.timestamp = self.now()
        
        msg.ready=True
        if values["rd_none"]:
            msg.ready=False
        
        msg.routine=values["rd_routine"]
        msg.path=values["rd_path"]
        msg.spin=values["rd_spin"]
        msg.updown=values["rd_updown"]
        
        #msg.theta = values["theta"]
        msg.radius = values["radius"]
        msg.omega = values["omega"]
        msg.spin_speed = values["spin_speed"]
        msg.height=-values["height"]

        msg.points=self.path_to_ros2(self.window["table_path"].Values).points
        msg.path_index=0
        
        self.commander_mode_pub.publish(msg)
        logging.info("sent change mode command")

    def path_to_ros2(self,lol):
        msg = CommanderMode()
        msg.points=[]
        for i in lol:
            p=CommanderPathPoint()
            p.points=i
            msg.points.append(p)
        return msg

    def action(self, action):
        msg = CommanderAction()
        msg.timestamp = self.now()
        msg.action = action
        self.commander_action_pub.publish(msg)
    
    def stop(self):
        msg = CommanderMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        msg.ready=False
        
        msg.routine=False
        msg.path=False
        msg.spin=False
        msg.updown=False
        #msg.theta = values["theta"]
        msg.radius = 0.0
        msg.omega = 0.0
        msg.spin_speed = 0.0
        
        self.commander_mode_pub.publish(msg)
    
    def manual(self, action, values):
        """manual controll with pad"""
        #check for pad max rate
        maxrate=values["pad_0_hz"]
        now = self.now()
        nextFrame=int(self.last_pad+1000000/maxrate)#timestamp in nanoseconds for next frame 
        if now<nextFrame:
            #need to wait more
            # logging.debug("transmission too fast, dropped command")
            # logging.debug(nextFrame/1000000)
            # logging.debug(now/1000000)
            return 0
        self.last_pad=now
        #world frame
        x1=self.vehicle_local_position.x
        y1=self.vehicle_local_position.y
        z1=self.vehicle_local_position.z
        yaw1=self.vehicle_local_position.heading
        #0 degrees is nord, 90 west ...
        
        #drone frame
        dx=0
        dy=0
        dz=0
        dyaw=0
        
        speed_1=values["pad_1_speed"]
        speed_2=values["pad_2_speed"]
        
        #first I set which values change with -1,0,1, then I multiply to get the real delta
        ##rip python3.8
        #match action:
        
        if action== "pad_forward":
            dx=1
        if action== "pad_back":
            dx=-1
        if action== "pad_left":
            dy=-1
        if action== "pad_right":
            dy=1
        if action== "pad_up":
            #drone frame
            dz=-1
        if action== "pad_down":
            #drone frame
            dz=1
        if action== "pad_turn_left":
            dyaw=-1
        if action=="pad_turn_right":
            dyaw=1
        
        dx*=speed_1
        dy*=speed_1
        dz*=speed_2
        dyaw*=speed_2
        
        
        print(f"\ndx {dx}\ndy {dy}")
        
        
        x2 = x1 + np.cos(yaw1)*dx + np.sin(-yaw1)*dy
        y2 = y1 + np.sin(yaw1)*dx + np.cos(-yaw1)*dy
        z2 = z1 + dz
        yaw2 = yaw1 + dyaw
        yaw2 = (yaw2 + np.pi) % (2 * np.pi) - np.pi #-pi;+pi

        print("yaw: ",yaw2)
        print(f"\nx1 {x1}\ny1 {y1}n")
        print(f"x2 {x2}\ny2 {y2}")

        
        
        # logging.debug("yaws")
        # logging.debug(yaw1)
        # logging.debug(yaw2)
        
        self.publish_position_setpoint(float(x2),float(y2),float(z2),float(yaw2))

    def load_path(self,values):
        try:
            f = open(values["file_input"])
            
            data = json.load(f)

            list = []
            for i in data['points on path']:
                print(i)
                list.append([i["x"],i["y"],i["z"]])
            
            # Closing file
            f.close()
            return list
        except:
            return []

    def now(self):
        return int(self.get_clock().now().nanoseconds / 1000)
    
    def parse_vlp(self):
        vlp=self.vehicle_local_position
        txt=""
        time="timestamp: "+ str(vlp.timestamp) + "\n"
        pos=f'\nPOSITION\nx: {vlp.x:.3f} \ny:  {vlp.y:.3f} \nz:  {vlp.z:.3f}\n'
        vel=f"\nVELOCITY\nvx: {vlp.vx:.3f}\nvy: {vlp.vy:.3f} \nvz: {vlp.vz:.3f}\n"
        acc=f"\nACCELERATION\nax: {vlp.ax:.3f} \nay: {vlp.ay:.3f} \naz:  {vlp.az:.3f} \n"
        dir=f"\nDIRECTION\nyaw: {vlp.heading:.3f} \n"

        return time+pos+vel+acc+dir

    def parse_gps(self):
        """uint64 timestamp		# time since system start (microseconds)
        uint64 timestamp_sample

        uint32 device_id                # unique device ID for the sensor that does not change between power cycles

        float64 latitude_deg		# Latitude in degrees, allows centimeter level RTK precision
        float64 longitude_deg		# Longitude in degrees, allows centimeter level RTK precision
        float64 altitude_msl_m		# Altitude above MSL, meters
        float64 altitude_ellipsoid_m	# Altitude above Ellipsoid, meters
        """
        gps=self.gps
        
        time="timestamp: "+ str(gps.timestamp) + "\n"
        pos=f'\nPOSITION\nlatitude: {gps.latitude_deg:.9f} \nlongitude:  {gps.longitude_deg:.9f} \naltitude msl:  {gps.altitude_msl_m:.2f}\naltitude ellipsoid:  {gps.altitude_ellipsoid_m:.2f}\n'
        misc1=f"\nMISC\nvelocity: {gps.vel_m_s:.2f}\nheading: {gps.heading}\noise {gps.noise_per_ms}\n"
        misc2=f"\njamming state: {gps.jamming_state}\nsatellite used: {gps.satellites_used}\n"
        return time+pos+misc1+misc2
    
    def parse_status(self):
        st=self.vehicle_status
        fs = self.failsafe
        time="timestamp: "+ str(st.timestamp) + "\n"
        armed_state={
            1:"DISARMED",
            2:"ARMED"
        }
        dict_arm_reason={
            0: "transistion to standby",
            1: "rc stick input" ,
            2: "rc switch" ,
            3: "internal command" ,
            4: "external command" ,
            5: "mission start" ,
            6: "safety button" ,
            7: "auto disarm on land" ,
            8: "auto disarm preflight" ,
            9: "kill switch" ,
            10: "lockdown" ,
            11: "failure detector" ,
            12: "shutdown" ,
            13: "unit testing"
            
        }
        dict_nav_state={
            0:"manual",
            1:"altitude controll mode",
            2:"position control mode",
            3:"auto mission mode",
            4:"auto loiter mode",
            5:"auto return mode",
            10:"acro mode",
            12:"descend mode",
            13:"terminator mode",
            14:"offboard control",
            15:"stabilized mode",
            17:"takeoff",
            18:"landing",
            19:"auto follow",
            20:"precision land",
            21:"orbit",
            22:"vtol takeoff"
        }
        time="timestamp: "+ str(st.timestamp) + "\n"
        arm=f"\nARMING STATE\narm status: {armed_state[st.arming_state]}\nlatest arming reason: {dict_arm_reason[st.latest_arming_reason]}\nlatest diarming reason: {dict_arm_reason[st.latest_disarming_reason]}\n"
        nav=f"\nNAVIGATION\nnavigation state: {dict_nav_state[st.nav_state]}\n"
        failure=f"\nFAILURE\nfailure detector: {st.failure_detector_status}\nfailsafe: {st.failsafe}\npreflight check pass: {st.pre_flight_checks_pass}\n"
        fs1=f"\noffboard signal lost: {fs.offboard_control_signal_lost}\nmanual control signal lost: {fs.manual_control_signal_lost}"
        fs2=f"\nmission failure: {fs.mission_failure}\nlow local position accuracy: {fs.local_position_accuracy_low}"
        fs3=f"\nesc arming failure: {fs.fd_esc_arming_failure}\nimbalanced props {fs.fd_imbalanced_prop}\nmotor failure: {fs.fd_motor_failure}"
        fs4=f"\ncritical failure: {fs.fd_critical_failure}"
        return time+arm+nav+failure+fs1+fs2+fs3+fs4
        
    def parse_battery(self):
        bt=self.battery
        
        time="timestamp: "+ str(bt.timestamp) + "\n"
        volt=f"\nVOLTAGE\nvoltage: {bt.voltage_v:.3f} V\n"
        amp=f"\nCURRENT\ncurrent: {bt.current_a:.3f}\ncurrent average: {bt.current_average_a:.3f} A\n"
        charge=f"\nCHARGE\nremaining: {(bt.remaining*100):.3f} %\ndischarged {bt.discharged_mah:.3f} mah\n time remaining: {(bt.time_remaining_s/60):.1f} minutes\n"
        info=f"\nINFO\ntemperature: {bt.temperature:.3f}"
        return time+volt+amp+charge+info
        
    def parse_battery_level(self):
        minV=13.2
        maxV=16.7
        bt=self.battery
        perc=bt.voltage_v*minV/maxV
        #return int(perc)
        return int(bt.remaining*100)
    
    def pose_error(self):
        try:
            x1=self.vehicle_local_position.x
            y1=self.vehicle_local_position.y
            z1=self.vehicle_local_position.z
            
            x2=self.setpoint_position[0]
            y2=self.setpoint_position[1]
            z2=self.setpoint_position[2]
        
            dist = math.dist([x1,y1,z1],self.setpoint_position)
            logging.debug("dist")
            logging.debug(dist)
            return dist
        except:
            return 0
        
def main(args=None) -> None:
    print('Starting Gui for commander')
    rclpy.init(args=args)
    gui = GUI()
    rclpy.spin(gui)
    gui.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
