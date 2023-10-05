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
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitudeSetpoint	
from commander_msg.msg import CommanderAll, CommanderArm, CommanderMode, CommanderAction

import logging
import threading, _thread

logging.basicConfig(level=logging.DEBUG)


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

        self.commander_arm_pub= self.create_publisher(
            CommanderArm, '/com/in/arm', qos_profile)
        self.commander_mode_pub= self.create_publisher(
            CommanderMode, '/com/in/mode', qos_profile)
        self.commander_action_pub= self.create_publisher(
            CommanderAction, '/com/in/action', qos_profile)
        
        
        self.exit=False
        self.last_heartbeat=0
        self.delta_hr=100000#alto per averlo staccato all'inizio
        
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
        pad=[
            [sg.Text('Drone manual control')],
            [sg.Text("\nspeed"),sg.Slider((0,3), key="pad_speed",default_value=0.2, resolution=slid.resolution, orientation=slid.orientation, s=slid.size)],
            [sg.Text()],
            [sg.RealtimeButton(sg.SYMBOL_UP, key='pad_turn_left'),sg.RealtimeButton(sg.SYMBOL_UP, key='pad_forward'),sg.RealtimeButton(sg.SYMBOL_UP, key='pad_turn_right')],
            [sg.RealtimeButton(sg.SYMBOL_LEFT, key='pad_left'),sg.Text(size=(10,1), key='pad_status', justification='c', pad=(0,0)),sg.RealtimeButton(sg.SYMBOL_RIGHT, key='pad_roght')],
            [sg.Text('           '),sg.RealtimeButton(sg.SYMBOL_DOWN, key='pad_back')],
            [sg.Text()],
            [sg.Column([[sg.Quit(button_color=(sg.theme_button_color()[1], sg.theme_button_color()[0]), focus=True)]], justification='r')]
        ]
        
        essential=[
                [sg.Text("drone: UNKNOWN", key="arm_text"),sg.Text("commander: DISCONNECTED", key="commander_status")],
                [sg.Button('ARM', key="arm_button"),sg.Button('DISARM', key="disarm_button")],
                [sg.Button('TAKEOFF', key="takeoff_button"),sg.Button('LAND', key="land_button")]
        ]
        
        stop=[
            [sg.Button("STOP", key="btn_stop",size=(5,5))]
        ]
        
        mode = [
            [sg.Radio('routine', 1, key= "rd_routine"),sg.Radio('path',  1, key = "rd_path"),sg.Radio('spin', 1, key = "rd_spin"),sg.Radio('updown', 1, key = "rd_updown"),sg.Radio('None', 1, key="rd_none",default=True)],
            [sg.Button('Confirm', key="mode_button")],
            [sg.Text("\nradius"),sg.Slider((0,slid.max_radius), key="radius",default_value=2.0, resolution=slid.resolution, orientation=slid.orientation, s=slid.size),
            sg.Text("\nomega"),sg.Slider((0,slid.max_omega), key="omega",default_value=1.0, resolution=slid.resolution, orientation=slid.orientation, s=slid.size)],
            [sg.Text("\nspin speed"),sg.Slider((0,slid.max_range), key="spin_speed",default_value=1.0, resolution=slid.resolution, orientation=slid.orientation, s=slid.size)]
        ]
        
        test = [
            [sg.Text('My one-shot window.')],      
            [sg.InputText()],      
            [sg.Submit(), sg.Cancel()]
        ] 
        test2 = [
            [sg.Text('My one-shot window.')],      
            [sg.InputText()],      
            [sg.Submit(), sg.Cancel()]
        ] 
        
        text = [
            [sg.Text('Text1'),sg.Text('Text2'),sg.Text('Text3')]
        ]
        text2 = [
            [sg.Text('Text1'),sg.Text('Text2'),sg.Text('Text3')]
        ]
        text3 = [
            [sg.Text('Text1'),sg.Text('Text2'),sg.Text('Text3')]
        ]
        
        
        slider1=[
            [sg.Slider((0,10), orientation='h', s=(10,15))]
        ]
        
        tabs = sg.TabGroup([[sg.Tab('local position',text), sg.Tab('gps', text2), sg.Tab('failsafe', text3)]])
        tabs_r = sg.TabGroup([[sg.Tab('Auto',mode), sg.Tab('Manual', pad)]])
        #right_frame= sg.Frame('Commands', [sg.Frame("core", essential), sg.Frame("Mode",mode)]    )
        right_layout= [
            [sg.Frame("", essential),sg.Frame("", stop)],
            [sg.Frame("",[[tabs_r]])]
        ]
        right_frame= sg.Frame('Commands', right_layout    )
        
        layout = [
                [tabs,right_frame]
            ]

        # window = sg.Window('Window Title', layout)    

        # event, values = window.read()    
        # window.close()
        
        # exit(0)

        #self.window = sg.Window('Commander GUI', sg.Frame('controller',essential))
        self.window = sg.Window('Commander GUI', layout)

        t=threading.Thread(target=self.window_thread)
        t.start()
        logging.info("thread for windo started")

    def window_thread(self):
        while True:
            # This is the code that reads and updates your window
            event, values = self.window.read(timeout=100)
            
            if event != sg.TIMEOUT_EVENT:
                logging.debug("event: ")
                logging.debug(event)
                
            if event in (sg.WIN_CLOSED, 'Quit'):
                break
            # elif event != sg.TIMEOUT_EVENT:
            #     # if not a timeout event, then it's a button that's being held down
            #     self.window['-STATUS-'].update(event)
            elif event == sg.TIMEOUT_EVENT:
                # if not a timeout event, then it's a button that's being held down
                # print("timeout")
                #self.window[event].update(self.arm_text)
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
                
                
                com_stat="commander: DISCONNECTED"
                color_stat= "red"
                if self.delta_hr<2000:#1.5 secondi arbitrari
                    com_stat="commander: CONNECTED"
                    color_stat = "green"
                try:
                    self.window["commander_status"].update(com_stat)
                    self.window["commander_status"].update(background_color=color_stat)
                    self.window["arm_text"].update(dict_arm[self.vehicle_status.arming_state])
                    self.window["arm_text"].update(background_color=dict_arm_color[self.vehicle_status.arming_state])
                except:
                    #logging.error("no vehicle status yet")
                    asdasd=1
                
            elif  event== "arm_button":
                logging.info("arm button pressed")
                self.arm()
            elif  event== "disarm_button":
                logging.info("disarm button pressed")
                self.disarm()
                
            elif  event== "disarm_button":
                logging.info("disarm button pressed")
                self.disarm()
                
            elif  event== "takeoff_button":
                logging.info("takeoff button pressed")
                self.action("takeoff")
                
            elif  event== "land_button":
                logging.info("land button pressed")
                self.action("land")
                
            elif  event== "mode_button":
                logging.info("confirm mode button pressed with value: ",values)
                self.changeMode(values)
                
            elif  event== "mode_button":
                logging.info("confirm mode button pressed with value: ",values)
                self.changeMode(values)
                
            else:
                # A timeout signals that all buttons have been released so clear the status display
                self.window['-STATUS-'].update('')

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
    
    def commander_heartbeat(self,msg):
        new_time = msg.timestamp
        current = int(self.get_clock().now().nanoseconds / 1000)
        #print("new time:", current)
        self.delta_hr=current- new_time
        self.last_heartbeat = new_time
        # logging.debug("hr delta")
        #logging.debug(self.delta_hr)
        #1 secondo arbitrario

    def arm(self):
        msg = CommanderArm()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.arm=True
        try:
            self.commander_arm_pub.publish(msg)
        except Exception as e:
            print(e)
        logging.info("sent arm command")
        
    def disarm(self):
        msg = CommanderArm()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.arm=False
        self.commander_arm_pub.publish(msg)
        logging.info("sent disarm command")
        
    def changeMode(self, values):
        msg = CommanderMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
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
        
        self.commander_mode_pub.publish(msg)
        logging.info("sent change mode command")

    def action(self, action):
        msg = CommanderAction()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.action = action
        self.commander_action_pub.publish(msg)
    

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
