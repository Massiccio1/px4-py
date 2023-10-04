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
import threading

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
        
        
        trl=sg.Frame('Input data data',[[
                sg.Text('Robotics Remote Control')],
                [sg.Text('Hold Down Button To Move')],
                [sg.Text()],
                [sg.Text('           '),
                sg.RealtimeButton(sg.SYMBOL_UP, key='-FORWARD-')],
                [sg.RealtimeButton(sg.SYMBOL_LEFT, key='-LEFT-'),
                sg.Text(size=(10,1), key='-STATUS-', justification='c', pad=(0,0)),
                sg.RealtimeButton(sg.SYMBOL_RIGHT, key='-RIGHT-')],
                [sg.Text('           '),
                sg.RealtimeButton(sg.SYMBOL_DOWN, key='-DOWN-')],
                [sg.Text()],
                [sg.Column([[sg.Quit(button_color=(sg.theme_button_color()[1], sg.theme_button_color()[0]), focus=True)]], justification='r')
            ]]) 
        essential=[sg.Frame('controller',[
                [sg.Text("UNKNOWN", key="arm_text"),sg.Text("comamnder: DISCONNECTED", key="commander_status")],
                [sg.Button('ARM', key="arm_button"),sg.Button('DISARM', key="disarm_button")],
                [sg.Button('TAKEOFF', key="takeoff_button"),sg.Button('LAND', key="land_button")],
                [sg.Text()],
                [sg.Radio('routine', 1, key= "rd_routine"),sg.Radio('path',  1, key = "rd_path"),sg.Radio('spin', 1, key = "rd_spin"),sg.Radio('updown', 1, key = "rd_updown"),sg.Radio('None', 1, default=True)],
                [sg.Button('Confirm', key="mode_button")]
                ])]
        pad=sg.Frame('Output data',[
                [sg.Text("UNKNOWN", key="arm_text"),sg.Text("comamnder: DISCONNECTED", key="commander_status")],
                [sg.Button('ARM', key="arm_button"),sg.Button('DISARM', key="disarm_button")],
                [sg.Button('TAKEOFF', key="takeoff_button"),sg.Button('LAND', key="land_button")],
                [sg.Text()],
                [sg.Radio('routine', 1, key= "rd_routine"),sg.Radio('path',  1, key = "rd_path"),sg.Radio('spin', 1, key = "rd_spin"),sg.Radio('updown', 1, key = "rd_updown"),sg.Radio('None', 1, default=True)],
                [sg.Button('Confirm', key="mode_button")],
                [sg.Text()],
                [sg.RealtimeButton(sg.SYMBOL_UP, key='-FORWARD-')],
                [sg.Column([[sg.Quit(button_color=(sg.theme_button_color()[1], sg.theme_button_color()[0]), focus=True)]], justification='r')
            ]])
        layout = [
            [sg.Frame('trl',[essential])
            ]
        ]

        self.window = sg.Window('Commander GUI', layout)
        
        #threading.Thread(target=self.window_thread).start()
        
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
                    0:"status: UNKNOWN",
                    1:"status: DISARMED",
                    2:"status: ARMED"
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
                
            else:
                # A timeout signals that all buttons have been released so clear the status display
                self.window['-STATUS-'].update('')

        logging.info("window closing")
        self.window.close()
        self.exit=True
        exit(0)
    
    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        #logging.debug("recived vehicle position")
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
        logging.debug(self.delta_hr)
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
        
        msg.routine=values["rd_routine"]
        msg.path=values["rd_path"]
        msg.spin=values["rd_spin"]
        msg.updown=values["rd_updown"]
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
