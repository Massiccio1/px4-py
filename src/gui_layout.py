import PySimpleGUI as sg
import numpy as np


class gui_layout:
    def __init__(self,GRAPH_SIZE) -> None:
        
        self.layout=[]
        
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
        
        status=[
            [sg.StatusBar("drone: UNKNOWN", key="arm_text"),sg.StatusBar('preflight check: UNKNOWN', key="preflight_text"),sg.StatusBar('failsafe status: UNKNOWN', key="failsafe_text")],
            [sg.StatusBar("commander: DISCONNECTED", key="commander_status"),sg.StatusBar('offboard signal: UNKNOWN', key="offboard_text")],
        ]
        
        essential=[
                
                [sg.Button('ARM', key="arm_button"),sg.Button('DISARM', key="disarm_button")],
                [sg.Button('TAKEOFF', key="takeoff_button"),sg.Button('LAND', key="land_button")]
        ]
        
        stop=[
            [sg.Button("STOP", key="btn_stop",size=(5,5)), 
             sg.Button("KILL", key="btn_force_disarm",size=(5,5), button_color="dark red"),
             sg.Button("TEST", key="btn_test",size=(5,5), button_color="dark goldenrod")
             ]
        ]
        
        mode = [
            [
                sg.Radio('spin', 1, key = "rd_spin"), 
                sg.Radio('routine', 1, key= "rd_routine"),
                sg.Radio('goto', 1, key = "rd_goto"),
                sg.Radio('path',  1, key = "rd_path"),
                sg.Radio('updown', 1, key = "rd_updown"),
                sg.Radio('None', 1, key="rd_none",default=True)
            ],
            [
                sg.Text("\nheight"),
                sg.Slider((1.0,20.0), key="height",default_value=2.0, resolution=slid.resolution, orientation=slid.orientation, s=slid.size),
                sg.Text("\nspin speed"),
                sg.Slider((0,slid.max_range), key="spin_speed",default_value=1.0, resolution=slid.resolution, orientation=slid.orientation, s=slid.size)
            ],
            [sg.Text("\nradius"),sg.Slider((0,slid.max_radius), key="radius",default_value=2.0, resolution=slid.resolution, orientation=slid.orientation, s=slid.size),
            sg.Text("\nomega"),sg.Slider((0,slid.max_omega), key="omega",default_value=1.0, resolution=slid.resolution, orientation=slid.orientation, s=slid.size)],
            [
                sg.Text("goto"),
                sg.Text("X:"),
                sg.Input(key='x',size=(4,2)),
                sg.Text("Y:"),
                sg.Input(key='y',size=(4,2)),
                sg.Text("Z:"),
                sg.Input(key='z',size=(4,2)),
                sg.Text("(NED frame)")
            ],
            [
                sg.Text("Chose a path file"),
                sg.Input(key='file_input',s=(20,1)),
                sg.FileBrowse(file_types=(("JSON Files", "*.json"), ("ALL Files", "*.*"),),key="file_browser"),
                sg.Button("Load", key="file_load")
            ],
            [
                sg.Text("\nmovement speed"),
                sg.Slider((0.0,10.0), key="movement_speed",default_value=0.7, resolution=slid.resolution, orientation=slid.orientation, s=slid.size),
            ],
            
            [sg.Button('Confirm', key="mode_button")],

            [sg.Table([], ['X','Y','Z'], vertical_scroll_only=True,expand_x=True,expand_y=False,key="table_path",num_rows=10,visible=False)]
        ]
        
        test = [
            [sg.Text('My one-shot window.')],      
            [sg.InputText()],      
            [sg.Submit(), sg.Cancel()]
        ] 
        
        mode_display = [
            [sg.Text('Mode')],      
            [sg.StatusBar('UNKNOWN',key="mode_text",background_color="DimGrey")],      
            [sg.StatusBar('UNKNOWN',key="submode_text",auto_size_text=True,background_color="DimGrey")],
            [sg.Graph(GRAPH_SIZE, (0,0), GRAPH_SIZE, key='graph_hb', background_color='lightblue')]
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
            [sg.Text("Battery:")],
            [sg.Text("no info", key="battery_text")],
            [sg.ProgressBar(100, orientation='v',expand_x=True,s=(10,20),expand_y=True, k='bar_battery')]
        ]

        graph_layout = [
            [sg.Text('Speed:', size=(20,1)),
             sg.Slider((1,30), default_value=5, orientation='h', key='graph_speed')],
            [sg.Text('Scale:', size=(20,1), ),
             sg.Slider((1,1000), default_value=100, orientation='h', key='graph_scale')],
            [sg.Graph(GRAPH_SIZE, (0,0), GRAPH_SIZE, key='graph_1', background_color='lightblue'),],
            [sg.Graph(GRAPH_SIZE, (0,0), GRAPH_SIZE, key='graph_2', background_color='lightblue'),],
        ]
        
        tabs_l = sg.TabGroup([[sg.Tab('local position',tab_lp), sg.Tab('gps', tab_gps), sg.Tab('battery', tab_battery), sg.Tab("error graph", graph_layout),sg.Tab('status', tab_status)]],expand_y=True)
        tabs_r = sg.TabGroup([[sg.Tab('Auto',mode), sg.Tab('Manual', pad_frame)]])
        #right_frame= sg.Frame('Commands', [sg.Frame("core", essential), sg.Frame("Mode",mode)]    )
        
        arm_and_stop=[
            [sg.Frame("", essential)],
            [sg.Frame("", stop)]
        ]
        
        right_layout= [
            [sg.Frame("", status)],
            [
                sg.Frame('',arm_and_stop),
                sg.Frame('',mode_display, vertical_alignment='center',element_justification='top',expand_x=True,expand_y=True),
                sg.Frame("", battery_layout,expand_y=True)
            ],
            [sg.Frame("",[[tabs_r]])],
            
        ]
        right_frame= sg.Frame('Commands', right_layout    )
        
        layout = [
                [tabs_l,right_frame]
            ]
        
        self.layout=layout
