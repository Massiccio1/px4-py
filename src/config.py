import logging 


MODE_NONE=0
MODE_ROUTINE=1
MODE_SPIN=2
MODE_GOTO=3
MODE_PATH=4
MODE_UPDOWN=5
MODE_STOP=6

mode_dict={
    MODE_NONE: "None",
    MODE_ROUTINE: "Routine",
    MODE_SPIN: "Spin",
    MODE_GOTO: "Goto",
    MODE_PATH: "Path",
    MODE_UPDOWN: "Updown",
    MODE_STOP: "Stop"
    }

dist_threshold=0.3
#circle procedure
dt = 0.1
theta = 0.0
radius = 2.0
omega = 2.0
pose=0

#tic of timer
tic=0

#mode, only one
mode = MODE_ROUTINE
#path procedure
path_points=[0.0, 0.0, 0.0]
path_index = -1



#spin procedure
spin_rad=0.05

#updown procedure

##flight config
#relative, negative means up (NED frame)
#absolute with optitrack, negative means up (NED frame)
takeoff_height = -1.4
cruising_speed = 0.5

#gui
gui = True
log=logging.INFO


############################################
#opti-to-px4
opti_to_px4_dt=0.01 #100 hz
movement_speed=0.7
