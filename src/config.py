import logging 
#circle procedureù
dt = 0.01
theta = 0.0
radius = 2.0
omega = 2.0
pose=0

#tic of timer
tic=0

#mode, only one
routine=False
path =False
spin=True
updown=False
#path procedure
path_points=[0.0, 0.0, 0.0]
path_index = -1

#spin procedure

#updown procedure

##flight config
#relative, negative means up (NED frame)
takeoff_height = -1.0

#gui
gui = True
log=logging.DEBUG
