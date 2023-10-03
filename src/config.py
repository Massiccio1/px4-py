import logging 
#circle procedureù
routine=0
dt = 0.01
theta = 0.0
radius = 2.0
omega = 2.0
pose=0

#tic of timer
tic=0

#path procedure
path =0
path_points=[0.0, 0.0, 0.0]
path_index = -1

#spin procedure
spin=0

#updown procedure
updown=0

##flight config
#relative, negative means up (NED frame)
takeoff_height = -1.0

#gui
gui = True
log=logging.DEBUG
