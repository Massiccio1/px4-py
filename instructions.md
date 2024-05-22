# instructions

# Table of Contents

- [command and programs](#commands-and-programs)
  1. [px4-py](#px4-pypy)
  2. [gui](#guipy)
  3. [opti-to-px4.py](#opti-to-px4py)
  4. [opti](#opti)
  5. [rviz](#rviz)
  6. [Docker commander](#docker-commander)
  7. [Docker companion](#docker-companion)
- [Build instructions](#build-instructions)
- [Run instructions](#run-instructions)

## commands and programs

Used and edit the file `resource/.bash_aliases` to fit your system

### px4-py.py

Main process, recives all ross messages from gui and companion, elaborates information and published commands to the drone

### gui.py

Graphical interface, does not comunicate direcly with the companion.\
Displays information and relays commands to the commander; it has a few options:

- `ARM` arms the drone if the above contitions are met
- `DISARM` disarms the drone before it takes off or when it landed, does not work mid flight
- `TAKEOFF` takes of to the height declared in `src/config.py` by the variable `takeoff_height`
- `LAND` lands at (0,0) local frame, position cannot be overriden
- `STOP` stops any activity and hold the position
- `TEST` button to test features, currently lands on a random position
- `KILL` emergency stop, terminates the flight immediately, enem mid air
- `SPIN` spin on self wit omega = spin speed slider
- `ROUTINE` starts hovering in circle around (0,0), at height = height , omega = omega slider,radius = radius
- `GOTO` goes to position x,y,z !!!in NED frame (up is negative height)
- `PATH` load a path file like src/medium.json and the drone fill follow all points with speed = moviment speed slider
- `UPDOWN` goes up and lands, mainly for testing
- `NONE` default, no activity, the drone will hover
- `GRAPH` shows arudimentary graph of position offset between the set position and the current position, gives a rough estimate of the offset

other commands and tabs are self explanatory

### opti-to-px4.py

Subscribes to the optitrack pose topic, converts the frame to NED and publishes it back

### opti

Alias for `TRACKED_ROBOT_ID=44 ros2 run optitrack_interface optitrack` , starts the optitrack client and published ROS2 pose messages, might need to change IP addersses int the code.
This code is used alongside with `opti-to-px4.py`
Alternatively use `TRACKED_ROBOT_ID=44 ros2 run optitrack_interface optitrack2` for an all in one solition of decoding optitrack messages and converting them into px4 messages.\
\
The `TRACKED_ROBOT_ID` tracked robot varable is used to chose the ID of the optitrack model without recompiling the code

### rviz

The command `r2path1000`, alias for `RVIZ_MAX_BUFFER=1000 ros2 launch px4_offboard visualize.launch.py` launches rviz with 2 poses, the big one is the pose of the drone, the small one is the pose from optitrack.\
`RVIZ_MAX_BUFFER=1000` sets the buffer to the latest 1000 messages recived, this prevents clogging of the iterface and slow down of the system, not setting the varable will keep the complete history

### Docker commander

`dockercommander` alias for `xhost + && docker run --rm -it --network host -e DISPLAY=:0 -v $HOME/shared:/root/shared -v /tmp/.X11-unix:/tmp/.X11-unix drone-commander:main-squash ; xhost -` launches the commander container with all programs and the gui should appear

### Docker companion

`dockerdrone` alias for .... #TODO\
launches the companion container with all programs

# build instructions

Always source ROS2 files (alias `s`)

## general setup

- Follow the instructions `Installation & Setup` at https://docs.px4.io/main/en/ros/ros2_comm.html#installation-setup
- clone the repositories in the ROS" `src` folder:
  ```
  git clone https://github.com/Massiccio1/px4-py.git
  git clone https://github.com/Massiccio1/optitrack_interface.git
  git clone https://github.com/Massiccio1/px4-offboard.git
  git clone https://github.com/Massiccio1/px4-22.git
  git clone https://github.com/PX4/px4_msgs.git
  git clone https://github.com/Massiccio1/commander_msg.git
  ```
- build for ROS2 with `colcon build --symlink-install --parallel-workers 8` or the alias `ccbsi`

# run instructions

Always source ROS2 files (alias `s`)

## Commander

run:

- `px4-py/src/px4-py.py` for the offboard controller
- `px4-py/src/px4-py.py` for the gui

## Companion

run:

- `agentreal` for the XRCE agent (edit /dev/tty based on your board)
- `TRACKED_ROBOT_ID=44 ros2 run optitrack_interface optitrack` for the optitrack client
- `px4-py/src/opti-to-px4.py` for converting pose messages to px4 messages
  - alternatively use `TRACKED_ROBOT_ID=44 ros2 run optitrack_interface optitrack2` instead of the prevuois 2 commands

## Docker

following the bash aliases, run `dockercommander` on the commander computer and `dockerdrone` on the companion

### PX4

Follow the `Installation & Setup` chapter on https://docs.px4.io/main/en/ros/ros2_comm.html#humble

## version

## Third Example

## [Fourth Example](http://www.fourthexample.com)
