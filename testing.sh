#!/bin/bash

# This script can be used when we use the workspace for testing by connecting it to MAVROS 

roscore & roslaunch mavros apm2.launch fcu_url:=udp://localhost:14550@ & rosrun map currentXY & rosrun map markPoints & rviz & rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map my_frame 10 & rosrun map waypoints 


# will have to run command separately:

# cd /home/sarthak/Desktop/ardupilot/ArduPlane
# /home/sarthak/Desktop/ardupilot/Tools/autotest/sim_vehicle.py --console --map --aircraft test
 