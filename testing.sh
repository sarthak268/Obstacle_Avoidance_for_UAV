xterm -e roscore
xterm -e cd /home/sarthak/Desktop/ardupilot/ArduPlane | /home/sarthak/Desktop/ardupilot/Tools/autotest/sim_vehicle.py --console --map --aircraft test
xterm -e roslaunch mavros apm2.launch fcu_url:=udp://localhost:14550@
xterm -e rosrun map currentXY