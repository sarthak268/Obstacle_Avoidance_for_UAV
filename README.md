# obstacle_avoidance_for_UAV
This is a ROS workspace that creates a trajectory for a UAV to follow passing through a set of given waypoints and avoiding a set of given cylindrical obstacles, using a path planning algorithm. The testing is done through a node that plots the waypoints, obstacles and the current pose of UAV on RVIZ for examining the accuracy of the algorithm. 

#Commands : 
## Testing :
..* roscore
..* /(path to sim_vehicle)/sim_vehicle.py --console --map --aircraft test
..* roslaunch mavros apm2.launch fcu_url:=udp://localhost:14550@ 
..* rosrun map currentXY  
..* rosrun map markPoints
..* rviz 
(write the frame id i.e. /my_frame in the Fixed Frame)
..* rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map my_frame 10
..* rosrun map waypoints


