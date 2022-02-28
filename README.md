# Obstacle Avoidance Simulator for Unmanned Aerial Vehicles (UAVs)

[![Hits](https://hits.seeyoufarm.com/api/count/incr/badge.svg?url=https%3A%2F%2Fgithub.com%2Fsarthak268%2FObstacle_Avoidance_for_UAV&count_bg=%2379C83D&title_bg=%23555555&icon=&icon_color=%23E7E7E7&title=hits&edge_flat=false)](https://hits.seeyoufarm.com)

This is a ROS workspace that creates a trajectory for a UAV to follow passing through a set of given waypoints and avoiding a set of given cylindrical obstacles, using a path planning algorithm. The testing is done through a node which plots the waypoints, obstacles and the current pose of UAV on RVIZ for examining the accuracy of the algorithm. 

# Requirements :
1. ROS 
2. ardupilot
3. mavros
4. rviz
5. Mission Planner (preferred) or apm planner

# Commands : 

## Testing existing Algorithms :
```
1. roscore
2. /(path to sim_vehicle)/sim_vehicle.py --console --map --aircraft test
3. roslaunch mavros apm2.launch fcu_url:=udp://localhost:14550@ 
4. rosrun map currentXY  
5. rosrun map markPoints
6. rviz 
(write the frame id i.e. /my_frame in the Fixed Frame)
7. rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map my_frame 10
8. rosrun map waypoints
```

# Citing

```
@misc{bhagat-obstacle-simulator-ros,
  author = {Sarthak Bhagat},
  title = {sarthak268/Obstacle_Avoidance_for_UAV},
  url = {https://github.com/sarthak268/Obstacle_Avoidance_for_UAV},
  year = {2018}
}
```
You may also want to look at the following paper (accepted at ICUAS'20).
```
@article{Bhagat2020UAVTT,
  title={UAV Target Tracking in Urban Environments Using Deep Reinforcement Learning},
  author={Sarthak Bhagat and P. B. Sujit},
  journal={ArXiv},
  year={2020},
  volume={abs/2007.10934}
}
```

For any queries, please contact me via mail on sarthak16189@iiitd.ac.in

# Support
Like my work? Buy me a coffee: https://ko-fi.com/sarthakbhagat


