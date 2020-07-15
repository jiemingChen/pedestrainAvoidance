# Pedestrain Avoidance
There are three parts including simulation in Gazebo, people tracking and control of the vehicle.

**People Tracking**: Using CNN to detect person. Kalman Filter and Matching algorithm to track people, based on SORT algorithm. Using RGBD camera and 1-layer Lidar to  perceive environment.

**Collision Avoidance**: Treate it as a optimization problem. MPC + Linearize Bicycle Model + Convexify collision avoidance constraints(sequential convex method).

### Mobile Robot
![track](https://media.giphy.com/media/eKg6VlYQj2uvz3WRWG/giphy-downsized-large.gif)
![avoid1](https://media.giphy.com/media/dBZR8UcLQvEMRegmBj/giphy-downsized-large.gif)
![avoidtraj1](https://media.giphy.com/media/VF5DpnOKp4Cw05afAS/giphy.gif)

![detecion](https://media.giphy.com/media/dVimvA9hbLlGFSFmL5/giphy.gif)
![avoidance](https://media.giphy.com/media/cPg8Gad5tsDsB80l3r/giphy.gif)
## Code Structure
**actor_plugin** and **vehicle_plugin**: Interface between ROS and GAZEBO which send command to objects in GAZEBO and send data from GAZEBO to ROS.

**my_car**: Model files of vehicle, sensors(RGBD camera and LIDAR)  and the world file. 

**people_sim**: Model files of pedestrains.

**rgbd_detector**: Object Detection and tracking people.

**collision_avoidance**: motion planning and control. 

### Prerequisites
```
ROS
GAZEBO
Eigen
OpenCV
CPLEX
```

### Installing
```
1. Clone this reporsitory in workspace.
2. Modify YOLO Library path in CmakeList of rgbd_detector.
3. Modify CPLEX path in CmakeList of avoidance.
4. catkin_make in this workspace
5. Move libactorplugin_ros.so and libmy_controller_lib.so generated in actor_plugin and vehicle_plugin into the directory ~/ws/devel/lib/ 
6. roslaunch al.launch
```


### Maintainer
Jieming Chen(15001781672@163.com)

Hao Li



### References
[1] Bewley, Alex, et al. "Simple online and realtime tracking."  IEEE, 2016.

[2] Redmon, Joseph, and Ali Farhadi. "Yolov3: An incremental improvement." 

[3] Ribeiro, Maria Isabel, and Pedro Lima. "Kinematics models of mobile robots." 

[4] Alrifaee, Bassam, Janis Maczijewski, and Dirk Abel. "Sequential convex programming MPC for dynamic vehicle collision avoidance."
