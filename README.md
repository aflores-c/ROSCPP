# ROSCPP
You can find a set of basics ROS C++ tutorials. 

## Requierments
You need to install ROS Melodic , wich you can find in the offical page. Aditionally, you need the turtlebot 3 simulator, that will be covered in the video tutorial.
By now, I put the installation links.
1. Ubuntu 18  
2. ROS Melodic - 
3. Turtlebot 3 simulator - 

## Usage 
You need to download the repository and paste the requiered package in your workspace, depending on the tutorial you are working.
1. Create a catkin workspace
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```
2. Download the repository insider the /src file of the workspace
```sh
$ cd /src
$ git clone https://github.com/aflores-c/ROSCPP.git
```
3. Make again and check all compiles correctly.
```sh
$ cd ..
$ catkin_make
``` 
## Tutorials

### 01_Publishers and subscribers
### src/turtlebot_obstacles
#### Overview
In this tutorial, a gazebo simulation is used to work with the turtlebot robot.
We create a node that subscribes to the /scan to read the distances from the objects.
Then, the node uses this data to avoid obstacles by publishing velocity commands to the /cmd_vel topic to move the robot.

[turtlebot_gazebo]: ./images/turtlebot_obst_avoid.png
[topics_graph]: ./images/rqt_graph.png

![alt text][turtlebot_gazebo]
![alt text][topics_graph]

#### Usage 
1. Source your environment
```sh
$ cd catkin_ws
$ cd source devel/setup.bash
```
2. Launch the simulator
```sh
$ rosrun turtlebot_obstacles turtlebot_obstacles_node
```
3. in other terminal, run the "avoid_obstacle" node
```sh
$ rosrun turtlebot_obstacles turtlebot_obstacles_node
```



