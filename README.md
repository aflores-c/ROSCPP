# ROSCPP
You can find a set of basics ROS C++ tutorials. 
In this tutorials, a gazebo simulation is used to work with the turtlebot robot.

## Requierements
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
We create a node that subscribes to the /scan topic to read the distances from the robot to the obstacles.
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
2. Launch the simulator. We need to choose the robot. In this case, we use the waffle turtlebot robot. 
```sh
$ export TURTLEBOT3_MODEL=waffle
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
3. In another terminal, run the "avoid_obstacle" node
```sh
$ rosrun turtlebot_obstacles turtlebot_obstacles_node
```
### 02_ROS Services
### src/turtlebot_services
#### Overview
We modified the previous turtlebot controller class and we add a service member function. This server function will allow to move the robot randomly, as we've seen in the previous tutorial. Now, the robot just use the callback to get the distance and you can call the service to move the robot. You request to move a robot for a certain duration and when the duration finishes, the service returns a response of success. 

[turtlebot_gazebo_2]: ./images/services.png
[topics_graph_2]: ./images/rqt_services.png

![alt text][turtlebot_gazebo_2]
![alt text][topics_graph_2]

#### Usage 
1. Source your environment
```sh
$ cd catkin_ws
$ cd source devel/setup.bash
```
2. Launch the simulator
```sh
$ export TURTLEBOT3_MODEL=waffle
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
3. In another terminal, run the "avoid_obstacle" node
```sh
$ rosrun turtlebot_services turtlebot_services_node
```
4. You can see the available services and then you can call that services.
```sh
$ rosservice list
$ rosservice call /move_robot "duration:
  secs: 15
  nsecs: 0" 
```
5. There is another way to call the services. It is done by running a service client.
```sh
$ rosrun turtlebot_services turtlebot_services_client 
```
