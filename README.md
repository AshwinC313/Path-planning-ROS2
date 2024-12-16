# Evaluating path planning methods on mobile robots using ROS2

### The idea of this project is to evaluate and implement the various global path planning approaches to perform autonomous navigation of mobile robots and compare on the performance of various approaches.

![A-model Architecture](images/A_model.jpg)
<caption>A-model architecture for designing an autonomous vehicle or robot.</caption>

### For this project, the followwing approach is followed :

* ### Teleoperating the robot to scan the environment
* ### Creating a costmap from the scanned environment
* ### Creating a global path planner which generates a path to the goal
* ### Creating a local controller which can stabilize the robot and perform obstacle avoidance

### For that, an occupancy map is created and a costmap is generated which stores the information about the available space in the map.
#### Turtlebot's odometry and other vital information is subscribed by a node which futher processes to break down into grid information, after which global path planners are used to generate the path.
