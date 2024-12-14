# Evaluating path planning methods on mobile robots using ROS2

### The idea of this project is to evaluate and implement the various global path planning approaches to perform autonomous navigation of mobile robots and compare on the performance of various approaches.
### Before implementing any path planning algorithm, its necessary to know the environment in which the robot would be traversing.

### For that, an occupancy map is created and a costmap is generated which stores the information about the available space in the map.
#### Turtlebot's odometry and other vital information is subscribed by a node which futher processes to break down into grid information, after which global path planners are used to generate the path.
