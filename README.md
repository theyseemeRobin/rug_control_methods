# Control Methods for Robotics - Template Repository
This repository is a starting template for the ["Control Methods for Robotics"](https://ocasys.rug.nl/2025-2026/catalog/course/WMAI037-05)
course at the University of Groningen. It contains the necessary files to get started with implementing your own 
control methods for the Franka Panda robot. The code is for use in conjunction with the [Franka Ros](https://frankaemika.github.io/docs/franka_ros.html)
package.



## Installation
The following instructions assume ROS Melodic is installed on your system, but may also work with other versions of 
ROS 1. If you have not done so already, please follow the [ROS installation instructions](https://wiki.ros.org/melodic/Installation/Ubuntu)
to install ROS on your system.

```bash
rest of instructions
```

## Usage
This repository contains 3 packages: 

### [rug_panda_controllers](rug_panda_controllers)
This is where you can add your own controllers that send torques to the Franka Panda robot. It currently contains two 
controllers, a joint space controller, and a workspace controller. Note that these controllers are strictly for 
demonstrating the structure of a controller, and do not correctly compute torques.

### [rug_panda_worlds](rug_panda_worlds)
This is where you can add your own worlds that contain the Franka Panda robot and any other objects you want to interact with.

### [rug_panda_planners](rug_panda_planners) 
This is where you can add your own planners, that determine which target poses the robot should move to. Currently, it 
publishes a sinusoidal joint trajectory, along with the corresponding end-effector trajectory.

## Adding your own controllers


## Miscellaneous Notes
Gravity compensation is handled internally by LibFranka, meaning you do not need to worry about it in your controllers.
Howver, there is an issue with the gravity compensation in the Gazebo physics simulation, causing drift and unintended 
motion (see issue [#160](https://github.com/frankaemika/franka_ros/issues/160) and [#233](https://github.com/frankaemika/franka_ros/issues/233)).
For jointspace controllers where error is minimized at the joint level, this is not a major issue while control 
is active. For workspace controllers, this can cause the elbow to drift. This issue can be mitigated by using nullspace 
control. 