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

### [rug_panda_worlds](rug_panda_worlds)
This is where you can add your own worlds that contain the Franka Panda robot and any other objects you want to interact 
with. Currently, there are two options: a world with a table and keyboard, and a world with a simple pick and place 
setup.  You can launch the worlds by running the launch files:
```bash
roslaunch rug_panda_worlds panda.launch world:=$(rospack find rug_panda_worlds)/world/stone.sdf
roslaunch rug_panda_worlds panda.launch world:=$(rospack find rug_panda_worlds)/world/keyboard.sdf

# With controller
roslaunch rug_panda_worlds panda.launch world:=$(rospack find rug_panda_worlds)/world/stone.sdf controller:=example_jointspace_controller
```

### [rug_panda_controllers](rug_panda_controllers)
This is where you can add your own controllers that send torques to the Franka Panda robot. It currently contains two 
controllers, a joint space controller, and a workspace controller. Note that these controllers are strictly for 
demonstrating the structure of a controller, and do not correctly compute torques. The controllers can be launched by 
running the launch files:
```bash
roslaunch rug_panda_controllers controller.launch controller:=example_jointspace_controller
roslaunch rug_panda_controllers controller.launch controller:=example_workspace_controller
```

### [rug_panda_planners](rug_panda_planners) 
This is where you can add your own scripts that determine which target poses the robot should move to. Currently, there 
are two options: a script that sends sinusoidal joint trajectories to the controllers, or a scripts for interactive
control through moveit. Moveit's forward and inverse kinematics are used to allow both example controllers to work with
either script. You can launch the scripts by running the launch files:
```bash
roslaunch rug_panda_planners planner.launch planner:=interactive
roslaunch rug_panda_planners planner.launch planner:=sinusoidal_trajectory
```

## Adding your own controllers
To add you own controller, follow the same structure as the existing example controller:
- Add the header file in [rug_panda_controllers/include](rug_panda_controllers/include)

- Add the cpp files in [rug_panda_controllers/src](rug_panda_controllers/src). Don't forget to add 
"PLUGINLIB_EXPORT_CLASS(rug_panda_controllers::<CONTROLLER_CLASS>, controller_interface::ControllerBase)" at the end 
of one of the cpp files (e.g. init.cpp). For information regarding the controller class, see 
[franka_ros documentation](https://frankaemika.github.io/docs/franka_ros.html#writing-your-own-controller).

- Add the controller to the [franka_example_controllers_plugin.xml](rug_panda_controllers/franka_example_controllers_plugin.xml)

- Add the controller in [rug_panda_controllers.yaml](rug_panda_controllers/config/rug_panda_controllers.yaml) 

## Miscellaneous Notes
Gravity compensation is handled internally by LibFranka, meaning you do not need to worry about it in your controllers.
Howver, there is an issue with the gravity compensation in the Gazebo physics simulation, causing drift and unintended 
motion (see issue [#160](https://github.com/frankaemika/franka_ros/issues/160) and [#233](https://github.com/frankaemika/franka_ros/issues/233)). For jointspace controllers where error is minimized at the joint level, 
this is not a major issue while control is active. For workspace controllers, this can cause the elbow to drift. This 
issue can be mitigated by using nullspace control, as implemented with
[cartesian example controller](https://github.com/frankaemika/franka_ros/blob/develop/franka_example_controllers/src/cartesian_impedance_example_controller.cpp) in the
franka_example_controllers package. 

You can work and run scripts from a venv and/or IDE like PyCharm. When using a venv, make sure to adjust the shebang 
(the line with "#!/usr/bin/env python") at the top of the scripts to the interpreter you are using (e.g. 
`#!/home/user/venv/bin/python`). When using PyCharm, launch it from a terminal that source the devel/setup.sh file, and 
set the `src` directory with python files as the source root to ensure pycharm recognizes the packages correctly.