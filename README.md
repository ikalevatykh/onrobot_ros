# onrobot
ROS onrobot meta-package


## Packages

### onrobot_description

This package contains the description of [OnRobot](https://onrobot.com/en/products) RG2 and RG6 grippers. The descriptions are based on the URDF format.

![Grippers](https://raw.githubusercontent.com/ikalevatykh/onrobot_ros/master/onrobot_description/media/rg_grippers.png "Grippers")

### onrobot_control

This package implements the onrobot_gripper_node for interfacing RG2 or RG6 grippers attached to a Universal Robot (UR3, UR5, UR10) from ROS. 
The node publishes the state of the gripper and offers a standard gripper action recognized by MoveIt!: `control_msgs::GripperCommandAction(width, max_effort)`

Important:
 - The package supports grippers attached to a Universal Robot controlled by the[Universal_Robots_ROS_Driver](http://wiki.ros.org/ur_robot_driver). 
 - The package assumes a gripper in a Teach Mode (without installed OnRobot UR Caps). See the *Teach Mode* section in the [gripper instruction](https://www.universal-robots.com/media/1226143/rg2-datasheet-v14.pdf) for details. In this mode only two gripper positions are supported: fingers fully opened and fingers fully closed.


You can launch the onrobot_gripper_node with:

```
roslaunch onrobot_control onrobot_gripper.launch
```

Parameters:

- ur_hardware_interface - Universal Robot driver namespace, default "/ur_hardware_interface"
- joint - gripper joint name in the robot description URDF, default "gripper_joint"
- max_position_voltage - analoug feedback voltage which correspond to the maximum width, default 3.0. See the *Analog feedback* section in the [gripper instruction](https://www.universal-robots.com/media/1226143/rg2-datasheet-v14.pdf) for details.
- state_publish_rate - joint states publish rate, default 50.0 Hz
- action_monitor_rate - GripperCommandAction feedback publish rate, default 20.0 Hz


