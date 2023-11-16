# Communication Setup

## Overview

The entire solution pipeline uses two systems. System 1 hosts the packages that control the UR5e, moveit commander, RobotIQ gripper control.
System 2 hosts the packages that performs the grasp planning.

#### Note: It is mandatory that both the systems are connected to the same network. Therefore we change the ip address of graspit system manually to `192.168.1.101` and netmask to ` 255.255.255.0` to ensure that both the systems are connected to the same network.

## ROS setup

In our pipeline, System 1 acts as the ros master and the system 2 is part of the ros network.
To ensure that both the systems run on the same ROS master, do the changes in the .bashrc file on system 1:
```
gedit .bashrc
```
Go to the last line of the file and add the following:

```
export ROS_MASTER_URI=192.168.1.50:11311 
export ROS_IP=192.168.1.50 # System 1's IP address to be used
```
Source the changes
```
source .bashrc
```
---
Now go to system 2, and add the following to the .bashrc file
```
gedit .bashrc
```
Go to the last line of the file and add the following:

```
export ROS_MASTER_URI=192.168.1.50:11311 # System 1's IP address to be used
export ROS_IP=192.168.1.101 # System 2's IP address to be used
```
Source the changes
```
source .bashrc
```

## Checking

Run the ROS Master on system 1.
```
roscore
```
Run rosnode list on system 2 to find the nodes that are active on the common network.
```
rosnode list
```

You should see the following output on system 2.
```
/rosout
```

If you see the output, the communication has been establised successfully.

#### Note: ALternate method would be using alias to name the ROS variables.
