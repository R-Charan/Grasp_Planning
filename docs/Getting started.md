# Instructions to get started with the UR5 

Host machine 
- OS version: Ubuntu 18.04
- ROS version: Melodic
- Create a Catkin workspace called catkin_ws under home  

## Change the below instructions to @GraspIt! official documentation [Universal Robots ROS Driver]( https://github.com/UniversalRobots/Universal_Robots_ROS_Driver): 
```
# change into the workspace

cd ~/catkin_ws

# clone the driver

git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver

# clone fork of the description. This is currently necessary, until the changes are merged upstream.

git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot

# install dependencies, following commands will take time be patient

sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y

# build the workspace

catkin_make

# activate the workspace (source it)

source devel/setup.bash
```

## Steps to install GraspIt!
<!-- Three packages(two prebuild) new one is grasp_service_node -->
### Install the following dependencies
```
sudo apt-get install libqt4-dev
sudo apt-get install libqt4-opengl-dev
sudo apt-get install libqt4-sql-psql
sudo apt-get install libcoin80-dev
sudo apt-get install libsoqt4-dev
sudo apt-get install libblas-dev
sudo apt-get install liblapack-dev
sudo apt-get install libqhull-dev
sudo apt-get install libeigen3-dev
```
### Graspit Simulator setup
Follow the given steps to install the graspit simulator. Clone the repository in the home directory.
```
cd 
git clone https://github.com/graspit-simulator/graspit.git
cd graspit
mkdir build
cd build
cmake ..
make -j5
sudo make install
```
To ensure successful installation of Graspit, run the following command and check if the graspit simulator window pops up
```
 ~/graspit/build/graspit_simulator
```
### Graspit Support Packages
In this section, the procedure to build the packages graspit_commander and graspit_interface will be explained. Here it is assumed that a catkin workspace has been completely setup. The mentioned repositories should be cloned in the catkin_ws/src folder.

```
# Clone these repositories inside the src folder
cd ~/catkin_ws/src
git clone https://github.com/graspit-simulator/graspit_interface.git
git clone https://github.com/graspit-simulator/graspit_commander.git

# Build Workspace after cloning the packages
cd ~/catkin_ws
catkin_make

# After successful build of the packages
source devel/setup.bash
```
The graspit_interface package allows working with graspit simulator from ROS (Robotic Operating System), though only C++ language is supported. Incase of python support, the package graspit_commanander allows control over graspit simulator using python.

Once the packages have been installed, Move on to the Script.md file.
Commands to run the interface
```
roslaunch graspt_interface graspit_interface.launch
```
