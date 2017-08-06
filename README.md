# UAV Challenge Landing Module (WIP)

This module contains the landing target searching and recognition module for the UAV Challenge (2017). 

## Prerequisities

* Ubuntu 16.04
* ROS Kinetic
* PX4 Firmware
* OpenCV 3
* MAVROS

## Installation Instructions

1. Install Ubuntu, ROS Kinetic, and catkin tools, see [here](https://github.com/eric1221bday/CMU_Quadcopter_Documentation/blob/master/ROSInstallCMU.md) for instructions for Windows and Mac. If Ubuntu is already installed, refer only to the portion for installing ROS and catkin tools.

2. Install mavros if it isn't installed yet

```bash
sudo apt update
sudo apt install ros-kinetic-mavros-extras
```

3. Setup the catkin workspace and download the necessary packages

```bash
mkdir -p ~/sandbox/uav_challenge/src
cd ~/sandbox/uav_challenge
catkin init
cd src
git clone https://github.com/PX4/Firmware.git --recursive
git clone https://github.com/eric1221bday/sitl_gazebo.git --recursive
git clone https://github.com/eric1221bday/landing_module.git
```

4. Build the modules

```bash
cd ~/sandbox/uav_challenge
catkin build
```

## Usage Instructions (Gazebo Simulation)

```bash
source ~/sandbox/uav_challenge/devel/setup.bash
roslaunch landing_module sitl_gazebo.launch
```

### Notes

* In `sitl_gazebo.launch`, there are options to switch the aircraft used to a VTOL instead of the hexacopter by switching the commented out `rcS` and `vehicle` arguments. The VTOL gimbal is currently **nonfunctional**.

* In order to change the location of the target within the simulation, edit the world file in `~/sandbox/uav_challenge/src/sitl_gazebo/worlds/uav_challenge.world`. The position of the target can be specified in the `<pose>-50 -30 0.01 0 0 0</pose>` clause above `<uri>model://landing_target</uri>`. Units in meters and degres and ordering is (x y z roll pitch yaw)

## Relevant Parameters

* `h_fov`: horizontal FOV of the camera used in degrees, default is 90

* `search_altitude`: altitude maintained by aircraft during search in meters, default is 50

* `search_position_x, search_position_y`: coordinate of the center of the search pattern in meters, default is (0,0)
 