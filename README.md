# inter_iit_uav_guided_ugv

## Setup Workspace

```
mkdir drdo22_ws
cd drdo22_ws 
mkdir src
catkin build
```
Inside the src folder clone this repository

```
git clone git@github.com:AerialRobotics-IITK/inter_iit_uav_guided_ugv.git
```

---

## Troubleshooting

1. If the models in the first world file aren't loading, make the following changes to the world file.

Previous 

```
<arg name="car_model" default="$(find prius_description)/urdf/prius.urdf"/>
<param name="robot_description" textfile="$(arg car_model)"/>
<include file="$(find gazebo_ros)/launch/empty_world.launch">
<arg name="world_name" value="$(find interiit22)/world/drdo_world1.world"/>
<arg name="gui" value="$(arg gui)"/>
<arg name="verbose" value="true"/>
<!-- more default parameters can be changed here -->
</include>
<!-- vim: set ft=xml noet : -->
```

After
```
<arg name="car_model" default="$(find prius_description)/urdf/prius.urdf"/>
<arg name="ardupilot_gazebo_path" default="~/ardupilot_gazebo"/>
<arg name="verbose" default="true"/>

<env name="GAZEBO_MODEL_PATH"       value="${GAZEBO_MODEL_PATH}:$(find interiit22)/models:$(arg ardupilot_gazebo_path)/models"/>
<env name="GAZEBO_RESOURCE_PATH"    value="${GAZEBO_RESOURCE_PATH}:$(find interiit22)/models:$(arg ardupilot_gazebo_path)/models"/>
<param name="robot_description" textfile="$(arg car_model)"/>
```

2. Depth camera topics not present : Delete **gimbal_small2d** model from the **.gazebo** folder in home.
---

## Running the simulation environment

1. The current files have paths set in accordance with the assumption that you have your ardupilot and ardupilot_gazebo in the home directory. If otherwise, head over to world.launch and change the ardupilot_gazebo_path.

2. Launch QGC and go to the safety settings, scroll to the bottom and uncheck the "all" option in arming safety checks. Just tick the GPS lock option and close

3. Start the launch file world.launch for world1 and in another terminal launch 
```
sim_vehicle.py -v ArduCopter -f gazebo-iris
```
4. Wait for GPS lock, and then launch QGC to manually arm the drone for take-off

---



# Segmentation

## Installation Instructions:

* Install PCL<br/>
    ```
    sudo apt-get update
    sudo apt-get upgrade
    sudo apt install libpcl-dev
    ```

## Running the simulation for mapping and exploration:

1. Launch QGC and go to the safety settings, scroll to the bottom and uncheck the "all" option in arming safety checks. Just tick the GPS lock option and close.

2. Start the launch file world.launch for world1 and in another terminal launch.

```
roslaunch interiit22 ${world}.launch
sim_vehicle.py -v ArduCopter -f gazebo-iris
```

3. Wait for GPS lock, and then launch QGC to manually arm the drone for take-off.

4. In another terminal run the following command to run ```road_seg_node``` and ```mapping_fsm_node```.

```
rosrun segmentation road_seg_node
rosrun mapping mapping_fsm_node
```

## Main Nodes

* road_seg_node
* mapping_fsm_node

## Main Topics

### 1. road_seg_node:
* Subscribed topics:-
    * /depth camera/rgb/image raw: Gets the raw RGB image from the camera
    * /depth camera/depth/points: Gets the pointcloud from the depth camera

* Published topics:-
    * /image/road seg : Image of the segmented road
    * /drone way : The next waypoint in the drone frame

### 2. mapping fsm node
* Subscribed topics:-
    * /drone way : Gets the next waypoint in the drone frame.
    * /mavros/local position/odom : Odometry of the drone
    * /mavros/home position/home : Odometry of the home position
* Published topics:-
    * /mavros/setpoint position/local : For publishing waypoints in the ground frame.