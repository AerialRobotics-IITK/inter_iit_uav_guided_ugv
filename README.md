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

## Run Using offboard

`roslaunch offboard default.launch`

Wait until this print on the terminal

```
[ INFO] [1647367092.856149039, 63.285000000]: FCU: EKF2 IMU0 is using GPS
[ INFO] [1647367092.857438349, 63.286000000]: FCU: EKF2 IMU1 is using GPS
```

then launch

`roslaunch offboard offboard_node.launch`

##  Running the prius_controller
Change the path of file world1.csv (/prius_controller/testing/world1.csv) in the file straight_file.py in the line 82 corresponding to the location in your PC
