# Team10_uav_guided_ugv

# Installation

## ROS

You can find these installation instructions [here](wiki.ros.org/melodic/Installation/Ubuntu).

#### Setup your sources.list

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

#### Set up your keys

    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

#### Update packages and install ROS

    sudo apt update
    sudo apt install ros-melodic-desktop-full

#### Setup the environment

    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
    source ~/.bashrc

#### Dependencies

    sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

#### Rosdep

    sudo apt install python-rosdep
    sudo rosdep init
    rosdep update

## Ardupilot

### Installing Ardupilot and MAVProxy

#### Clone ArduPilot

In home directory:

```
cd ~
sudo apt install git
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git checkout Copter-3.6
git submodule update --init --recursive
```

#### Install dependencies:

```
sudo apt install python-matplotlib python-serial python-wxgtk3.0 python-wxtools python-lxml python-scipy python-opencv ccache gawk python-pip python-pexpect
```

#### Use pip (Python package installer) to install mavproxy:

```
sudo pip install future pymavlink MAVProxy
```

Open `~/.bashrc` for editing:

```
gedit ~/.bashrc
```

Add these lines to end of `~/.bashrc` (the file open in the text editor):

```
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH
```

Save and close the text editor.

Reload `~/.bashrc`:

```
. ~/.bashrc
```

Run SITL (Software In The Loop) once to set params:

```
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```

## Gazebo and Plugins

#### Gazebo

Setup your computer to accept software from http://packages.osrfoundation.org:

```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```

Setup keys:

```
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

Reload software list:

```
sudo apt update
```

Install Gazebo:

```
sudo apt install gazebo9 libgazebo9-dev
```

### Install Gazebo plugin for APM (ArduPilot Master) :

```
cd ~
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo
git checkout dev
```

build and install plugin

```
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

```
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
```

Set paths for models:

```
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc
. ~/.bashrc
```

#### Run Simulator

In one Terminal (Terminal 1), run Gazebo:

```bash
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world
```

In another Terminal (Terminal 2), run SITL:

```bash
cd ~/ardupilot/ArduCopter/bs
sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```

## Setup Workspace

```bash
mkdir ~/drdo22_ws
cd ~/drdo22_ws
mkdir src
catkin build
```

Inside the src folder clone this repository

```bash
git clone <repo_link>
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

# Road Segmentation

## Installation Instructions:

* Install PCL<br/>
    ```
    sudo apt-get update
    sudo apt-get upgrade
    sudo apt install libpcl-dev
    sudo apt-get install ros-melodic-pcl-ros ros-melodic-pcl-conversions ros-melodic-pcl-msgs
    sudo apt-get install ros-melodic-eigen-conversions
    ```
* Change the path of the generated map csv file at line number 20 in the file named `mapping_fsm` inside `~/drdo22_ws/inter_iit_uav_guided_ugv/segmentation/src/mapping_fsm.cpp`. Set it to `~/drdo22_ws/inter_iit_uav_guided_ugv/maps/<world_name>.csv`.

## Running the simulation for mapping and exploration:

1. Launch QGC and go to the safety settings, scroll to the bottom and uncheck the "all" option in arming safety checks. Just tick the GPS lock option and close.

2. Start the launch file world.launch for world1 and in another terminal launch.

```
roslaunch interiit22 ${world}.launch
sim_vehicle.py -v ArduCopter -f gazebo-iris
```

3. Delete Prius from the world after gazebo is launched.

4. Wait for GPS lock, and then launch QGC to manually arm the drone for take-off.

5. In another terminal run the following command to run ```road_seg_node``` and ```mapping_fsm_node```. Running ```default.launch``` downloads certain modules necessary for running the nodes. 

```
rosrun segmentation road_seg_node
roslaunch segmentation default.launch
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
then launch

# UAV Guided UGV Naviagtion

## Running the simulation environment

1. The current files have paths set in accordance with the assumption that you have your ardupilot and ardupilot_gazebo in the home directory. If otherwise, head over to world.launch and change the ardupilot_gazebo_path. For running the simulation after mapping in `~/drdo22_ws/src/inter_iit_uav_guided_ugv/interiit22/world/drdo_<world-name>.world`, change `model://<world_name>  ` to `model://<world_name>_mesh` at line 19.

2. To start the UGV simulation:
   ```bash
   roslaunch interiit22 drdo_<world_name>.launch
   ```
3. For ArduCopter connection, in another terminal:

   ```bash
   sim_vehicle.py -v ArduCopter -f gazebo-iris
   ```

   Wait until this print on the terminal

   ```
   [ INFO] [1647367092.856149039, 63.285000000]: FCU: EKF2 IMU0 is using GPS
   [ INFO] [1647367092.857438349, 63.286000000]: FCU: EKF2 IMU1 is using GPS
   ```

4. For drone takeoff:

   Launch QGC and go to the safety settings, scroll to the bottom and uncheck the "all" option in arming safety checks. Just tick the GPS lock option and close. Wait for GPS lock, and then launch QGC to manually arm the drone for take-off. QGC automatically hovers the drone at 10m, ensure to manually increse the drone height to 16-18m.

5. For Prius detection and localization, in another terminal run:
   ```bash
   roslaunch prius_detector default.launch
   ```
6. To publish the mean path extracted by the drone, in another terminal run:
   ```bash
   cd ~/drdo22_ws/src/inter_iit_uav_guided_ugv/prius_controller/scripts
   python path_publisher.py
   ```
      It will prompt to enter the world number. Enter 1 or 2.
7. Run the pure persuit controller, in another terminal run:
   ```bash
   cd ~/drdo22_ws/src/inter_iit_uav_guided_ugv/prius_controller/scripts
   python pure_pursuit_ros.py
   ```
8. To run the PID controller and the simulation, in another terminal run:
   ```bash
   cd ~/drdo22_ws/src/inter_iit_uav_guided_ugv/prius_controller/scripts
   python PID_velocity_controller_ROS.py
   ```

## Run Using offboard

This can also be used to run the simulation environment and connect iris to ArduPilot. Also it arms the drone and takes off. (For recordings we have used QGC).

`roslaunch offboard default.launch`

Wait until this print on the terminal

```
[ INFO] [1647367092.856149039, 63.285000000]: FCU: EKF2 IMU0 is using GPS
[ INFO] [1647367092.857438349, 63.286000000]: FCU: EKF2 IMU1 is using GPS
```

---


`roslaunch offboard offboard_node.launch`
