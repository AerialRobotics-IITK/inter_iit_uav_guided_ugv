# inter_iit_uav_guided_ugv

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

```
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world
```

In another Terminal (Terminal 2), run SITL:

```
cd ~/ardupilot/ArduCopter/
sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```

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

2. To start the simulation

   ```bash
   r
   ```

   

3. Launch QGC and go to the safety settings, scroll to the bottom and uncheck the "all" option in arming safety checks. Just tick the GPS lock option and close

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

## Running the prius_controller

Change the path of file world1.csv `(/prius_controller/testing/world1.csv)` in the file straight_file.py in line 83 corresponding to the location in your PC.

Run the following python scripts in the written sequence

```bash
python straight_path.py
python pure_pursuit_ros.py
python PID_velocity_controller_ROS.py
```

## Saving Odometry

To save the odometry of the vehicle run the script present in `prius_controller/testing/odometry`

```bash
python save_odometry.py
```

## Plotting the followed path

Before running install the following dependencies

```bash
pip install pandas
pip install matplotlib
sudo apt-get install python3-tk
```

Then run the following script present in `prius_controller/testing/plot.py`

```bash
python3 plot.py
```
