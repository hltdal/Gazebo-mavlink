# Installing Ardupilot and MAVProxy Ubuntu 20.04

**Main link => https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/Installing_Ardupilot_20_04.md**

**Video Tutorial at https://youtu.be/1FpJvUVPxL0**

**Pay attention to the environment in which the operations are performed while watching the video.**

## Clone ArduPilot

**In home directory:**

cd ~

sudo apt install git

git clone https://github.com/ArduPilot/ardupilot.git

cd ardupilot

### Install dependencies:

cd ardupilot

Tools/environment_install/install-prereqs-ubuntu.sh -y

**Use pip (Python package installer) to install mavproxy:**

sudo pip install future pymavlink MAVProxy

gedit ~/.bashrc

**Add these lines to end of ~/.bashrc (the file open in the text editor):**

export PATH=$PATH:$HOME/ardupilot/Tools/autotest

export PATH=/usr/lib/ccache:$PATH

**Save and close the text editor.**

Reload ~/.bashrc:

. ~/.bashrc

reload profile

. ~/.profile

## Checkout Latest Copter Build
**https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/ReleaseNotes.txt**

**Find the last version at link above(if 4.5.7 beta version then take version 4.5.6)**

git checkout Copter-4.5.6

git submodule update --init --recursive

## Run SITL (Software In The Loop) once to set params:

cd ~/ardupilot/ArduCopter

sim_vehicle.py -w

**If it launchs without any problem you can exit all command prompt**

# Installing Gazebo and ArduPilot Plugin
**Main links => https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_gazebo_arduplugin.md**

**Video Tutorial at https://youtu.be/m7hPyJJmWmU**

## Install Gazebo [18.04-20.04]

sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

sudo apt update

sudo apt-get install gazebo11 libgazebo11-dev

**for more detailed instructions for installing gazebo checkout http://gazebosim.org/tutorials?tut=install_ubuntu**

## Install Gazebo plugin for APM (ArduPilot Master) :

cd ~

git clone https://github.com/khancyr/ardupilot_gazebo.git

cd ardupilot_gazebo

mkdir build

cd build

cmake ..

make -j4

sudo make install


echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc

echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc

. ~/.bashrc

# Install ROS and Setup Catkin

**Main link => https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_ros_20_04.md***

**Video Tutorial at https://youtu.be/1FpJvUVPxL0**

## Install ROS

**Install ROS Noetic using the following instructions: http://wiki.ros.org/noetic/Installation/Ubuntu**

**Do Desktop-full Install**

# 

**Set Up Catkin workspace**

sudo apt-get install python3-wstool python3-rosinstall-generator python3-catkin-lint python3-pip python3-catkin-tools

pip3 install osrf-pycommon

mkdir -p ~/catkin_ws/src

cd ~/catkin_ws

catkin init

**Dependencies installation**

cd ~/catkin_ws

wstool init ~/catkin_ws/src

rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall

rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall

wstool merge -t src /tmp/mavros.rosinstall

wstool update -t src

rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y

catkin build

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

source ~/.bashrc

sudo ~/catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh

**Clone IQ Simulation ROS package**

cd ~/catkin_ws/src

git clone https://github.com/Intelligent-Quads/iq_sim.git

echo "GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/catkin_ws/src/iq_sim/models" >> ~/.bashrc

**Build instructions**

cd ~/catkin_ws

catkin build

source ~/.bashrc

# Introduction to Ros for Autonomous Drones

**https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/ros_intro.md**

**Video Tutorial at https://youtu.be/N4XvVldWlXk**

**Make sure Install ROS plugins for Gazebo:**

sudo apt install ros-melodic-gazebo-ros ros-melodic-gazebo-plugins

# 

### Launch Gazebo World

roslaunch iq_sim runway.launch

### Launch Arducopter simulator in new terminal

cp ~/catkin_ws/src/iq_sim/scripts/startsitl.sh ~

~/startsitl.sh

### Intoduction to ROS Commandline Tools

rostopic list

**you should see some gazebo topics**

**now you can give orders to drone through follow steps in video**

## Using MAVROS to get telemetry data from the FCU

roslaunch iq_sim apm.launch

**when you run "rostopic list" you should see a bunch of mavros topics**




# All sources are given below

**https://www.youtube.com/watch?v=EmIjedzHwzI&list=PLy9nLDKxDN683GqAiJ4IVLquYBod_2oA6&index=5&ab_channel=IntelligentQuads**

https://www.youtube.com/watch?v=1FpJvUVPxL0&ab_channel=IntelligentQuads

https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/Installing_Ardupilot.md

https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/Installing_Ardupilot_20_04.md

https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_gazebo_arduplugin.md

https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_ros_20_04.md

https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/ros_intro.md
