# Installing Ardupilot and MAVProxy Ubuntu 20.04

**Main link => https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/Installing_Ardupilot_20_04.md**

**Video Tutorial at https://youtu.be/1FpJvUVPxL0**

**Pay attention to the environment in which the operations are performed while watching the video.**

## Clone ArduPilot

**In home directory:**
```
cd ~
sudo apt install git
```
**If you are installing for the first time, go to config: (git config --global user.name "GitHubKullaniciAdi") , (git config --global user.email "email@example.com")**
```
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
```
### Install dependencies:
```
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
```
**If the above code fails, run it again**

**Reload ~/.bashrc and profile**
```
. ~/.profile
```
## Checkout Latest Copter Build
**https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/ReleaseNotes.txt**

**Find the last version at link above(if 4.5.7 beta version then take version 4.5.6)**
```
cd ardupilot
git checkout Copter-4.5.6
git submodule update --init --recursive

sudo apt install python-matplotlib python-serial python-wxgtk3.0 python-wxtools python-lxml python-scipy python-opencv ccache gawk python-pip python-pexpect

sudo pip install future pymavlink MAVProxy
gedit ~/.bashrc

#Add these lines to end of ~/.bashrc (the file open in the text editor):
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH

. ~/.bashrc
```
## Run SITL (Software In The Loop) once to set params:
```
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```
**If it launchs without any problem you can exit all command prompt**

# Installing Gazebo and ArduPilot Plugin
**Main links => https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_gazebo_arduplugin.md**

**Video Tutorial at https://youtu.be/1FpJvUVPxL0**

## Install QGroundControl 

```
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
```
To install QGroundControl:
```
https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
```

Install (and run) using the terminal commands:

```
chmod +x ./QGroundControl.AppImage
./QGroundControl.AppImage
(or double click)
```
Run SITL and connect with Q Ground
```
cd ~/ardupilot/ArduCopter/
sim_vehicle.py
```
(bir konsolda sim vehicle diğer konsolda ./Qground çalışırsa otomatik bağlanıyorlar. Denemek istersen sürmek istersen oynayabilirsin)

## Install Gazebo [18.04-20.04]
```
sudo apt update
sudo apt install curl 
curl --version
```
```
sudo apt-get update
sudo apt-get install lsb-release gnupg
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
curl https://packages.osrfoundation.org/gazebo.key | sudo apt-key add -
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```
```
sudo apt update
sudo apt-get install gazebo11 libgazebo11-dev
```
**For more detailed instructions for installing gazebo checkout http://gazebosim.org/tutorials?tut=install_ubuntu**

## Install Gazebo plugin for APM (ArduPilot Master) :
```
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
```
# Install ROS and Setup Catkin

**Main link => https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_ros_20_04.md***

**Video Tutorial at https://youtu.be/1FpJvUVPxL0**

## Install ROS

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full

source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```
## Set Up Catkin workspace
```
sudo apt-get install python3-wstool python3-rosinstall-generator python3-catkin-lint python3-pip python3-catkin-tools

pip3 install osrf-pycommon
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
```
**Dependencies installation**
```
cd ~/catkin_ws
wstool init ~/catkin_ws/src

rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src

sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y

catkin build
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo ~/catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```
**Clone IQ Simulation ROS package**
```
cd ~/catkin_ws/src
git clone https://github.com/Intelligent-Quads/iq_sim.git
echo "GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/swarm-2025-backend/catkin_ws/src/iq_sim/models" >> ~/.bashrc
```
**Build instructions**
```
cd ~/catkin_ws
catkin build
source ~/.bashrc
```
# Introduction to Ros for Autonomous Drones

**https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/ros_intro.md**

**Video Tutorial at https://youtu.be/1FpJvUVPxL0**

**Make sure Install ROS plugins for Gazebo:**

### Launch Gazebo World
```
roslaunch iq_sim runway.launch
```
### Launch Arducopter simulator in new terminal
```
cp ~/catkin_ws/src/iq_sim/scripts/startsitl.sh ~
~/startsitl.sh
```
```
bunları çalıştırdıktan sonra apm.launch'daki bağlantı
"<arg name="fcu_url" default="udp://127.0.0.1:14550@5760" />"
satırını yukarıdaki gibi yaptım ve çalıştırdım
MAVROS kendi 14550 portundan ArduPilot'un 5760 portuna bağlanır.
sim_vehicle.py -v ArduCopter -f gazebo-drone2 -I0
bu komutu yazınca 
SIM_VEHICLE: "mavproxy.py" "--out" "127.0.0.1:14550" "--master" "tcp:127.0.0.1:5760" "--sitl" "127.0.0.1:5501" "--console"
diye bir mesaj verir. Buradan mavrosun ve ardupilotun hangi portları kullandığını görebilirsin
```
**Now you can give orders to drone through follow steps in video**

# Guidance Navigation and Control

**https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/gnc_tutorial.md**

**Video Tutorial Part 1 at https://youtu.be/eRAfeC8OFfs**
```
cd catkin_ws/src
git clone https://github.com/Intelligent-Quads/iq_gnc.git

catkin build
source ~/.bashrc
```
```
Run example code
roslaunch iq_sim runway.launch
# New Terminal
./startsitl.sh
# New Terminal
roslaunch iq_sim apm.launch
# New Terminal 
rosrun iq_gnc square
```
# Swarming Using Ardupilot

**https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/swarming_ardupilot.md**

**Video Tutorial at https://youtu.be/r15Tc6e2K7Y**

**Add the models folder in the iq_sim repo to the gazebo models path**
```
echo "export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/catkin_ws/src/iq_sim/models" >> ~/.bashrc
. ~/.bashrc
```
## Connecting Multiple Vehicles SITL to Gazebo

**When you launch the world**
```
roslaunch iq_sim multi_drone.launch
```
**Should see the world with three drones in it**

**Launch ardupilot terminals**
```
sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I0
sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I1
sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I2
```
şimdi konsollardan drone lara teker teker emir verebilirsin

örnek emir:

mode guided

arm throttle

takeoff 10

**It is recommended that you watch the video for a sample simulation (6.15 minutes)**

# Launching Ardupilot SITL Instances with Unique Parameters

**Video Tutorial at https://www.youtube.com/watch?v=UWsya46ZG4M&list=PLy9nLDKxDN683GqAiJ4IVLquYBod_2oA6&index=21&ab_channel=IntelligentQuads**

**First, we will want to edit the file `ardupilot/Tools/autotest/pysim/vehicleinfo.py` add the following lines in the SIM section.**
```
"gazebo-drone1": {
    "waf_target": "bin/arducopter",
    "default_params_filename": ["default_params/copter.parm",
                                "default_params/gazebo-drone1.parm"],
},
"gazebo-drone2": {
    "waf_target": "bin/arducopter",
    "default_params_filename": ["default_params/copter.parm",
                                "default_params/gazebo-drone2.parm"],
},
"gazebo-drone3": {
    "waf_target": "bin/arducopter",
    "default_params_filename": ["default_params/copter.parm",
                                "default_params/gazebo-drone3.parm"],
},
```

```
cd ~/ardupilot/Tools/autotest/default_params
```
**We will then need to create the following files**

`default_params/gazebo-drone1.parm`

`default_params/gazebo-drone2.parm`

`default_params/gazebo-drone3.parm`

For drone1
```
nano gazebo-drone1.parm
```
**Each with their corresponding SYSID_THISMAV parameter value ie**

default_params/gazebo-drone1.parm should contain `SYSID_THISMAV 1`

default_params/gazebo-drone2.parm should contain `SYSID_THISMAV 2`

default_params/gazebo-drone3.parm should contain `SYSID_THISMAV 3`
```
# Iris is X frame
FRAME_CLASS 1
FRAME_TYPE  1
# IRLOCK FEATURE
RC8_OPTION 39
PLND_ENABLED    1
PLND_TYPE       3
# SONAR FOR IRLOCK
SIM_SONAR_SCALE 10
RNGFND1_TYPE 1
RNGFND1_SCALING 10
RNGFND1_PIN 0
RNGFND1_MAX_CM 5000
SYSID_THISMAV 1
```
# Install Github Simulation Repo
```
git clone https://github.com/hltdal/Gazebo-mavlink.git
```
**Change multi_drone.launch**
```
cd catkin_ws/src/iq_sim/launch/
nano multi_drone.launch

<launch>
  <!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find iq_sim)/worlds/multi_drone.world"/>
    <!-- more default parameters can be changed here -->
  </include>
</launch>
```
**Save multi_drone.launch and close**

**Change multi_drone.world**
```
cd catkin_ws/src/iq_sim/worlds
nano multi_drone.world
```
**Replace drones part with**  
```
<model name="drone1">
  <pose> 0 0 0 0 0 0</pose>
  <include>
    <uri>model://drone1</uri>
  </include>
</model>
<model name="drone2">
  <pose> 2 0 0 0 0 0</pose>
  <include>
    <uri>model://drone2</uri>
  </include>
</model>
<model name="drone3">
  <pose> 4 0 0 0 0 0</pose>
  <include>
    <uri>model://drone3</uri>
  </include>
</model>
```
```
cd catkin_ws
catkin build iq_gnc
```
### Install Tmux

```
sudo apt-get update 
sudo apt-get install tmux.
```
**Create multi-sitl**
```
cd ~
nano multi_sitl.sh
```
```
#!/bin/bash

tmux new-session -d -s sitl_session 'sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I0'
tmux split-window -v -t sitl_session 'sim_vehicle.py -v ArduCopter -f gazebo-drone2 -I1'
tmux split-window -h -t sitl_session 'sim_vehicle.py -v ArduCopter -f gazebo-drone3 -I2'
tmux -2 attach-session -t sitl_session
```
```
chmod +x multi_sitl.sh
```
**Run the program**
```
cd Gazebo-mavlink/gazebo mavlink 
python3 main_gui.py
```

# All sources are given below

**https://www.youtube.com/playlist?list=PLy9nLDKxDN683GqAiJ4IVLquYBod_2oA6**

https://www.youtube.com/watch?v=1FpJvUVPxL0&ab_channel=IntelligentQuads

https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/Installing_Ardupilot.md

https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/Installing_Ardupilot_20_04.md

https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_gazebo_arduplugin.md

https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_ros_20_04.md

https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/ros_intro.md
