# Installation
## Installation from release
not yet available

## Installing from source
### Prerequisites:

* Install Ubuntut 16.04 LTS "xenial" (either fresh from an image or using dist-ugrade)
* Install ROS Kinetic (ros-kinetic-ros-base): http://wiki.ros.org/kinetic/Installation/Ubuntu
* Install additional tools: sudo apt-get install git python-wstool python-ros* python-catkin-tools build-essential 

### Create empty catkin workspace
Note: adjust workspace directory if needed
```
source /opt/ros/kinetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin build
```

### Get ATF sources from github
```
cd ~/catkin_ws/src
git clone https://github.com/floweisshardt/atf
wstool merge -y https://raw.githubusercontent.com/floweisshardt/atf/master/.travis.rosinstall
wstool update
```

### Get dependendies
Note: you need sudo rights!
```
source ~/catkin_ws/devel/setup.bash
sudo rosdep init
rosdep update
cd ~/catkin_ws
rosdep install --from-path src -i -y
```

### Compile sources
```
cd ~/catkin_ws
catkin build
```

### Source setup.bash
Note: You might want to add that to your `~/.bashrc`
```
source ~/catkin_ws/devel/setup.bash
```

## Testing ATF
[Test on local computer](Examples.md)


