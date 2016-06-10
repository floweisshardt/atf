# Installation
### Installation from release
not yet available

### Installing from source

1. Prerequisites:
  * Install Ubuntut 14.04 LTS "trusty" (either fresh from an image or using dist-ugrade)
  * Install ros-indigo-dektop-full as described here: http://wiki.ros.org/indigo/Installation/Ubuntu)
  * Install additional tools: sudo apt-get install ros-indigo-ros python-wstool

1. Create catkin workspace

```
source /opt/ros/indigo/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_init_workspace src
wstool init src
catkin_make
```

1. Get ATF sources from github
```
cd ~/catkin_ws/src
wstool merge -y https://raw.githubusercontent.com/ipa-fmw/atf/master/.travis.rosinstall
wstool update
```

1. Get dependendies
(Note: you need sudo rights!)
```
sudo rosdep init
rosdep update
cd ~/catkin_ws
rosdep install --from-path src -i -y
```

1. Compile sources
```
cd ~/catkin_ws
catkin_make
```

1. Source setup.bash
```
source ~/catkin_ws/devel/setup.bash
```

