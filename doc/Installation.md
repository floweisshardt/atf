# ATF installation
Auto-generated TOC with https://imthenachoman.github.io/nGitHubTOC/.
- [Installation from release](#installation-from-release)
- [Installation from source](#installing-from-source)
  - [Prerequisites:](#prerequisites)
  - [Create empty catkin workspace](#create-empty-catkin-workspace)
  - [Get atf sources](#get-atf-sources)
  - [Get atf_test_apps sources (optional)](#get-atf_test_apps-sources-optional)
  - [Get dependendies](#get-dependendies)
  - [Compile sources](#compile-sources)
  - [Source setup.bash](#source-setupbash)
- [Running ATF tests](#running-atf-tests)

## Installation from release
not yet available

## Installation from source
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

### Get atf sources
```
cd ~/catkin_ws/src
git clone https://github.com/floweisshardt/atf
```

### Get atf_test_apps sources (optional)
For each [implemented metric](../README.md#implemented-metrics) there is a test app package that uses the metric in a simple application. You can find all the test apps in the [atf_test_apps](https://github.com/floweisshardt/atf_test_apps) repository.

```
cd ~/catkin_ws/src
git clone https://github.com/floweisshardt/atf_test_apps
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
catkin build --force-cmake
```

### Source setup.bash
Note: You might want to add that to your `~/.bashrc`
```
source ~/catkin_ws/devel/setup.bash
```

## Running ATF tests
see [ATF examples](Examples.md).


