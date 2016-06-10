# The Automated Test Framework (ATF)

CI-Status ```master```: [![Build Status](https://travis-ci.org/ipa-fmw/atf.svg?branch=master)](https://travis-ci.org/ipa-fmw/atf)

CI-Status ```stable```: [![Build Status](https://travis-ci.org/ipa-fmw/atf.svg?branch=stable)](https://travis-ci.org/ipa-fmw/atf)

CI-Status ```atf_test_apps/master```: [![Build Status](https://travis-ci.org/ipa-fmw/atf_test_apps.svg?branch=master)](https://travis-ci.org/ipa-fmw/atf_test_apps)

The ATF is a testing framework written for [ROS](http://www.ros.org/) which supports executing integration and system tests, running benchmarks and monitor the code behaviour over time. The ATF provides basic building blocks for easy integration of the tests into your application. Furthermore the ATF provides everything to automate the execution and analysis of tests as well as a graphical web-based frontend to visualize the results.

## Architecture
### Overview
### Recording data
### Analysing metrics
### Visualising results
### Implemented metrics
The following metrics have been implemented so far:

| Metric        | Description   | Unit  | Mode (span, snap, min/max) |
|:--------------|:--------------|:-----:|:--------------------------:|
| ```time```    | The ```time``` metric measures the elapsed time. | [sec] | span |
| ```path_length```      | The ```path_length``` metric measures the cartesian path (distance integrated over time) of a TF frame with respect to another frame.    |  [m] | span |
| ```publish_rate``` | The ```publish_rate``` metric measures the publising rate of a topic   | [1/sec] | span |
| ```api``` | The ```api``` metric checks if an interfaces (nodes, publishers, subscribers, service servers, action servers) matches its specification. | [bool] | snap |

Further metrics (in development):

| Metric        | Description   | Unit  | Mode (span, snap, min/max) |
| :------------ |:--------------| :----:|:--------------------------:|
| ```ressources```    | The ```ressources``` metric measures the ressource consumption of a node on the operating system level (CPU, RAM, IO). | [%], [MB], [MB/sec] | snap |
| ```path_velocity```      | The ```path_velocity``` metric measures the cartesian velocity (distance differntiated over time) of a TF frame with respect to another frame.    |  [m/sec] | span |
| ```distance```      | The ```distance``` metric measures the cartesian distance between two TF frames.    |  [m] | snap, min/max |
| ```obstacle_distance``` | The ```obstacle_distance``` metric measures the distance between two meshes   | [m] | snap, min/max |
| ```message_match``` | The ```message_match``` metric checks if a message content matches its desired content. | [bool] | snap |

## Using the ATF (by examples)
### Basic installation
Installing from source

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

### Running simple atf test apps
For each [implemented metric](https://github.com/ipa-fmw/atf#implemented-metrics) there is a test app package that uses the metric in a simple application. You can find all the test apps in the [atf_test_apps](https://github.com/ipa-fmw/atf_test_apps) repository.

1. Get sources
```
cd ~/catkin_ws/src
wstool set atf_test_apps --git https://github.com/ipa-fmw/atf_test_apps.git
wstool update
```

1. Build and generate tests
```
cd ~/catkin_ws
catkin_make --force-cmake
```

1. Run the tests
To run all tests:
```
cd ~/catkin_ws
catkin_make run_tests
```
or just tests from a specific package (e.g. for ```atf_test_app_time```):
```
cd ~/catkin_ws
catkin_make atf_atf_test_app_time
```

1. Check the results
If all goes well, you should see the recorded bag files in ```/tmp/atf_test_app_time```:
```
ls /tmp/atf_test_app_time/data
rosbag info /tmp/atf_test_app_time/ts0_c0_r0_e0_0.bag
```
and the (merged) results in ```/tmp/atf_test_app_time/results_yaml```:
```
cat /tmp/atf_test_app_time/results_yaml/merged_ts0_c0_r0_e0.yaml
```

1. Visualize the results
You an use the ATF presenter to visualize the results in a webbrowser:
```
rosrun atf_presenter chromium.sh
```
Please select the file ```test_list.json``` and all ```merged_*.json``` files out of the ```results_json``` directory.

SCREENSHOT


### Integrate the ATF into your own application
### How to use the ATF in a "simulation-in-the-loop" setup using [gazebo](http://gazebosim.org/)
### How to use the ATF for benchmarking

## Contributing to the ATF
### Extend the ATF with your own metric

## Acknowledgments
The work leading to these results has received funding from the European Community's Seventh Framework Program (FP7/2007-2013) under grant agreement no 609206 [Factory-in-a-Day](http://www.factory-in-a-day.eu/) and the German Federal Ministry for Economic Affairs and Energy under grant agreement no 01MA13001A [ReApp](http://www.reapp-projekt.de/).
