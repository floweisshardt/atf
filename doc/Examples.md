# Using the ATF (by examples)
### Running simple atf test apps
###### Download and build test apps
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

###### Run the test apps
Run all tests:
```
cd ~/catkin_ws
catkin_make run_tests
```
or just tests from a specific package (e.g. for ```atf_test_app_time```):
```
cd ~/catkin_ws
catkin_make atf_atf_test_app_time
```

###### Check and visualize the results
If all goes well, you should see the recorded bag files in ```/tmp/atf_test_app_time```:
```
ls /tmp/atf_test_app_time/data
rosbag info /tmp/atf_test_app_time/ts0_c0_r0_e0_0.bag
```
and the (merged) results in ```/tmp/atf_test_app_time/results_yaml```:
```
cat /tmp/atf_test_app_time/results_yaml/merged_ts0_c0_r0_e0.yaml
```
You an use the ATF presenter to visualize the results in a webbrowser:
```
rosrun atf_presenter chromium.sh
```
You should now see 

SCREENSHOT1

Please select the file ```test_list.json``` and all ```merged_*.json``` files out of the ```results_json``` directory.

SCREENSHOT2

If all results could be loaded successfully you can press on the 'Details' button to see the test details.

SCREENSHOT3

Now for all analyzed metrics you will see a diagramm showing the average results, the min/max deviation and the allowed groundtruth tollerances.

SCREENSHOT4

### Integrate the ATF into your own application
### How to use the ATF in a "simulation-in-the-loop" setup using [gazebo](http://gazebosim.org/)
### How to use the ATF for benchmarking
