# Using the ATF (by examples)
Auto-generated TOC with https://imthenachoman.github.io/nGitHubTOC/.
- [ATF installation](#atf-installation)
- [Running simple atf test apps](#running-simple-atf-test-apps)
  - [Run the test apps](#run-the-test-apps)
  - [Check test results](#check-test-results)
  - [Visualize the results](#visualize-the-results)
- [Integrate the ATF into your own application](#integrate-the-atf-into-your-own-application)
  - [Python](#python)
  - [C++](#c)
- [Automating ATF test execution](#automating-atf-test-execution)
  - [How to use the ATF with Travis CI](#how-to-use-the-atf-with-travis-ci)
- [How to use the ATF in a "simulation-in-the-loop" setup using [gazebo](http://gazebosim.org/)](#how-to-use-the-atf-in-a-simulation-in-the-loop-setup-using-gazebohttpgazebosimorg)
- [How to use the ATF for benchmarking](#how-to-use-the-atf-for-benchmarking)
- [Manual test execution (without rostest)](#manual-test-execution-without-rostest)
  - [Generation](#generation)
  - [Recording](#recording)
  - [Analysing](#analysing)

## ATF installation
see [ATF installation](Installation.md).

## Running simple atf test apps
### Run the test apps
Run all tests
```
cd ~/catkin_ws
catkin run_tests -j1
```
or run tests from a specific package (e.g. for ```atf_test```):
```
cd ~/catkin_ws
catkin run_tests atf_test -j1
```
or
```
roscd atf_test
catkin run_tests --this -j1
```

### Check test results
check test summary with
```
cd ~/catkin_ws
catkin_test_results
```

If all goes well, you should see the recorded bag files in ```/tmp/atf_test```:
```
ls /tmp/atf_test/data
rosbag info /tmp/atf_test/data/ts0_c0_r0_e0_s0_0.bag
rqt_bag /tmp/atf_test/data/ts0_c0_r0_e0_s0_0.bag
```
and the results in ```/tmp/atf_test/results_txt```:
```
cat /tmp/atf_test/results_txt/atf_result.txt
cat /tmp/atf_test/results_txt/ts0_c0_r0_e0_s0_0.txt
```

### Visualize the results
You an use the ATF plotter to visualize the results:
```
rosrun atf_plotter plot.py --help
```

Example usage:
```
rosrun atf_plotter plot.py plot-benchmark /tmp/atf_test/results_txt/atf_result_aggregated.bag
```

## Integrate the ATF into your own application
### Python
A python app with ATF looks as simple as

```python
import atf_core

class Application:
    def __init__(self):
        # initialize ATF
        self.atf = atf_core.ATF()                  

    def execute(self):
        # you can call start/(pause/purge)/stop for each testblock during the execution of your app

        self.atf.start("testblock_all")
        self.atf.start("testblock_1")

        # Do something

        self.atf.stop("testblock_1")
        self.atf.start("testblock_2")

        # Do something else

        self.atf.stop("testblock_2")
        self.atf.stop("testblock_all")

        # finally we'll have to call shutdown() to tell the ATF to stop all recordings and wrap up
        self.atf.shutdown()

if __name__ == '__main__':
    rospy.init_node('test_app')
    app = Application()
    app.execute()
```

Alongside with the modifications in the app code, you need to create some configuration files. Please have a look at the `atf_test` package within this repository or at the [atf test apps repository](https://github.com/floweisshardt/atf_test_apps) for configuration references. Typically you will have the following files:

```
atf/
├── envs                         # directory for environment specific settings, each environment is configured in one <env_name>.yaml file
│   ├── env1.yaml
│   └── env2.yaml
├── robots                       # directory for robot specific settings, each robot is configured in one <robot_name>.yaml file
│   ├── robot1.yaml
│   └── robot2.yaml
├── testblocksets                # directory for various testblocksets (collection of metrics and groundtruth data)
│   ├── testblockset1.yaml
│   └── testblockset2.yaml
├── tests                        # directory for test configurations, each test is configured in one <test_name>.yaml file
│   ├── test1.yaml
│   └── test2.yaml
└── test_generation_config.yaml  # specifies how to generate test files and defines testsuites (combination of configurations for envs, robots, testblocksets and tests.
```

The last thing to modify is your ```package.xml``` and ```CMakeLists.txt```. Add a test dependency to ```atf_core``` to your ```package.xml```

```
<test_depend>atf_core</test_depend>
```

and the following line to your ```CMakeLists.txt``` which triggers the test generation and execution:

```
if(CATKIN_ENABLE_TESTING)
  find_package(atf_core REQUIRED)
  atf_test(atf/test_generation_config.yaml)
endif()

```

After a ```catkin build --force-cmake``` you will find all the generated test files in the `build` directory of your workspace, e.g. ```build/atf_test/test_generated```.

You can [run the tests automatically](#run-the-test-apps) or [run the tests manually](#manual-test-execution-without-rostest) and finally [check the test results](#check-test-results).

### C++
not yet available

## Automating ATF test execution
If you have your test app written and configuration setup as shown in the [example above](#integrate-the-atf-into-your-own-application), there are new catkin targets which trigger the execution of tests in a package:
* ```catkin atf_<YOUR_PACKAGE>_cleaning ```: cleans all test artefacts (bag, json and yaml files)
* ```catkin atf_<YOUR_PACKAGE>_recording```: triggers the ```cleaning``` target and all recording tests
* ```catkin atf_<YOUR_PACKAGE>_analysing```: triggers the ```recording``` target and all analysing tests
* ```catkin atf_<YOUR_PACKAGE>_uploading```: triggers the ```analysing``` target and the uploading test
* ```catkin atf_<YOUR_PACKAGE>          ```: triggers all tests in your package
* ```catkin run_tests                   ```: triggers all tests in your catkin workspace

You can use the above targets to setup your continuous integration scripts.

### How to use the ATF with Travis CI
An example using [industrial_ci](https://github.com/ros-industrial/industrial_ci) to integrate ATF into [Travis CI](https://travis-ci.com/) can be found in this repository or the [atf test apps repository](https://github.com/floweisshardt/atf_test_apps). Have a look at the ```.travis.yml``` file.

## How to use the ATF in a "simulation-in-the-loop" setup using [gazebo](http://gazebosim.org/)
If you'd like to run tests using gazebo, you just setup a launch file which starts all nodes and include that to your ```application.launch```. As catkin normally uses multiple threads for executing the tests, we'll need to limit that to only one concurrent job as running multiple gazebo instances at a time causes troubles. Thus run your tests with
```
catkin run_tests -j1
```

For github actions or travis scripts using [industrial_ci](https://github.com/ros-industrial/industrial_ci) you can set the ```PARALLEL_TESTS``` environment variable:
```
jobs:
  industrial_ci:
    env:
      PARALLEL_TESTS: 1
```

## How to use the ATF for benchmarking
TBD

## Manual test execution (without rostest)

### Generation
The tests can be generated by calling
```
catkin build --force-cmake atf_test
```

### Recording
Record a single test
```
rosrun atf_core record_tests.py atf_test -t ts0_c0_r0_e0_s0_0
```

Record a subset of tests
```
rosrun atf_core record_tests.py atf_test -t ts0_c0_r0_e0_s0  --> record all iterations of test ts0_c0_r0_e0_s0_*
rosrun atf_core record_tests.py atf_test -t ts0              --> record all tests with ts0: ts0_c*_r*_e*_s*_*
rosrun atf_core record_tests.py atf_test -t c0*r0*           --> record all tests with c0 and r0: ts*_c0_r0_s*_*
```

Record all tests
```
rosrun atf_core record_tests.py atf_test
```

Full list of arguments
```
rosrun atf_core record_tests.py -h
```

While tests are executed:

- you can watch the progress with
```
rosrun smach_viewer smach_viewer.py
```
- you can use RVIZ for visualization
```
rosrun rviz rviz
```

### Analysing
```
rosrun atf_core analyser.py atf_test
```
or for full result print
```
rosrun atf_core analyser.py atf_test -v
```

Full list of arguments
```
rosrun atf_core analyser.py -h
```

check results
```
cat /tmp/atf_test/results_txt/atf_result.txt
cat /tmp/atf_test/results_txt/ts0_c0_r0_e0_s0_0.txt
```
