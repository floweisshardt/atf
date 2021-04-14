# The Automated Test Framework (ATF)

CI-Status ```master```: [![Build Status](https://github.com/floweisshardt/atf/workflows/CI/badge.svg?branch=master)](https://github.com/floweisshardt/atf/actions/workflows/main.yml?query=branch%3Amaster)

CI-Status ```atf_test_apps/master```: [![Build Status](https://github.com/floweisshardt/atf_test_apps/workflows/CI/badge.svg?branch=master)](https://github.com/floweisshardt/atf_test_apps/actions/workflows/main.yml?query=branch%3Amaster)

The ATF is a testing framework written for [ROS](http://www.ros.org/) which supports executing integration and system tests, running benchmarks and monitor the code behaviour over time. The ATF provides basic building blocks for easy integration of the tests into your application. Furthermore the ATF provides everything to automate the execution and analysis of tests as well as a graphical web-based frontend to visualize the results.

Auto-generated TOC with https://imthenachoman.github.io/nGitHubTOC/.
- [Architecture](#architecture)
  - [Overview](#overview)
  - [Recording data](#recording-data)
  - [Analysing metrics](#analysing-metrics)
  - [Visualising results](#visualising-results)
- [Implemented metrics](#implemented-metrics)
- [Installation](#installation)
- [Using the ATF (by examples)](#using-the-atf-by-examples)
- [Contributing to the ATF](#contributing-to-the-atf)
- [Acknowledgements](#acknowledgements)

## Architecture
### Overview
There is a [presentation](doc/data/ATF_Intro.pdf) for a brief ATF introduction.
### Recording data
### Analysing metrics
### Visualising results
## Implemented metrics
The following metrics have been implemented so far:

| Metric        | Description   | Unit  | Mode (span, snap) |
|:-------------:|:--------------|:-----:|:--------------------------:|
| ```interface``` | The ```interface``` metric checks if an interfaces (nodes, publishers, subscribers, service servers, action servers) matches its specification. | [bool] | snap |
| ```publish_rate``` | The ```publish_rate``` metric measures the publising rate of a topic   | [1/sec] | span |
| ```tf_acceleration_translation```   | The ```tf_acceleration_translation``` metric measures the cartesian acceleration of a TF frame with respect to another frame.    |  [m/s^2] |  span |
| ```tf_distance_rotation```      | The ```tf_distance_rotation``` metric measures the cartesian angular distance of a TF frame with respect to another frame at the end of a testblock.    |  [rad] | snap |
| ```tf_distance_translation```   | The ```tf_distance_translation``` metric measures the cartesian distance of a TF frame with respect to another frame at the end of a testblock.    |  [m] | snap |
| ```tf_jerk_translation```   | The ```tf_jerk_translation``` metric measures the cartesian jerk of a TF frame with respect to another frame.    |  [m/s^3] |  span |
| ```tf_length_rotation```        | The ```tf_length_rotation``` metric measures the cartesian angular path (angular distance integrated over time) of a TF frame with respect to another frame.    |  [rad] | span |
| ```tf_length_translation```     | The ```tf_length_translation``` metric measures the cartesian path (distance integrated over time) of a TF frame with respect to another frame.    |  [m] | span |
| ```tf_velocity_translation```   | The ```tf_velocity_translation``` metric measures the cartesian velocity of a TF frame with respect to another frame.    |  [m/s] |  span |
| ```time```    | The ```time``` metric measures the elapsed time. | [sec] | span |
| ```topic_data```      | The ```topic_data``` metric fetches a numerical data filed from a message    |  [any] | span, snap |
| ```user_result```      | The result for the ```user_result``` metric can be set from the user within the `application.py`.    |  [any] | span, snap |
See full list of implemented metrics at https://github.com/floweisshardt/atf/tree/master/atf_metrics.

Further metrics (in development):

| Metric        | Description   | Unit  | Mode (span, snap) |
|:-------------:|:--------------|:-----:|:--------------------------:|
| ```resources```    | The ```resources``` metric measures the resource consumption of a node on the operating system level (CPU, RAM, IO). | [%], [MB], [MB/sec] | snap |
| ```obstacle_distance``` | The ```obstacle_distance``` metric measures the distance between two meshes   | [m] | snap |
| ```message_match``` | The ```message_match``` metric checks if a message content matches its desired content. | [bool] | snap |

## Installation
For installation instruction see [ATF Installation](doc/Installation.md).

## Using the ATF (by examples)
For examples how to use the ATF see [ATF Examples](doc/Examples.md)

## Contributing to the ATF
For examples how to extend the ATF with your own contribution see [ATF Contribution](doc/Contribution.md)

## Acknowledgements
The work leading to these results has received funding from the European Community's Seventh Framework Program (FP7/2007-2013) under grant agreement no 609206 [Factory-in-a-Day](http://www.factory-in-a-day.eu/) and the German Federal Ministry for Economic Affairs and Energy under grant agreement no 01MA13001A [ReApp](http://www.reapp-projekt.de/).
