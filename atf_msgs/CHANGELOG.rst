^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package atf_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2024-02-21)
------------------
* Merge pull request `#181 <https://github.com/floweisshardt/atf/issues/181>`_ from floweisshardt/feature/add_units
  add units
* add units
* Merge pull request `#170 <https://github.com/floweisshardt/atf/issues/170>`_ from floweisshardt/feature/actions
  noetic with updated action config
* roscompile noetic_migration
* Merge pull request `#150 <https://github.com/floweisshardt/atf/issues/150>`_ from floweisshardt/fix/user_result
  rework groundtruth handling
* use enum for result
* remove unused action
* Merge pull request `#14 <https://github.com/floweisshardt/atf/issues/14>`_ from floweisshardt/master
  sync from floweisshardt/master
* Merge pull request `#149 <https://github.com/floweisshardt/atf/issues/149>`_ from floweisshardt/feature/tf_velocity
  new tf metrics
* introduce SPAN_MEAN, SPAN_MAX, SPAN_ABSMAX, SPAN_MIN, SPAN_ABSMIN
* Merge pull request `#145 <https://github.com/floweisshardt/atf/issues/145>`_ from floweisshardt/feature/atf_plotter
  more features for atf_plotter
* add name of atf package to atf result
* Merge pull request `#140 <https://github.com/floweisshardt/atf/issues/140>`_ from floweisshardt/feature/atf_plotter
  Feature/atf plotter
* Merge pull request `#143 <https://github.com/floweisshardt/atf/issues/143>`_ from floweisshardt/feature/merger
  merge results
* merge results
* fix Travis
* make groundtruth a standalone message and separate submetrics with unique names
* add groundtruth available to MetricResult
* Merge pull request `#9 <https://github.com/floweisshardt/atf/issues/9>`_ from floweisshardt/master
  sync from floweisshardt/master
* Merge pull request `#113 <https://github.com/floweisshardt/atf/issues/113>`_ from fmessmer/ci_updates
  [travis] ci updates
* fix test dependendies
* catkin_lint fixes
* Merge pull request `#110 <https://github.com/floweisshardt/atf/issues/110>`_ from floweisshardt/feature/data_series
  use data series
* series_mode for all metrics
* add min/max/mean/std
* use data series
* Merge pull request `#89 <https://github.com/floweisshardt/atf/issues/89>`_ from floweisshardt/benchmarking
  prepare for benchmarking
* prepare for benchmarking with new testblocksets
* Merge pull request `#85 <https://github.com/floweisshardt/atf/issues/85>`_ from floweisshardt/config_in_result_print
  Config in result print
* test config in result print
* Merge pull request `#80 <https://github.com/floweisshardt/atf/issues/80>`_ from floweisshardt/travis_sinlge_test
  Travis sinlge test
* user result working
* remove unused message
* catkin linting
* Merge pull request `#70 <https://github.com/floweisshardt/atf/issues/70>`_ from floweisshardt/result_msgs
  Result msgs
* add result messages
* update email
* Merge pull request `#60 <https://github.com/floweisshardt/atf/issues/60>`_ from floweisshardt/metrics_path_length
  atf without rosbag play
* atf_test is close to working with all metrics
* recording works for individual test calling rostest recording\_*.test
* recording testblock status to bagfile working
* application and ATF sm decupled using ROS topics
* fix typo
* adapt licenses
* Merge branch 'master' into merge
* Merge pull request `#22 <https://github.com/floweisshardt/atf/issues/22>`_ from ipa-fmw/feature/atf2.0
  Feature/atf2.0
* atf2.0 working with metrics again
* rework TestblockTrigger message
* pull in some changes from atf2.0
* catkin lint'ing
* Merge branch 'feature/analysing' into merge
* allow speedup of analysis with factor 10
* Merge pull request `#5 <https://github.com/floweisshardt/atf/issues/5>`_ from ipa-fmw/master
  master --> stable
* Merge pull request `#8 <https://github.com/floweisshardt/atf/issues/8>`_ from ipa-fmw/master
  master --> recover
* interface metric with type check
* interface working (without types)
* Merge branch 'master' of /home/fmw/git/atf/src/atf
  Conflicts:
  .gitignore
  README.md
* added live status update for testblocks in analysing phase
* Merge pull request `#9 <https://github.com/floweisshardt/atf/issues/9>`_ from koellsch/liveupdate_feature
  added status update feature
* fixed size of integers
* fixed size of integers
* changed type
* some changes for test status update
* added new message for Test status
* code beautification
* added new msg type for obstacle distance
* new format 2 in package.xml
* removed Time.msg
* deleted Time.msg
* added error trigger to msg
* updated msg files
* added new msgs
* Merge pull request `#1 <https://github.com/floweisshardt/atf/issues/1>`_ from ipa-fmw/master
  update from fmw
* first shot for measuring cartesian path length
* Contributors: Felix Messmer, Florian Köhler, Florian Weisshardt, floweisshardt, fmessmer
