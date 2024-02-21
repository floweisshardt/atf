^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package atf_metrics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2024-02-21)
------------------
* Merge pull request `#164 <https://github.com/floweisshardt/atf/issues/164>`_ from floweisshardt/feature/metric_topic_data
  new metric topic_data
* fix pylint
* add units
* add missing method
* fix pylint
* fix for pylint update
* new metric topic_data
* Merge pull request `#181 <https://github.com/floweisshardt/atf/issues/181>`_ from floweisshardt/feature/add_units
  add units
* add units
* Merge pull request `#177 <https://github.com/floweisshardt/atf/issues/177>`_ from LoyVanBeek/feature/improve_tf_error_messaging
  Report about no transforms found when a TF metric fails for that reason.
* Report about no transforms found when a TF metric fails for that reason.
  The generic "Stopped without result" pointed me to a different, wrong direction.
  Issue was that I had a wrong transform with non-existant frame but I interpreted that as the test did not succeed or did not get a result set
* Merge pull request `#173 <https://github.com/floweisshardt/atf/issues/173>`_ from floweisshardt/feature/manual_test_execution_noetic
  fix manual test execution for noetic and add a rostest for it
* fix time details format
* Merge pull request `#170 <https://github.com/floweisshardt/atf/issues/170>`_ from floweisshardt/feature/actions
  noetic with updated action config
* roscompile noetic_migration
* Merge pull request `#168 <https://github.com/floweisshardt/atf/issues/168>`_ from fmessmer/python3-compatibility
  [WIP] python3 compatibility
* 2to3 for python3 compatibility
* Merge pull request `#153 <https://github.com/floweisshardt/atf/issues/153>`_ from fmessmer/feature/pylint_some_more
  [WIP] Feature/pylint-cleanup
* resolve cyclic dependency
* fixup: import atf_core.error
* revert special atf_metrics module
* properly register metrics
* proper python modules, cleanup, pylint fixes
* Merge pull request `#163 <https://github.com/floweisshardt/atf/issues/163>`_ from floweisshardt/feature/metric_interface_regex
  support regex in interface metric
* support regex in interface metric
* Merge pull request `#150 <https://github.com/floweisshardt/atf/issues/150>`_ from floweisshardt/fix/user_result
  rework groundtruth handling
* fix some more boolean operations of groundtruth.result
* use enum for result
* fix user result metric
* rework groundtruth handling in metrics
* check details of user_result
* Merge pull request `#155 <https://github.com/floweisshardt/atf/issues/155>`_ from floweisshardt/feature/remove_unsupported_merics
  remove unsupported metrics
* remove unsupported metrics
* Merge pull request `#14 <https://github.com/floweisshardt/atf/issues/14>`_ from floweisshardt/master
  sync from floweisshardt/master
* Merge pull request `#149 <https://github.com/floweisshardt/atf/issues/149>`_ from floweisshardt/feature/tf_velocity
  new tf metrics
* introduce SPAN_MEAN, SPAN_MAX, SPAN_ABSMAX, SPAN_MIN, SPAN_ABSMIN
* fix no result handling
* harmonize metrics
* add acceleration and jerk metric
* add new tf_velicity metric
* fix error messages for user_result metric
* Merge pull request `#145 <https://github.com/floweisshardt/atf/issues/145>`_ from floweisshardt/feature/atf_plotter
  more features for atf_plotter
* fix user_result handling
* fix groundtruth handling in user result
* Merge pull request `#140 <https://github.com/floweisshardt/atf/issues/140>`_ from floweisshardt/feature/atf_plotter
  Feature/atf plotter
* Merge pull request `#143 <https://github.com/floweisshardt/atf/issues/143>`_ from floweisshardt/feature/merger
  merge results
* fix details in metric time
* fix min and max values for metric publish_rate
* fix mode in metric user_result
* handover mode to metric_result
* adjust metrics with standalone groundtruth message and unique submetrics
* introduce mode (span/snap) to all metrics
* Merge pull request `#135 <https://github.com/floweisshardt/atf/issues/135>`_ from floweisshardt/feature/check_metric_configuration
  validate metric configuration
* validate metric configuration
* Merge pull request `#9 <https://github.com/floweisshardt/atf/issues/9>`_ from floweisshardt/master
  sync from floweisshardt/master
* Merge pull request `#113 <https://github.com/floweisshardt/atf/issues/113>`_ from fmessmer/ci_updates
  [travis] ci updates
* more cleanup
* catkin_lint fixes
* Merge pull request `#112 <https://github.com/floweisshardt/atf/issues/112>`_ from floweisshardt/fix/groundtruth
  Fix groundtruth handling
* fix tf message handling
* fix groundtruth argument parsing
* beautifying metrics
* Merge pull request `#110 <https://github.com/floweisshardt/atf/issues/110>`_ from floweisshardt/feature/data_series
  use data series
* series_mode for all metrics
* add min/max/mean/std
* use data series
* Merge pull request `#7 <https://github.com/floweisshardt/atf/issues/7>`_ from floweisshardt/master
  sync from floweisshardt/master
* Merge pull request `#107 <https://github.com/floweisshardt/atf/issues/107>`_ from floweisshardt/fix/tf_distance_metrics
  fix calculation of tf_distance metrics
* fix calculation of tf_distance metrics
* Merge pull request `#6 <https://github.com/floweisshardt/atf/issues/6>`_ from floweisshardt/master
  sync from floweisshardt/master
* Merge pull request `#101 <https://github.com/floweisshardt/atf/issues/101>`_ from floweisshardt/feature/tf_displacement
  new metrics tf_length_translation/rotation and tf_distance_translation/rotation
* remove outdated files
* rename tf metrics
* new metrics angle and linear displacement and integrated rotation
* Merge pull request `#87 <https://github.com/floweisshardt/atf/issues/87>`_ from floweisshardt/path_length
  catch path_length exceptions
* catch path_length exceptions
* Merge pull request `#84 <https://github.com/floweisshardt/atf/issues/84>`_ from floweisshardt/recorder_updates
  only call recorder plugins if metric is specified
* fix calculate path length
* Merge pull request `#83 <https://github.com/floweisshardt/atf/issues/83>`_ from floweisshardt/fix_path_length_for_tf_static
  fix path length for tf_static
* handover topics via get_topics
* fix path length for tf_static
* Merge pull request `#80 <https://github.com/floweisshardt/atf/issues/80>`_ from floweisshardt/travis_sinlge_test
  Travis sinlge test
* user result working
* fix user result metric
* catkin linting
* try creating subscribers multiple times
* fix user result metric calculation
* add user_result metric
* Merge pull request `#76 <https://github.com/floweisshardt/atf/issues/76>`_ from floweisshardt/fix/analyser
  Fix/analyser
* handle no result in metrics
* Merge pull request `#70 <https://github.com/floweisshardt/atf/issues/70>`_ from floweisshardt/result_msgs
  Result msgs
* metrics using result messages
* update email
* Merge pull request `#65 <https://github.com/floweisshardt/atf/issues/65>`_ from floweisshardt/fix/less_output
  less output for metrics
* Merge pull request `#64 <https://github.com/floweisshardt/atf/issues/64>`_ from floweisshardt/fix/tf_exception
  catch tf exception
* less output for metrics
* catch tf exception
* Merge pull request `#61 <https://github.com/floweisshardt/atf/issues/61>`_ from floweisshardt/parallel-tests
  allow parallel tests
* fix calculate time
* fix metrics
* Merge pull request `#60 <https://github.com/floweisshardt/atf/issues/60>`_ from floweisshardt/metrics_path_length
  atf without rosbag play
* recording, analysing and merging is working with atf_test
* path length working
* atf_test is close to working with all metrics
* calculate path length working without threads
* path length analyser working for ts0_c0_r0_e0_0
* recording and analysing working for publish rate
* publish_rate not yet working
* recording works for individual test calling rostest recording\_*.test
* recording working with generated rostest file
* Merge pull request `#44 <https://github.com/floweisshardt/atf/issues/44>`_ from fmw-hb/fix/merger
  Fix/merger
* include path length
* fixed merging bug
* add path length and distance metrics
* long tests
* big slam testing
* hannes test
* fix typo
* adapt licenses
* Merge branch 'master' into feature/presenter
* Merge pull request `#26 <https://github.com/floweisshardt/atf/issues/26>`_ from ipa-fmw/feature/linting
  Feature/linting
* pylint'ing
* Merge branch 'master' into merge
* Merge branch 'master' into feature/cmake
* Merge pull request `#24 <https://github.com/floweisshardt/atf/issues/24>`_ from ipa-fmw/feature/analysing
  feature/analysing
* add speed factor for analysis
* Merge pull request `#22 <https://github.com/floweisshardt/atf/issues/22>`_ from ipa-fmw/feature/atf2.0
  Feature/atf2.0
* atf2.0 working with metrics again
* catkin lint'ing
* Merge branch 'feature/analysing' into merge
* allow speedup of analysis with factor 10
* Merge pull request `#6 <https://github.com/floweisshardt/atf/issues/6>`_ from ipa-fmw/fix/pylint
  pylint --> stable
* Merge pull request `#4 <https://github.com/floweisshardt/atf/issues/4>`_ from ipa-fmw/fix/pylint
  pylint --> master
* Merge pull request `#5 <https://github.com/floweisshardt/atf/issues/5>`_ from ipa-fmw/master
  master --> stable
* Merge pull request `#8 <https://github.com/floweisshardt/atf/issues/8>`_ from ipa-fmw/master
  master --> recover
* pylint
* catch case if no interfaces are specified
* use groundtruth and result data for interface metric
* interface metric with type check
* add interface config to atf_test
* fix reporting for interface metric
* fix interface to metrics handler
* cleanup metric files
* interface working (without types)
* fix path length calculation
* Merge pull request `#3 <https://github.com/floweisshardt/atf/issues/3>`_ from ipa-fmw/feature/list_metrics
  Feature/list metrics
* get rid of obsolete else path
* Merge pull request `#2 <https://github.com/floweisshardt/atf/issues/2>`_ from ipa-fmw/feature/list_metrics
  publish rate working
* fix groundtruth result merging
* fix time analysis
* publish rate working
* unify analysis result and integrate details to presenter
* groundtruth for path_length
* reduce logs and handover groundtruth data
* fix groundtruth evaluation
* fix path length metrix for groundtruth
* add groundtruth check for time
* fix install tags
* Merge branch 'master' of /home/fmw/git/atf/src/atf
  Conflicts:
  .gitignore
  README.md
* Merge pull request `#1 <https://github.com/floweisshardt/atf/issues/1>`_ from ipa-fmw/feature/selftest_debug
  Feature/selftest debug
* fix dependency
* deactivate obstacle distance
* added new metric to calculate the publish rate of a topic
* moved obstacle_distance node to external package
* added example for own metric
* added todo
* added functions for purge
* added functions for pause and purge
* Merge pull request `#9 <https://github.com/floweisshardt/atf/issues/9>`_ from koellsch/liveupdate_feature
  added status update feature
* code beautification
* added feature to measure distance to all objects and from link chain to objects
* modified documentation
* added metric "minimal distance to obstacles"
* changed topic name
* new format 2 in package.xml
* deleted "data" from output
* changed output of results
* removed logging of timestamp
* updated documentation
* added documentation
* fixed bug in path calculation
* enabled error output
* added check if resource data is available
* fixed return value for calculate_path_length metric
* added handler for receiving the needed metrics
* added metrics.yaml temporary, will be deleted later
* deleted metrics.yaml
* added todo tags
* test_builder builds metrics with infos from metrics.yaml
* added documentation
* small beautifications
* added error return when not stopped
* removed deprecated print output
* removed unused import
* added 'min' and 'max' to output && changed output syntax
* changed output syntax
* added rounding of values
* added activation time output
* changed metric return data to list
* added export results to yaml file
* added dummy for distance to obstacles
* added metric for calculating resources
* added pause function
* small changes
* calculate path length and time are working
* some cleaning && preparations for time measurement
* changed structure
* first test
* switched to new metric package
* fixed typo
* added metric for path length
* added seperate package for metrics
* Contributors: Björn Eistel, Felix Messmer, Florian Köhler, Florian Weisshardt, Loy van Beek, floweisshardt, fmessmer, fmw-hb
