^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package atf_recorder_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2024-02-21)
------------------
* Merge pull request `#164 <https://github.com/floweisshardt/atf/issues/164>`_ from floweisshardt/feature/metric_topic_data
  new metric topic_data
* ignore pylint import error
* Merge pull request `#170 <https://github.com/floweisshardt/atf/issues/170>`_ from floweisshardt/feature/actions
  noetic with updated action config
* roscompile noetic_migration
* Merge pull request `#168 <https://github.com/floweisshardt/atf/issues/168>`_ from fmessmer/python3-compatibility
  [WIP] python3 compatibility
* fix import http.client
* 2to3 for python3 compatibility
* Merge pull request `#153 <https://github.com/floweisshardt/atf/issues/153>`_ from fmessmer/feature/pylint_some_more
  [WIP] Feature/pylint-cleanup
* resolve cyclic dependency
* revert special atf_recorder_plugins module
* proper python modules, cleanup, pylint fixes
* fix some pylint issues
* Merge pull request `#157 <https://github.com/floweisshardt/atf/issues/157>`_ from floweisshardt/fix/remove_obstacle_distance_recorder_plugin
  remove obstacle distance recorder plugin
* remove obstacle distance recorder plugin
* Merge pull request `#155 <https://github.com/floweisshardt/atf/issues/155>`_ from floweisshardt/feature/remove_unsupported_merics
  remove unsupported metrics
* remove unsupported metrics
* Merge pull request `#9 <https://github.com/floweisshardt/atf/issues/9>`_ from floweisshardt/master
  sync from floweisshardt/master
* Merge pull request `#113 <https://github.com/floweisshardt/atf/issues/113>`_ from fmessmer/ci_updates
  [travis] ci updates
* catkin_lint fixes
* Merge pull request `#84 <https://github.com/floweisshardt/atf/issues/84>`_ from floweisshardt/recorder_updates
  only call recorder plugins if metric is specified
* fix rosservice exception
* only call recorder plugins if metric is specified
* Merge pull request `#70 <https://github.com/floweisshardt/atf/issues/70>`_ from floweisshardt/result_msgs
  Result msgs
* update email
* Merge pull request `#60 <https://github.com/floweisshardt/atf/issues/60>`_ from floweisshardt/metrics_path_length
  atf without rosbag play
* atf_test is close to working with all metrics
* Merge pull request `#46 <https://github.com/floweisshardt/atf/issues/46>`_ from ipa-fmw/fix/handling_shutdown_delay
  Fix/handling shutdown delay
* fixed roslaunch check error caused due to missing package.xml file
* this intermediary PR consists of following:
  - triggering the record only before and after specific event
  - disabled the record for all topics
* fix typo
* adapt licenses
* Merge branch 'master' into feature/presenter
* Merge pull request `#26 <https://github.com/floweisshardt/atf/issues/26>`_ from ipa-fmw/feature/linting
  Feature/linting
* pylint'ing
* Merge branch 'master' into merge
* Merge pull request `#22 <https://github.com/floweisshardt/atf/issues/22>`_ from ipa-fmw/feature/atf2.0
  Feature/atf2.0
* recording working for atf2.0
* cleaup code
* catkin lint'ing
* Merge branch 'feature/analysing' into merge
* allow speedup of analysis with factor 10
* Merge pull request `#6 <https://github.com/floweisshardt/atf/issues/6>`_ from ipa-fmw/fix/pylint
  pylint --> stable
* Merge pull request `#11 <https://github.com/floweisshardt/atf/issues/11>`_ from ipa-fmw/fix/timestamps
  Fix/timestamps
* Merge pull request `#4 <https://github.com/floweisshardt/atf/issues/4>`_ from ipa-fmw/fix/pylint
  pylint --> master
* fix timestamps
* Merge pull request `#5 <https://github.com/floweisshardt/atf/issues/5>`_ from ipa-fmw/master
  master --> stable
* Merge pull request `#8 <https://github.com/floweisshardt/atf/issues/8>`_ from ipa-fmw/master
  master --> recover
* pylint
* interface metric with type check
* interface working (without types)
* fix install tags
* Merge branch 'master' of /home/fmw/git/atf/src/atf
  Conflicts:
  .gitignore
  README.md
* Merge pull request `#1 <https://github.com/floweisshardt/atf/issues/1>`_ from ipa-fmw/feature/selftest_debug
  Feature/selftest debug
* deactivate resources plugin
* deactivate obstacle distance
* deactivate obstacle distance
* added obstacle_distance as recorder plugin (wip)
* added robot_config content to plugins
* moved obstacle_distance node to external package
* added example for own plugin
* renamed plugin config file
* added fix for missing msgs
* fixed bug in output
* added output messages
* fixed error management
* changed method for distance measurement
* Merge pull request `#9 <https://github.com/floweisshardt/atf/issues/9>`_ from koellsch/liveupdate_feature
  added status update feature
* code beautification
* added check for distance topic
* code beautification
* added check for joints and robot_description
* added check for tests without a robot
* added variable for maximum minimal distance
* adaptations to ROS C++ Style Guide
* moved atf\_* packages into atf repository
* modified error message
* bug fix
* moved obstacle_distance node to pkg atf_recorder_plugins
* changed topic name
* new format 2 in package.xml
* performance adjustments
* small changes and documentation
* added package for recorder plugins
* Contributors: Florian Köhler, Florian Weisshardt, floweisshardt, fmessmer, fmw-ss
