^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package atf_test
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2024-02-21)
------------------
* Merge pull request `#191 <https://github.com/floweisshardt/atf/issues/191>`_ from floweisshardt/feature/disable_non_supported_OS
  disable kinetic and melodic
* disable kinetic and melodic
* Merge pull request `#187 <https://github.com/floweisshardt/atf/issues/187>`_ from floweisshardt/fix/atf_test_groundtruth
  Fix atf_test groundtruth
* harmonize testblocksets
* raise time limit
* extend testblock small to 10sec
* extend testblock large to 10sec
* Merge pull request `#185 <https://github.com/floweisshardt/atf/issues/185>`_ from floweisshardt/feature/wait_timeout
  introduce wait_timeout for topics, services, actions, tfs and diagnostics
* wait_timeout kinetic compatible
* introduce wait_timeout for topics, services, actions, tfs and diagnostics
* Merge pull request `#164 <https://github.com/floweisshardt/atf/issues/164>`_ from floweisshardt/feature/metric_topic_data
  new metric topic_data
* raise time limit for analysing
* add test config for metric topic_data in atf_test
* fix circ tf helper
* Merge pull request `#173 <https://github.com/floweisshardt/atf/issues/173>`_ from floweisshardt/feature/manual_test_execution_noetic
  fix manual test execution for noetic and add a rostest for it
* test depend rostest
* install test directory
* disable manual execution test
* dry run
* dry run analyser
* Merge branch 'feature/manual_test_execution_noetic' of github.com:floweisshardt/atf into feature/manual_test_execution_noetic
* move manual test to atf_test
* Merge pull request `#170 <https://github.com/floweisshardt/atf/issues/170>`_ from floweisshardt/feature/actions
  noetic with updated action config
* add missing dependency
* fix python shebang
* roscompile noetic_migration
* Merge pull request `#153 <https://github.com/floweisshardt/atf/issues/153>`_ from fmessmer/feature/pylint_some_more
  [WIP] Feature/pylint-cleanup
* reenable ts1
* fix tests
* proper python modules, cleanup, pylint fixes
* Merge pull request `#150 <https://github.com/floweisshardt/atf/issues/150>`_ from floweisshardt/fix/user_result
  rework groundtruth handling
* use enum for result
* raise groundtruth limit
* raise groundtruth limit
* rework groundtruth handling in metrics
* Merge pull request `#14 <https://github.com/floweisshardt/atf/issues/14>`_ from floweisshardt/master
  sync from floweisshardt/master
* Merge pull request `#149 <https://github.com/floweisshardt/atf/issues/149>`_ from floweisshardt/feature/tf_velocity
  new tf metrics
* adjust metrics
* Merge pull request `#140 <https://github.com/floweisshardt/atf/issues/140>`_ from floweisshardt/feature/atf_plotter
  Feature/atf plotter
* make groundtruth a standalone message and separate submetrics with unique names
* Merge pull request `#127 <https://github.com/floweisshardt/atf/issues/127>`_ from floweisshardt/feature/multiple_cmake_macros
  multiple cmake macros
* set default for EXECUTE_TESTS to True
* cmake macro working with multiple macros
* multiple cmake macros - not working yet
* cmake macro working with multiple macros
* multiple cmake macros - not working yet
* Merge pull request `#13 <https://github.com/floweisshardt/atf/issues/13>`_ from floweisshardt/master
  sync from floweisshardt/master
* Merge pull request `#132 <https://github.com/floweisshardt/atf/issues/132>`_ from floweisshardt/feature/wait_for_tfs
  wait for tfs
* wait for tfs
* Merge pull request `#9 <https://github.com/floweisshardt/atf/issues/9>`_ from floweisshardt/master
  sync from floweisshardt/master
* Merge pull request `#126 <https://github.com/floweisshardt/atf/issues/126>`_ from floweisshardt/fix/print_and_data_series
  Fix analyser output and data series
* add data series to atf_test
* Merge pull request `#118 <https://github.com/floweisshardt/atf/issues/118>`_ from floweisshardt/feature/repetitions_optional
  make repetitions optional
* make repetitions optional
* Merge branch 'master' of github.com:floweisshardt/atf
* Merge pull request `#114 <https://github.com/floweisshardt/atf/issues/114>`_ from floweisshardt/doc
  update docu
* update docu
* Merge pull request `#113 <https://github.com/floweisshardt/atf/issues/113>`_ from fmessmer/ci_updates
  [travis] ci updates
* fix test dependendies
* catkin_lint fixes
* Merge pull request `#112 <https://github.com/floweisshardt/atf/issues/112>`_ from floweisshardt/fix/groundtruth
  Fix groundtruth handling
* fix tf message handling
* fix groundtruth argument parsing
* Merge pull request `#110 <https://github.com/floweisshardt/atf/issues/110>`_ from floweisshardt/feature/data_series
  use data series
* fix user_result handling
* Merge pull request `#7 <https://github.com/floweisshardt/atf/issues/7>`_ from floweisshardt/master
  sync from floweisshardt/master
* Merge pull request `#109 <https://github.com/floweisshardt/atf/issues/109>`_ from floweisshardt/fix/testblock_definition
  define testblocks as list (not dict)
* define testblocks as list (not dict)
* Merge pull request `#6 <https://github.com/floweisshardt/atf/issues/6>`_ from floweisshardt/master
  sync from floweisshardt/master
* Merge pull request `#101 <https://github.com/floweisshardt/atf/issues/101>`_ from floweisshardt/feature/tf_displacement
  new metrics tf_length_translation/rotation and tf_distance_translation/rotation
* fix tests
* add missing dependency
* rename tf metrics
* Merge pull request `#89 <https://github.com/floweisshardt/atf/issues/89>`_ from floweisshardt/benchmarking
  prepare for benchmarking
* fix dropbox uploader
* fix for test generation without rospack find
* small fixes
* prepare for benchmarking with new testblocksets
* Merge pull request `#83 <https://github.com/floweisshardt/atf/issues/83>`_ from floweisshardt/fix_path_length_for_tf_static
  fix path length for tf_static
* fix atf_test
* fix dependency to tf2_ros
* fix groundtruth for path length
* Merge pull request `#82 <https://github.com/floweisshardt/atf/issues/82>`_ from floweisshardt/full_result_print
  Full result print
* full result print with run_tests
* Merge pull request `#80 <https://github.com/floweisshardt/atf/issues/80>`_ from floweisshardt/travis_sinlge_test
  Travis sinlge test
* fix application of atf_test
* user result working
* remove unused speed factor
* extend time limit for recording
* add user_result metric to atf_test
* extend atf test
* Merge pull request `#70 <https://github.com/floweisshardt/atf/issues/70>`_ from floweisshardt/result_msgs
  Result msgs
* add txt export config
* update email
* Merge pull request `#66 <https://github.com/floweisshardt/atf/issues/66>`_ from floweisshardt/fix/deterministic_recording
  Fix/deterministic recording
* use shutdown in atf_test example package
* Merge pull request `#62 <https://github.com/floweisshardt/atf/issues/62>`_ from floweisshardt/fix/57
  fix waiting in application code for state_machine beeing initialized
* fix waiting in application code for state_machine beeing initialized. fixes `#57 <https://github.com/floweisshardt/atf/issues/57>`_
* Merge pull request `#60 <https://github.com/floweisshardt/atf/issues/60>`_ from floweisshardt/metrics_path_length
  atf without rosbag play
* adapt test config
* path length working
* atf_test is close to working with all metrics
* path length analyser working for ts0_c0_r0_e0_0
* fix typo
* adapt licenses
* increase groundtruth epsilon for path length
* Merge pull request `#30 <https://github.com/floweisshardt/atf/issues/30>`_ from ipa-fmw/feature/generator
  feature/generator
* config update
* Merge pull request `#29 <https://github.com/floweisshardt/atf/issues/29>`_ from ipa-fmw/feature/generator
  feature/generator
* genration working again
* close to working
* Merge branch 'master' into feature/roslaunch_check
* Merge pull request `#27 <https://github.com/floweisshardt/atf/issues/27>`_ from ipa-fmw/feature/presenter
  feature/presenter
* increase path length epsilon
* Merge branch 'master' into feature/presenter
* Merge pull request `#26 <https://github.com/floweisshardt/atf/issues/26>`_ from ipa-fmw/feature/linting
  Feature/linting
* catkin lint'ing
* Merge branch 'master' into merge
* Merge branch 'master' into feature/cmake
* Merge pull request `#24 <https://github.com/floweisshardt/atf/issues/24>`_ from ipa-fmw/feature/analysing
  feature/analysing
* add speed factor for analysis
* Merge pull request `#22 <https://github.com/floweisshardt/atf/issues/22>`_ from ipa-fmw/feature/atf2.0
  Feature/atf2.0
* increase groundtruth eplison for publish rate
* analysing (basic handling) working without metrics
* adapt test_app to new config
* recording working for atf2.0
* catkin lint'ing
* Merge branch 'master' into merge
  Conflicts:
  atf_test/package.xml
* Merge branch 'master' into feature/cmake
* merge
* Merge branch 'feature/analysing' into merge
* allow speedup of analysis with factor 10
* Merge pull request `#6 <https://github.com/floweisshardt/atf/issues/6>`_ from ipa-fmw/fix/pylint
  pylint --> stable
* Merge pull request `#4 <https://github.com/floweisshardt/atf/issues/4>`_ from ipa-fmw/fix/pylint
  pylint --> master
* Merge pull request `#9 <https://github.com/floweisshardt/atf/issues/9>`_ from ipa-fmw/master
  master --> stable
* Merge branch 'feature/cmake' into fix/pylint
* Merge pull request `#10 <https://github.com/floweisshardt/atf/issues/10>`_ from ipa-fmw/feature/cmake
  cmake --> master
* using macro from atf_core
* cmake dependencies working
* cmake dependencies working with manual entries
* fix cmake targets
* Merge branch 'master' into fix/pylint
* make targets unique
* Merge branch 'stable' into fix/pylint
* Merge pull request `#5 <https://github.com/floweisshardt/atf/issues/5>`_ from ipa-fmw/master
  master --> stable
* Merge branch 'master' into fix/pylint
  Conflicts:
  atf_core/src/atf_core/atf.py
* Merge pull request `#8 <https://github.com/floweisshardt/atf/issues/8>`_ from ipa-fmw/master
  master --> recover
* Merge pull request `#7 <https://github.com/floweisshardt/atf/issues/7>`_ from ipa-fmw/recover
  Recover
* add dependencies to CMakeLists for recording, analysing, merging and uploading
* allow more time for recording and analysing
* pylint
* increase groundtruth epsilon
* increase time limit
* increase allowed path error
* add interface config to atf_test
* explicit names for tests
* fix path length calculation
* Merge pull request `#2 <https://github.com/floweisshardt/atf/issues/2>`_ from ipa-fmw/feature/list_metrics
  publish rate working
* introduce merge step
* publish rate working
* adapt test config for groundtruth evaluation
* minor config cleanup
* cleaup test config
* cleaup test config
* fix groundtruth evaluation
* remove dropbox config
* do not use rosrun and rospack find during build time
* new config format
* use custom dropbox uploader config
* generate tests in source dir
* Merge branch 'master' of /home/fmw/git/atf/src/atf
  Conflicts:
  .gitignore
  README.md
* Merge pull request `#1 <https://github.com/floweisshardt/atf/issues/1>`_ from ipa-fmw/feature/selftest_debug
  Feature/selftest debug
* extend test suite
* more travis debug output
* small test suite for testing
* extend test suite
* add atf_test package
* removed wrong packages
* moved atf\_* packages into atf repository
* small fix
* added metric "minimal distance to obstacles"
* new format 2 in package.xml
* deleted launch file
* new recorder layout (wip)
* deleted test code
* added time limit
* changes for testing
* added rostest as class
* added rostest class for testing
* added more test parameter
* new robot_config layout
* small change
* small changes
* added working automated example for recording and analysing
* more test preparations
* added package with minimal test
* Contributors: Felix Messmer, Florian Köhler, Florian Weisshardt, floweisshardt, fmessmer
