^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package atf_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2024-02-21)
------------------
* Merge pull request `#191 <https://github.com/floweisshardt/atf/issues/191>`_ from floweisshardt/feature/disable_non_supported_OS
  disable kinetic and melodic
* disable kinetic and melodic
* Merge pull request `#185 <https://github.com/floweisshardt/atf/issues/185>`_ from floweisshardt/feature/wait_timeout
  introduce wait_timeout for topics, services, actions, tfs and diagnostics
* only print diagnostic failures
* fixes from fxm
* prevent TF_REPEATED_DATA
* supress error message about not added subscibers
* fix pylint
* wait_timeout kinetic compatible
* introduce wait_timeout for topics, services, actions, tfs and diagnostics
* Merge pull request `#164 <https://github.com/floweisshardt/atf/issues/164>`_ from floweisshardt/feature/metric_topic_data
  new metric topic_data
* reformat tbm plots with units
* Merge pull request `#182 <https://github.com/floweisshardt/atf/issues/182>`_ from LoyVanBeek/feature/clearer_errors
  Log which config file is used when no files are found
* Log which config file is used when no files are found
* increase tf_static frequency
* Merge pull request `#181 <https://github.com/floweisshardt/atf/issues/181>`_ from floweisshardt/feature/add_units
  add units
* add units
* Merge pull request `#176 <https://github.com/floweisshardt/atf/issues/176>`_ from floweisshardt/feature/install_space
  support atf in install space
* apply code comments from @fmessmer
  Co-authored-by: Felix Messmer <felixmessmer@gmail.com>
* support atf in install space
* Merge pull request `#173 <https://github.com/floweisshardt/atf/issues/173>`_ from floweisshardt/feature/manual_test_execution_noetic
  fix manual test execution for noetic and add a rostest for it
* fix time calculation
* remove duplicate strip
* cleanup and update docu
* dry run
* dry run analyser
* Merge branch 'feature/manual_test_execution_noetic' of github.com:floweisshardt/atf into feature/manual_test_execution_noetic
* move manual test to atf_test
* add dependency to catkin-tools
* add catkin dependency
* add build dependency to rostest
* fix manual test execution for noetic and add a rostest for it
* Merge pull request `#172 <https://github.com/floweisshardt/atf/issues/172>`_ from fmessmer/test_noetic
  test noetic
* use find_package(PythonInterp)
* use env to call correct python executable
* Merge pull request `#170 <https://github.com/floweisshardt/atf/issues/170>`_ from floweisshardt/feature/actions
  noetic with updated action config
* fix yaml.load deprecation
* fix python shebang
* roscompile noetic_migration
* Merge pull request `#168 <https://github.com/floweisshardt/atf/issues/168>`_ from fmessmer/python3-compatibility
  [WIP] python3 compatibility
* pylint fixes
* 2to3 for python3 compatibility
* Merge pull request `#153 <https://github.com/floweisshardt/atf/issues/153>`_ from fmessmer/feature/pylint_some_more
  [WIP] Feature/pylint-cleanup
* fix pylint
* fix import
* resolve cyclic dependency
* proper python modules, cleanup, pylint fixes
* fix some pylint issues
* Merge pull request `#150 <https://github.com/floweisshardt/atf/issues/150>`_ from floweisshardt/fix/user_result
  rework groundtruth handling
* fix some more boolean operations of groundtruth.result
* use enum for result
* use info instead of warn for not setting user result
* rework groundtruth handling in metrics
* Merge pull request `#156 <https://github.com/floweisshardt/atf/issues/156>`_ from HannesBachter/fix/tf_handling
  fix tf handling
* fix tf handling
* Merge pull request `#14 <https://github.com/floweisshardt/atf/issues/14>`_ from floweisshardt/master
  sync from floweisshardt/master
* Merge pull request `#149 <https://github.com/floweisshardt/atf/issues/149>`_ from floweisshardt/feature/tf_velocity
  new tf metrics
* aggregated results are always SPAN_MEAN
* Merge pull request `#145 <https://github.com/floweisshardt/atf/issues/145>`_ from floweisshardt/feature/atf_plotter
  more features for atf_plotter
* use long test_description for style mbt
* fix docu
* fix order or test parameters
* fix result aggregation
* fix aggregation of details
* allow multiple filter
* add package name to atf_result_aggregated
* fix string replacing for test_generation_config
* add name of atf package to atf result
* Merge pull request `#146 <https://github.com/floweisshardt/atf/issues/146>`_ from floweisshardt/fix/parallel_tests
  fix parallel tests
* fix directory and filename replacement
* Merge pull request `#140 <https://github.com/floweisshardt/atf/issues/140>`_ from floweisshardt/feature/atf_plotter
  Feature/atf plotter
* Merge pull request `#144 <https://github.com/floweisshardt/atf/issues/144>`_ from floweisshardt/feature/remove_deprecated_packages
  Feature/remove deprecated packages
* remove atf_status_server
* remove old merge script and rename to aggregate
* fix result aggregation for multiple testsuites
* Merge pull request `#143 <https://github.com/floweisshardt/atf/issues/143>`_ from floweisshardt/feature/merger
  merge results
* fix groundtruth aggregation
* fix imports
* move get_sorted_plot_dicts to configuration parser
* merge results
* check if metric is implemented
* make groundtruth a standalone message and separate submetrics with unique names
* export result to bag file
* Merge pull request `#138 <https://github.com/floweisshardt/atf/issues/138>`_ from floweisshardt/fix/check_merics
  check if metrics are implemented
* check if metrics are implemented
* Merge pull request `#127 <https://github.com/floweisshardt/atf/issues/127>`_ from floweisshardt/feature/multiple_cmake_macros
  multiple cmake macros
* fix target name
* fix target name
* add targets again
* fix syntax help
* fix execute_as_test
* handle test_generation_config_file in record_tests
* fix execute_as_test launch arg
* remove rospy
* handle arguments for tets correctly
* add missing dependency
* set default for EXECUTE_TESTS to True
* explicit handover of execute_test
* cmake macro working with multiple macros
* multiple cmake macros - not working yet
* Merge pull request `#135 <https://github.com/floweisshardt/atf/issues/135>`_ from floweisshardt/feature/check_metric_configuration
  validate metric configuration
* validate metric configuration
* Merge pull request `#13 <https://github.com/floweisshardt/atf/issues/13>`_ from floweisshardt/master
  sync from floweisshardt/master
* Merge pull request `#133 <https://github.com/floweisshardt/atf/issues/133>`_ from floweisshardt/feature/wait_for_diagnostics
  wait for diagnostics
* wait for diagnostics
* Merge pull request `#132 <https://github.com/floweisshardt/atf/issues/132>`_ from floweisshardt/feature/wait_for_tfs
  wait for tfs
* wait for tfs
* Merge pull request `#131 <https://github.com/floweisshardt/atf/issues/131>`_ from floweisshardt/feature/wait_for_actions
  wait for actions
* Merge pull request `#130 <https://github.com/floweisshardt/atf/issues/130>`_ from floweisshardt/fix/shutdown
  stop all testblocks during shutdown
* Merge pull request `#129 <https://github.com/floweisshardt/atf/issues/129>`_ from floweisshardt/fix/incomplete_testblock
  fix incomplete testblock
* wait for actions
* stop all testblocks during shutdown
* fixes `#128 <https://github.com/floweisshardt/atf/issues/128>`_ incomplete testblock
* Merge pull request `#9 <https://github.com/floweisshardt/atf/issues/9>`_ from floweisshardt/master
  sync from floweisshardt/master
* Merge pull request `#126 <https://github.com/floweisshardt/atf/issues/126>`_ from floweisshardt/fix/print_and_data_series
  Fix analyser output and data series
* fix print output of analyser
* Merge pull request `#124 <https://github.com/floweisshardt/atf/issues/124>`_ from floweisshardt/feature/result_summary
  print result summary
* print result summary
* Merge pull request `#118 <https://github.com/floweisshardt/atf/issues/118>`_ from floweisshardt/feature/repetitions_optional
  make repetitions optional
* make repetitions optional
* Merge pull request `#113 <https://github.com/floweisshardt/atf/issues/113>`_ from fmessmer/ci_updates
  [travis] ci updates
* fix script install destination
* fix test dependendies
* catkin_lint fixes
* Merge pull request `#110 <https://github.com/floweisshardt/atf/issues/110>`_ from floweisshardt/feature/data_series
  use data series
* fix user_result handling
* use data series
* Merge pull request `#7 <https://github.com/floweisshardt/atf/issues/7>`_ from floweisshardt/master
  sync from floweisshardt/master
* Merge pull request `#109 <https://github.com/floweisshardt/atf/issues/109>`_ from floweisshardt/fix/testblock_definition
  define testblocks as list (not dict)
* define testblocks as list (not dict)
* Merge pull request `#102 <https://github.com/floweisshardt/atf/issues/102>`_ from floweisshardt/params_and_args
  add support for list and dict types for parameters and arguments
* add support for list and dict types for parameters and arguments
* Merge pull request `#6 <https://github.com/floweisshardt/atf/issues/6>`_ from floweisshardt/master
  sync from floweisshardt/master
* sort tests
* Merge pull request `#101 <https://github.com/floweisshardt/atf/issues/101>`_ from floweisshardt/feature/tf_displacement
  new metrics tf_length_translation/rotation and tf_distance_translation/rotation
* remove print
* rename tf metrics
* Merge pull request `#5 <https://github.com/floweisshardt/atf/issues/5>`_ from floweisshardt/master
  sync from floweisshardt/atf master
* new metrics angle and linear displacement and integrated rotation
* increase trigger rate
* Merge pull request `#100 <https://github.com/floweisshardt/atf/issues/100>`_ from floweisshardt/manual_test_execution
  Manual test execution
* remove recording directory from generated tests
* use regex to manually trigger a subset of recording tests
* add script for manual test execution
* Merge pull request `#98 <https://github.com/floweisshardt/atf/issues/98>`_ from floweisshardt/manual_test_execution
  Manual test execution
* use regex to manually trigger a subset of recording tests
* Merge pull request `#99 <https://github.com/floweisshardt/atf/issues/99>`_ from HannesBachter/feature/for_travis
  enhancements for atf
* do not use recording folder
* execute roslaunch checks for rostests and cleanup atf_test
* load params and arguments for tests
* add script for manual test execution
* Merge pull request `#97 <https://github.com/floweisshardt/atf/issues/97>`_ from mojin-robotics/master
  sync from mojin-robotics fork
* Merge pull request `#2 <https://github.com/floweisshardt/atf/issues/2>`_ from floweisshardt/master
  fix raise condition for zero time testblocks
* Merge pull request `#3 <https://github.com/floweisshardt/atf/issues/3>`_ from HannesBachter/fix/use_rosparam
  load params from yamls via rosparam instead of param
* load params from yamls via rosparam instead of param
* Merge pull request `#93 <https://github.com/floweisshardt/atf/issues/93>`_ from floweisshardt/fix/travis_io_bag_error
  fix travis io bag error
* handle zero time testblocks
* add more verbose logs to stop and shutdown
* Merge pull request `#92 <https://github.com/floweisshardt/atf/issues/92>`_ from floweisshardt/feature/export_atf_test_result
  export atf test result
* Merge pull request `#90 <https://github.com/floweisshardt/atf/issues/90>`_ from floweisshardt/benchmarking
  pylinting atf_core/analyser.py
* Merge pull request `#91 <https://github.com/floweisshardt/atf/issues/91>`_ from floweisshardt/fix/recorder_raise_condition
  fix raise condition in recorder for zero time testblocks
* export atf test result
* fix raise condition in recorder for zero time testblocks
* pylinting atf_core/analyser.py
* Merge pull request `#89 <https://github.com/floweisshardt/atf/issues/89>`_ from floweisshardt/benchmarking
  prepare for benchmarking
* fix dropbox uploader
* fix for test generation without rospack find
* remove merge job
* small fixes
* prepare for benchmarking with new testblocksets
* Merge pull request `#88 <https://github.com/floweisshardt/atf/issues/88>`_ from floweisshardt/generation
  fix test generation for additioanl parameters
* fix test generation for additioanl parameters
* Merge pull request `#87 <https://github.com/floweisshardt/atf/issues/87>`_ from floweisshardt/path_length
  catch path_length exceptions
* catch path_length exceptions
* clean txt results dir
* Merge pull request `#86 <https://github.com/floweisshardt/atf/issues/86>`_ from floweisshardt/speedup_tests
  fix speed up tests
* adding sleep again to wait for all trigger subscribers to be initialized
* Merge pull request `#81 <https://github.com/floweisshardt/atf/issues/81>`_ from floweisshardt/speedup_tests
  Speedup tests
* remove sleeps
* Merge pull request `#85 <https://github.com/floweisshardt/atf/issues/85>`_ from floweisshardt/config_in_result_print
  Config in result print
* fix print of result details
* test config in result print
* Merge pull request `#84 <https://github.com/floweisshardt/atf/issues/84>`_ from floweisshardt/recorder_updates
  only call recorder plugins if metric is specified
* tf_static callback with 0.1Hz
* only call recorder plugins if metric is specified
* Merge pull request `#83 <https://github.com/floweisshardt/atf/issues/83>`_ from floweisshardt/fix_path_length_for_tf_static
  fix path length for tf_static
* handover topics only vial get_topics
* catch error for bag file not found
* fix path length for tf_static
* Merge pull request `#82 <https://github.com/floweisshardt/atf/issues/82>`_ from floweisshardt/full_result_print
  Full result print
* full result print with run_tests
* verbose result print option
* Merge pull request `#80 <https://github.com/floweisshardt/atf/issues/80>`_ from floweisshardt/travis_sinlge_test
  Travis sinlge test
* user result working
* fix parameter parsing
* minor fixes
* catkin linting
* first tigger recorder handles and then record status
* wait 10sec during shutdown
* try creating subscribers multiple times
* subscribe to topics from beginning and forever
* sleep again before starting application
* remove unused speed factor
* remvoe unused self variable
* add default for details
* add user_result metric
* Merge pull request `#76 <https://github.com/floweisshardt/atf/issues/76>`_ from floweisshardt/fix/analyser
  Fix/analyser
* no more result validation in testblock
* fix counter in analyser
* Merge pull request `#70 <https://github.com/floweisshardt/atf/issues/70>`_ from floweisshardt/result_msgs
  Result msgs
* fix groundtruth_result aggregation for None
* analysis using result msgs
* add support for txt exports
* update email
* Merge pull request `#67 <https://github.com/floweisshardt/atf/issues/67>`_ from floweisshardt/fix/analyser_overall_result
  fix overall result reporting
* fix overall result reporting
* fix overall result reporting
* Merge pull request `#68 <https://github.com/floweisshardt/atf/issues/68>`_ from floweisshardt/fix/deterministic_recording
  add additional sleep, waiting for sm_test to be ready
* add additional sleep, waiting for sm_test to be ready
* Merge pull request `#66 <https://github.com/floweisshardt/atf/issues/66>`_ from floweisshardt/fix/deterministic_recording
  Fix/deterministic recording
* proper shutdown of atf application and recorder
* Merge pull request `#62 <https://github.com/floweisshardt/atf/issues/62>`_ from floweisshardt/fix/57
  fix waiting in application code for state_machine beeing initialized
* add another sleep after waiting for first state machine message
* fix waiting in application code for state_machine beeing initialized. fixes `#57 <https://github.com/floweisshardt/atf/issues/57>`_
* increase time limit for merging
* fix metrics
* Merge branch 'metrics_path_length'
* increase timeout for cleaning job
* Merge pull request `#60 <https://github.com/floweisshardt/atf/issues/60>`_ from floweisshardt/metrics_path_length
  atf without rosbag play
* fix install tags
* recording, analysing and merging is working with atf_test
* skip json output
* fix install tags
* fix progressbar
* add dependencies
* add smach dependency
* add launch file for manual testing (without rostest)
* path length working
* atf_test is close to working with all metrics
* calculate path length working without threads
* path length analyser working for ts0_c0_r0_e0_0
* recording and analysing working for publish rate
* recorder working with recording topics
* publish_rate not yet working
* change import
* full pipeline working locally using catkin run_tests
* merger working
* recording works for individual test calling rostest recording\_*.test
* sm_test is the test (not application anymore
* recording working with generated rostest file
* recording testblock status to bagfile working
* recording for atf_test_app_time working
* transitions based on trigger topics
* read test config from parameter server
* application and ATF sm decupled using ROS topics
* initial smach SM running
* not yet working
* remove some prints
* analyzer working without rosbag play
* working towards no rosbag play
* Merge pull request `#46 <https://github.com/floweisshardt/atf/issues/46>`_ from ipa-fmw/fix/handling_shutdown_delay
  Fix/handling shutdown delay
* cleaup naming or variables
* reduce logoutput by using logdebug
* fixed roslaunch check error caused due to missing package.xml file
* uncommented the callback for the recorder plugin
* fixed an error due to invalid test-generated file path
* Modified the test generation folder to be in build directory
* minor modification for checking valid metrics
* this intermediary PR consists of following:
  - triggering the record only before and after specific event
  - disabled the record for all topics
* Merge pull request `#44 <https://github.com/floweisshardt/atf/issues/44>`_ from fmw-hb/fix/merger
  Fix/merger
* fixed merging bug
* run for the weekend
* cleanup for PR
* add path length and distance metrics
* Merge pull request `#37 <https://github.com/floweisshardt/atf/issues/37>`_ from ipa-fmw/fix/travis
  add dependency to python-lxml
* add dependency to python-lxml
* fix concurrency issue with directory creation
* fix typo
* adapt licenses
* Merge pull request `#30 <https://github.com/floweisshardt/atf/issues/30>`_ from ipa-fmw/feature/generator
  feature/generator
* config update
* Merge pull request `#29 <https://github.com/floweisshardt/atf/issues/29>`_ from ipa-fmw/feature/generator
  feature/generator
* genration working again
* mark nodes as required
* Merge branch 'master' into feature/presenter
* Merge pull request `#26 <https://github.com/floweisshardt/atf/issues/26>`_ from ipa-fmw/feature/linting
  Feature/linting
* Merge pull request `#25 <https://github.com/floweisshardt/atf/issues/25>`_ from ipa-fmw/feature/cmake
  feature/cmake
* pylint'ing
* catkin lint'ing
* Merge branch 'feature/cmake' into merge
* Merge branch 'master' into merge
* Merge branch 'master' into feature/cmake
* add roslaunch check for test generated paths
* Merge pull request `#24 <https://github.com/floweisshardt/atf/issues/24>`_ from ipa-fmw/feature/analysing
  feature/analysing
* Merge branch 'master' into feature/analysing
* Merge pull request `#23 <https://github.com/floweisshardt/atf/issues/23>`_ from ipa-fmw/feature/cmake
  Feature/cmake
* add speed factor for analysis
* fix install tags
* add clean target
* Merge pull request `#22 <https://github.com/floweisshardt/atf/issues/22>`_ from ipa-fmw/feature/atf2.0
  Feature/atf2.0
* fix merger and presenter
* atf2.0 working with metrics again
* analysing (basic handling) working without metrics
* adapt test_app to new config
* recording working for atf2.0
* fix threading problem in testblock SM
* pull in some changes from atf2.0
* catkin lint'ing
* Merge branch 'feature/roslaunch_check' of github.com:ipa-fmw/atf into feature/roslaunch_check
* Merge branch 'master' of github.com:ipa-fmw/atf
* Merge branch 'master' into feature/roslaunch_check
* Merge branch 'feature/analysing' into merge
* Merge branch 'master' into feature/cmake
* allow speedup of analysis with factor 10
* Merge branch 'master' of github.com:ipa-fmw/atf into feature/analysing
* Merge branch 'master' into feature/analysing
* Merge pull request `#18 <https://github.com/floweisshardt/atf/issues/18>`_ from ipa-fmw/feature/merger
  feature/merger
* another fix for cmake dependency handling
* clean build order for cmake targets
* fix merger
* Merge branch 'feature/merger' into merge
* Merge branch 'master' into feature/presenter
* Merge pull request `#15 <https://github.com/floweisshardt/atf/issues/15>`_ from ipa-fmw/feature/merger
  handle errored testblocks in merger
* handle errored testblocks in merger
* beautify console log
* Merge pull request `#6 <https://github.com/floweisshardt/atf/issues/6>`_ from ipa-fmw/fix/pylint
  pylint --> stable
* Merge pull request `#4 <https://github.com/floweisshardt/atf/issues/4>`_ from ipa-fmw/fix/pylint
  pylint --> master
* Merge pull request `#9 <https://github.com/floweisshardt/atf/issues/9>`_ from ipa-fmw/master
  master --> stable
* Merge branch 'feature/cmake' into fix/pylint
* minor cleanup
* Merge pull request `#10 <https://github.com/floweisshardt/atf/issues/10>`_ from ipa-fmw/feature/cmake
  cmake --> master
* using macro from atf_core
* Merge pull request `#5 <https://github.com/floweisshardt/atf/issues/5>`_ from ipa-fmw/master
  master --> stable
* Merge branch 'master' into fix/pylint
  Conflicts:
  atf_core/src/atf_core/atf.py
* Merge pull request `#8 <https://github.com/floweisshardt/atf/issues/8>`_ from ipa-fmw/master
  master --> recover
* merge
* pylint
* interface working (without types)
* avoid abolute path in test_generated
* explicit names for tests
* Merge pull request `#3 <https://github.com/floweisshardt/atf/issues/3>`_ from ipa-fmw/feature/list_metrics
  Feature/list metrics
* use subprocess.call instead of os.system
* Merge pull request `#2 <https://github.com/floweisshardt/atf/issues/2>`_ from ipa-fmw/feature/list_metrics
  publish rate working
* fix install tags
* fix groundtruth result merging
* add merge test to aggregate test results
* add scripts for recording and analysing all
* add missing merger
* introduce merge step
* use copy
* replace rosparam by rospy
* fix time analysis
* publish rate working
* unify analysis result and integrate details to presenter
* groundtruth for path_length
* reduce logs and handover groundtruth data
* change file properties to non-executive
* rename test_builder to analyser
* prevent rosbag play from spaming terminal
* fix groundtruth evaluation
* add groundtruth check for time
* remove lof
* remove dropbox config
* uploading is an optional parameter
* remove sim time and add recording/analysing prefix to generated test file
* remove debug print
* do not use rosrun and rospack find during build time
* use custom dropbox uploader config
* dropbox upload working
* generate tests in source dir
* resolve config path to ros packages
* Merge branch 'master' of /home/fmw/git/atf/src/atf
  Conflicts:
  .gitignore
  README.md
* deactivate obstacle distance
* deactivate obstacle distance
* moved obstacle_distance node to external package
* added all needed dependencies
* bugfix with sysargs
* debugging
* small code changes
* added separate time limits for recording and analysing
* added additional arguments and parameter to robot_config
* Merge branch 'feature_testing' into indigo_dev
* small changes
* added live status update for testblocks in analysing phase
* changed live status trigger
* changed atf_server to atf_status_server
* Merge pull request `#9 <https://github.com/floweisshardt/atf/issues/9>`_ from koellsch/liveupdate_feature
  added status update feature
* implemented server for status update
* added wait for subscriber
* fixed bug in message declaration
* some changes for test status update
* code beautification
* added check for joints and robot_description
* added time limit to recording test
* changed generation of test files
* renamed entry for move_group launch file
* changed output folder for generated tests
* small fix
* moved config files from atf_core to manipulation app
* added error handling
* removed comments
* any valid path is now possible as output directory
* code beautifications
* uncomment folder depletion
* multiple robots, scenes and tests can now be added to one testsuite
* added output of original test_list
* robot bringup launch file can now be defined through the test_generation_config.yaml file && added test repetition value
* added feature to measure distance to all objects and from link chain to objects
* fixed bug in folder management
* fixed folder management
* test files will be generated in building process
* added standalone launch file
* added metric "minimal distance to obstacles"
* deleted test_config.yaml
* deleted test_suite.yaml
* removed custom app from python code
* new format 2 in package.xml
* added example file for test_config
* renamed package cob_grasping to cob_grasping_app
* added example test_suite file
* new function for natural sorting
* added custom application test file
* test adjustments
* update due to new path length output
* beautification
* removed logging of timestamp
* lower error limit
* deleted dublicated planer id
* merged with web_interface branch
* new test parameter
* test updates
* updated test config files
* modified config files for more testing
* disabled .yaml output
* small fixes
* deleted testblock count from test_list (not necessary)
* fixed yaml output
* added count of testblocks to test_list.yaml
* fixed bug in error handling
* updated documentation
* changes for new recorder
* deleted test code
* changes for testing
* added rostest as class
* added generation of analysing_tests
* deleted some tests
* added check if program failed outside monitored testblocks
* added prettyxml output
* enabled testing
* small changes for testing
* first test try (still buggy)
* fixed bug at getting results
* fixed return value for calculate_path_length metric
* added handler for receiving the needed metrics
* small changes
* builded test_builder as class
* added robot name as argument
* added todo tags
* changed package name to atf_core
* Contributors: Björn Eistel, Felix Messmer, Florian Köhler, Florian Weisshardt, Loy van Beek, floweisshardt, fmessmer, fmw-hb, fmw-ss, hyb
