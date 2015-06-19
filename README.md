Pick &amp; place application for the Care-O-Bot 4

Various options are available via rosparam in grasping_test.launch:

- scene: Defines which scene to use. All available scenes are stored in the scene_config.yaml file
- arm: Defines the arm for the manipulation task
       Options:
         - left
         - right
- switch_arm: Defines if the arm should switch to the other after a successful manipulation
              Options: 
                - True
                - False
- wait_for_user: Defines if the program should wait for the user to start the manipulation or not
                 Options:
                   - True
                   - False
- object_dimensions: Sets the dimensions of the object which should be manipulated
                     Use: [diameter in x, diameter in y, height]
- load_obstacles: If additionl objects are defined in the scene_config.yaml file they can be loaded explicitly.
                  Options:
                    - all
                    - none
                    - "id of the object"
- eef_step: Sets the step width for the endefector in motion planning
- jump_threshold: Sets the jump threshold in motion planning
- planer_id: Sets the id of the planer witch will be used for motion planning
             Options:
               - SBLkConfigDefault
               - ESTkConfigDefault
               - LBKPIECEkConfigDefault
               - BKPIECEkConfigDefault
               - KPIECEkConfigDefault
               - RRTkConfigDefault
               - RRTConnectkConfigDefault
               - RRTstarkConfigDefault
               - TRRTkConfigDefault
               - PRMkConfigDefault
               - PRMstarkConfigDefault
- planning_method: Sets the planning method for the targets
                   Options:
                     - cartesian_linear: Only linear movement from point to point. Set target coordinates in x, y, z and additional waypoints
                     - cartesian_mixed: Use linear movement for grasp, lift, drop and retreat. Set target coordinates in x, y, z
                     - cartesian: All movements are planned in joint space. Set target coordinates in x, y, z
                     - joint: Set targets as poses in joint space
- joint_trajectory_speed: Sets the endeffector speed for joint trajectories
- max_error: Defines how many errors can be raised untill the program stops
- lift_height: Defines the height for lifting / dropping the object
- approach_distance: Defines the distance how far the object should be approached
- manipulation_repeats: Sets the number of manipulation repeats