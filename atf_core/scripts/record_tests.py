#!/usr/bin/env python
import glob
import rospkg
import sys
import os
import subprocess


if __name__ == '__main__':
    r = rospkg.RosPack()
    if len(sys.argv) == 2:
        pkg = sys.argv[1]
        test="*"
        test_generation_config_file = "atf/test_generation_config.yaml"
    elif len(sys.argv) == 3:
        pkg = sys.argv[1]
        test = sys.argv[2]
        test_generation_config_file = "atf/test_generation_config.yaml"
    elif len (sys.argv) == 4:
        pkg = sys.argv[1]
        test = sys.argv[2]
        test_generation_config_file = sys.argv[3]
    else:
        print "wrong arguments. Usage: 'record_tests.py <<pkg>> [<<test>>]'"
        print "e.g. 'record_tests.py atf_test'                                      --> record all tests"
        print "e.g. 'record_tests.py atf_test ts0_c0_r0_e0_s0_0'                    --> record test ts0_c0_r0_e0_s0_0"
        print "e.g. 'record_tests.py atf_test ts0_c0_r0_e0_s0_*'                    --> record all iterations of test ts0_c0_r0_e0_s0_*"
        print "e.g. 'record_tests.py atf_test ts0_*'                                --> record all tests with ts0"
        print "e.g. 'record_tests.py atf_test ts0_*_r0_*'                           --> record all tests with ts0 and r0"
        print "e.g. 'record_tests.py atf_test ts* atf/test_generation_config.yaml'  --> record all tests defined in atf/test_generation_config.yaml"
        sys.exit(1)
    print "recording test in package '%s' with test generation file '%s'"%(pkg, test_generation_config_file)

    cmake_prefix_path = os.environ['CMAKE_PREFIX_PATH']

    path_to_build_space = None
    for directory in cmake_prefix_path.split(":"):
        command="catkin locate --workspace " + directory + " -be " + pkg
        try: 
            path_to_build_space = subprocess.check_output(command, shell=True)
        except subprocess.CalledProcessError as e:
            #print "output=", e.output
            #print "returncode=", e.returncode
            continue # continue searching in next directory
        path_to_build_space = path_to_build_space.rstrip() # remove trailing new line in result string from 'catkin locate'
        break # found package

    # check overall search result
    if path_to_build_space != None:
        print "found package '%s' in '%s'"%(pkg, path_to_build_space)
    else:
        print "Could not find package '%s' in current CMAKE_PREFIX_PATH '%s'"%(pkg, cmake_prefix_path)
        sys.exit(1)
    
    # get all recording files
    test_generation_config_file_replaced = test_generation_config_file
    # replace directory "/" with "_"
    test_generation_config_file_replaced = test_generation_config_file_replaced.replace("/", "_")
    # replace "*.yaml" with "*_yaml"
    test_generation_config_file_replaced = test_generation_config_file_replaced.replace(".", "_")
    path_to_test_files = os.path.join(path_to_build_space, "test_generated", test_generation_config_file_replaced)
    filenames = glob.glob(os.path.join(path_to_test_files, "recording_*" + test + "*.test"))
    filenames.sort() # sort tests alphabetically
    print "found %d files for '%s':\n%s"%(len(filenames), pkg, str(filenames))

    # record all
    counter = 1
    for f in filenames:
        print "\n--> recording " + str(counter) + "/" + str(len(filenames)) + " (" + str(os.path.basename(f)) + ")"
        command = "roslaunch " + os.path.join(path_to_test_files, f) + " execute_as_test:=false"
        subprocess.call(command, shell=True)
        counter += 1
