#!/usr/bin/env python
import argparse
import unittest
import rostest
import glob
import rospkg
import sys
import os
import subprocess


class Recorder():
    def __init__(self):
        pass

    def record(self, pkg, test_generation_config_file, dry_run):
        cmake_prefix_path = os.environ['CMAKE_PREFIX_PATH']

        path_to_build_space = None
        for directory in cmake_prefix_path.split(":"):
            command="catkin locate --workspace " + directory + " -be " + pkg
            try: 
                path_to_build_space = subprocess.check_output(command, shell=True, universal_newlines=True)
            except subprocess.CalledProcessError as e:
                #print "output=", e.output
                #print "returncode=", e.returncode
                continue # continue searching in next directory
            path_to_build_space = path_to_build_space.rstrip() # remove trailing new line in result string from 'catkin locate'
            break # found package

        # check overall search result
        if path_to_build_space != None:
            print("found package '%s' in '%s'"%(pkg, path_to_build_space))
        else:
            print("Could not find package '%s' in current CMAKE_PREFIX_PATH '%s'"%(pkg, cmake_prefix_path))
            sys.exit(1)

        # get all recording files
        test_generation_config_file_replaced = test_generation_config_file
        # replace directory "/" with "_"
        test_generation_config_file_replaced = test_generation_config_file_replaced.replace("/", "_")
        # replace "*.yaml" with "*_yaml"
        test_generation_config_file_replaced = test_generation_config_file_replaced.replace(".", "_")
        path_to_test_files = os.path.join(path_to_build_space, "test_generated", test_generation_config_file_replaced)
        filenames = glob.glob(os.path.join(path_to_test_files, "recording_*" + args.test + "*.test"))
        if not filenames:
            print("No files found using test generation config file '%s'" % test_generation_config_file)
            sys.exit(1)
        filenames.sort()  # sort tests alphabetically
        print("found %d files for '%s':\n%s" % (len(filenames), pkg, str(filenames)))

        # record all
        counter = 1
        for f in filenames:
            print("\n--> recording " + str(counter) + "/" + str(len(filenames)) + " (" + str(os.path.basename(f)) + ")")
            command = "roslaunch " + os.path.join(path_to_test_files, f) + " execute_as_test:=false"
            if dry_run:
                print("dry run:", command)
            else:
                print("execute:", command)
                subprocess.call(command, shell=True)
            counter += 1
        return True

class TestRecording(unittest.TestCase):

    def test_recording_results(self):
        recorder = Recorder()
        self.assertTrue(recorder.record(args.pkg, args.test_generation_config_file, args.dry_run), "Could not record results.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Manual exection of ATF recording phase.', formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('pkg', type=str,
                        help='test package name')
    parser.add_argument('-g', dest='test_generation_config_file',
                        default='atf/test_generation_config.yaml',
                        help='path to test_generation_config file, relative to package root')
    parser.add_argument('-t', dest='test',
                        default="ts*",
                        help='test identifier, e.g. \n'
                         + '* or ts*                  --> record all tests (default)\n'
                         + 'ts0_*                     --> record all tests with ts0\n'
                         + 'ts0_*_r0_*                --> record all tests with ts0 and r0\n'
                         + 'ts0_c0_r0_e0_s0_0         --> record a single test ts0_c0_r0_e0_s0_0\n'
                         + 'ts0_c0_r0_e0_s0_*         --> record all iterations of test ts0_c0_r0_e0_s0_*\n'
                        )
    parser.add_argument('-e', dest='execute_as_test', action='count',
                        help='execute as rostest')
    parser.add_argument('-d', dest='dry_run', action='count',
                        help='execute dry run')

    args, unknown = parser.parse_known_args()
    if args.execute_as_test:
        rostest.rosrun("atf_core", 'recording', TestRecording)
    else:
        args = parser.parse_args() # strictly parse only known arguments again. will raise an error if unknown arguments are specified
        print("recording test in package '%s' with test generation file '%s'"%(args.pkg, args.test_generation_config_file))
        recorder = Recorder()
        recorder.record(args.pkg, args.test_generation_config_file, args.dry_run)
