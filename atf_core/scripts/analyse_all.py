#!/usr/bin/env python
import rospy
import rospkg
import sys
import os


if __name__ == '__main__':
    r = rospkg.RosPack()
    pkg = sys.argv[1]
    print "analysing all in package '" + pkg + "'"
    pkg_path = r.get_path(pkg)
    path_to_test_files = os.path.join(pkg_path, "test_generated/analysing")
    
    filenames = []
    for (dirpath, dirnames, list_of_files) in os.walk(path_to_test_files):
        for f in list_of_files:
            if f.endswith(".test"):
                filenames.append(f)

    print "found " + str(len(filenames)) + " files: " + str(filenames)
    
    # analyse all
    for f in filenames:
        command = "rostest " + pkg + " " + f
        os.system(command)
    
    # merge
    command = "rostest " + pkg + " " + "merging.test"
    os.system(command)
