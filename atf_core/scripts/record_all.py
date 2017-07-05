#!/usr/bin/env python
import rospkg
import sys
import os
import subprocess


if __name__ == '__main__':
    r = rospkg.RosPack()
    pkg = sys.argv[1]
    print "recording all in package '" + pkg + "'"
    pkg_path = r.get_path(pkg)
    path_to_test_files = os.path.join(pkg_path, "test_generated/recording")

    filenames = []
    for (dirpath, dirnames, list_of_files) in os.walk(path_to_test_files):
        for f in list_of_files:
            if f.endswith(".test"):
                filenames.append(f)

    print "found " + str(len(filenames)) + " files: " + str(filenames)

    # record all
    counter = 1
    for f in filenames:
        print "\n--> recording " + str(counter) + "/" + str(len(filenames)) + " (" + str(f) + ")"
        command = "rostest " + pkg + " " + f
        subprocess.call(command, shell=True)
        counter += 1
