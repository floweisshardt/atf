#!/bin/bash


if [ "$#" -ne "2" ]
then
  echo "not enought arguments. Usage: 'record_test.sh <<pkg>> <<test>>'"
  echo "e.g. 'record_test.sh atf_test ts0_c0_r0_e0_s0_0'"
  exit 1
fi

pkg=$1
test=$2
echo "Record test '$2' for package '$pkg'"

IFS=:; set -o noglob
for dir in $CMAKE_PREFIX_PATH""; do
  cd $dir
  output=$(catkin locate -be $pkg)
  ret=$?
  if [ $ret -ne 0 ]
  then
    #echo "package $pkg not found in $dir"
    continue
  else
    pkg_build_dir=$output
    #echo "found package $pkg in $pkg_build_dir"
    break
  fi
done

if [ -v $pkg_build_dir ]; then
  echo "Could not find package '$pkg'."
  exit 1
else
  echo "found package '$pkg' in '$pkg_build_dir'"
fi

roslaunch $pkg_build_dir/test_generated/recording/recording_$test.test execute_as_test:=false


