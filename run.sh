#!/bin/bash

usage() { echo "Usage: $0 [-m] [-r <robot_name>] [-s <number>]" 1>&2; exit 1; }

while getopts "mr:s:t:" o; do
    case "${o}" in
        r)
            rn=${OPTARG}
            ;;
        s)
            markers=${OPTARG}
            ;;
        m)
            m=1
            ;;
        t) rot=${OPTARG}
            ;;
    esac
done

if (( $OPTIND == 1 )); then
    rn='thymio17'
    markers=2
fi
if [[ -z "${rn}" ]]; then
    rn='thymio17'
fi
if [[ "${markers}" < 1 ]]; then
    markers=2
fi
if [[ "${rot}" != "true" && "${rot}" != "false" ]]; then
    #statements
    rot='false'
fi


echo "running the following command:\n roslaunch MightyCleaner thymiolaunch.launch name:=${rn} markers:=${markers} rotate:=${rot}"
if [[ ${m} == 1 ]]; then
    #statements
    export ROS_MASTER_URI=http://"${rn}":11311
    catkin_make && source ~/catkin_ws/devel/setup.bash && roslaunch MightyCleaner thymiolaunch.launch name:="${rn}" markers:="${markers}" rotate:="${rot}"2> /dev/null
else
    export ROS_MASTER_URI=http://"${rn}":11311
    catkin build && source ~/catkin_ws/devel/setup.bash && roslaunch MightyCleaner thymiolaunch.launch name:="${rn}" markers:="${markers}" rotate:="${rot}" 2> /dev/null
fi
