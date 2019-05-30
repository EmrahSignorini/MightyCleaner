#!/bin/bash

usage() { echo "Usage: $0 [-m] [-rn <robot_name>] [-markers]" 1>&2; exit 1; }

while getopts ":mrn:markers:t:" o; do
    case "${o}" in
        rn)
            rn=${OPTARG}
            ;;
        markers)
            w=${OPTARG}
            ;;
        m)
            m=1
            ;;
    esac
done

if (( $OPTIND == 1 )); then
    rn='thymio10'
    markers=2
fi
if [ -z "${rn}" ]; then
    rn='thymio10'
fi
if [[ "${markers}" < 1 ]]; then
    markers=2
fi
if [[ "${t}" != "true" && "${t}" != "false" ]]; then
    #statements
    t='false'
fi
if [[ "${t}" == "true" ]]; then
    #statements
    w='wall'
fi

echo "running the following command:\n roslaunch MightyCleaner thymiolaunch.launch name:=${rn} markers:=${markers}"
if [[ ${m} == 1 ]]; then
    #statements
    catkin_make && source ~/catkin_ws/devel/setup.bash && roslaunch MightyCleaner thymiolaunch.launch name:="${rn}" markers:="${markers}" 2> /dev/null
else
    catkin build && source ~/catkin_ws/devel/setup.bash && roslaunch MightyCleaner thymiolaunch.launch name:="${rn}" markers:="${markers}" 2> /dev/null
fi
