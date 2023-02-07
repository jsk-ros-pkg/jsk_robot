#!/usr/bin/env bash

echo "waiting for roscore to come up"
. /opt/ros/noetic/setup.sh
ret=`rosnode list`
for i in `seq 0 60`
do
  if [ "$ret" != '' ]
  then
    break
  fi
  ret=`rosnode list`
  sleep 1
  echo "Waiting for roscore $i/60"
done

if [ "$ret" = '' ]
then
  echo "Roscore is down"
else
  sleep 10  # omajinai
  ret=`timeout 30.0 rostopic echo /calibrated/data -n1 | grep True`
  while true
  do
    if [ "$ret" = "True" ]
    then
      echo "Robot is successfully started."
      # initctl emit robot-is-up
      break
    fi
    ret=`timeout 30.0 rostopic echo /calibrated/data -n1 | grep True`
    sleep 1
    echo "Waiting for topic /calibrated is advertised"
    ret=`rosnode list`
    if [ "$ret" != '' ]
    then
      break
    fi
  done
fi
