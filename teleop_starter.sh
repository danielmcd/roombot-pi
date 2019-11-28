#!/bin/bash
while [ 1 ]; do

  if [ $(ip add sh dev wlan0 | grep inet | wc -l) -ne 0 ]; then
     break
  fi

  sleep 1

done

source /home/pi/catkin_ws/devel/setup.bash
#export ROS_MASTER_URI=http://10.0.0.102:11311/
#export ROS_MASTER_URI=http://10.0.0.107:11311/
export ROS_MASTER_URI=http://10.0.0.108:11311/
#export ROS_MASTER_URI=http://192.168.0.196:11311/
#export ROS_MASTER_URI=http://192.168.2.3:11311/
export ROS_IP=`hostname --all-ip-addresses | xargs`

roslaunch --wait roombot-pi robot.launch > /home/pi/robot.log 2>&1 &
