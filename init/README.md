# roombot-pi startup scripts

Initialization scripts allow the robot to start on boot.

To install the init scripts, copy them to the /etc/init.d directory.

`cd catkin_ws/src/roombot-pi/init`

`sudo cp roscore roomba_teleop /etc/init.d`

`sudo update-rc.d roscore defaults`

`sudo update-rc.d roomba_teleop defaults`
