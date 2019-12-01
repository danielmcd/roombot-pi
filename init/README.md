# roombot-pi startup scripts

Initialization scripts allow the robot to start on boot.

To install the init scripts, copy them to the /etc/init.d directory.

`cd /etc/init.d`

`sudo ln -s /home/pi/catkin_ws/src/roombot-pi/init/roscore .`

`sudo ln -s /home/pi/catkin_ws/src/roombot-pi/init/roomba_teleop .`

`sudo update-rc.d roscore defaults`

`sudo update-rc.d roomba_teleop defaults`
