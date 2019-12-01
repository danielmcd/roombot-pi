#!/usr/bin/python
#===================================================================
# Joy 10 Hz
#
# Subscribes to:
#   /joy
#
# Publishes to:
#   /joy10hz
#
# Reduce joystick message frequency to no more than 10 Hz
#===================================================================
from __future__ import division
import argparse
import atexit
import geometry_msgs.msg
import rospy
import sensor_msgs.msg
import std_msgs.msg
import sys

class Joy10HzNode(object):

    def __init__(self):
	rospy.init_node('joy10hz_node')
	rospy.myargv(sys.argv) # process ROS args and return the rest

	self.msg = None

	joy_topic = self.get_param("~joy", "/joy")
	out_topic = self.get_param("~out", "/joy10hz")
	self.freq = int(self.get_param("~hz", "10"))

	self.out_pub = rospy.Publisher(out_topic, sensor_msgs.msg.Joy, queue_size=50)
	joy_sub = rospy.Subscriber(joy_topic, sensor_msgs.msg.Joy, self.on_joy)


    def get_param(self, param, default):
	value = rospy.get_param(param, default)
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name(param), value)
	return value


    def run(self):
	r = rospy.Rate(self.freq) # 10hz 
	while not rospy.is_shutdown():
	    if self.msg:
		self.out_pub.publish(self.msg)
	
	    r.sleep()


    def on_joy(self, joy):
	self.msg = joy


if __name__ == '__main__':
    try:
	node = Joy10HzNode()
	node.run()
    except rospy.ROSInterruptException:
	pass


