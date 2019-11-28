#!/usr/bin/python
from __future__ import division
import roslib
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
import sys

##### pin assignments
tilt_pin = 0
pan_pin = 1

class PanTilt:
    def __init__(self):
        try:
            self.sb = open('/dev/servoblaster', 'w')
        except (IOError):
            print "*** ERROR ***"
            print "Unable to open the device, check that servod is running"
            print "To start servod, run: sudo /etc/init.d/servoblaster.sh start"
            exit()

    def pwm(self, pin, angle):
        self.sb.write(str(pin) + '=' + str(int(angle)) + '\n')
        self.sb.flush()

    def go(self, pan_angle, tilt_angle):
	self.pwm(tilt_pin, self.map_tilt_angle(tilt_angle))
	self.pwm(pan_pin, self.map_pan_angle(pan_angle))

    def map_tilt_angle(self, angle):
	return map(angle, 0, 100, 170, 60)

    def map_pan_angle(self, angle):
	return map(angle, 0, 100, 60, 180)

# joy axes
#   0 is left stick left-right (+ left, - right)
#   1 is left stick up-down (+ up, - down)
#   2 is unused
#   3 is right stick up-down (+ up, - down)
#   4 is right stick left-right (+ left, - right)
#   5 is POV pad left-right (+ left, - right) (analog on)
#   6 is POV pad up-down (+ up, - down) (analog on)
# buttons
#   2 is X
#   7 is R1
#
# Left stick:
#   drive - forward/reverse only
#	push up to go forward with velocity proportional to up value
#	push down to go reverse with velocity proportional to down value
#	push left to rotate in place ccw with velocity proportional to left value
#	push right to rotate in place cw with velocity proportional to right value
#	push left stick up-left to rotate forward and left with velocity proportional to length of
#		up-left vector and radius proportional to angular deviation from straight up
#	push left stick up-right to rotate forward and right with velocity proportional to length of
#		up-right vector and radius proportional to angular deviation from straight up
#	same for down-left and down-right, but rotate reverse and left/right

def map(value, domainLow, domainHigh, rangeLow, rangeHigh):
    return ((value - domainLow) / (domainHigh - domainLow)) * (rangeHigh - rangeLow) + rangeLow

last_twist = None
tilt = 0
pan = 0
last_dt = 0
last_dp = 0
tilt_scale = 10
pan_scale = 10
rev_motor = 0
fire = 0
buttons_key = ""
buttons = None

def on_joy(joy):
    global last_twist, last_dt, last_dp, tilt, rev_motor, fire, buttons, buttons_key
    key = "".join([str(b) for b in joy.buttons])
    if key != buttons_key:  # test if anything changed since last time
	buttons = joy.buttons
	buttons_key = key
    left_stick_left = joy.axes[0]
    left_stick_up = joy.axes[1]
    right_stick_up = joy.axes[3]
    right_stick_left = joy.axes[4]
    pov_left = joy.axes[5]
    pov_up = joy.axes[6]

    rev_motor = joy.buttons[7]
    fire = joy.buttons[2]

    if left_stick_left != 0 and right_stick_left == 0:
	right_stick_left = left_stick_left

    vector_length = min(1.0, math.sqrt(left_stick_up*left_stick_up + right_stick_left*right_stick_left))

    velocity = int(500 * left_stick_up)  # drive forward at proportional velocity
#    if left_stick_up >= 0:
#	velocity = int(500 * vector_length)  # drive forward at proportional velocity
#    elif left_stick_up < 0:
#	velocity = int(-500 * vector_length)  # drive reverse at proportional velocity

    if right_stick_left == 0:
	radius = 32767  # drive straight
    elif left_stick_up == 0:
	radius = 1 if right_stick_left > 0 else -1  # rotate in place
	velocity = int(500 * abs(right_stick_left))  # drive at proportional velocity
    elif right_stick_left > 0:
	#radius = int(2000 * (1 - right_stick_left))  # rotate to the left
	radius = map(right_stick_left, 0, 1, 500, 2)  # rotate to the left
    else:
	#radius = int(-2000 * (1 + right_stick_left))  # rotate to the right
	radius = map(right_stick_left, -1, 0, -2, -500)  # rotate to the right

    last_dt = tilt_scale * right_stick_up
    last_dp = pan_scale * right_stick_left

    twist = Twist()
    twist.linear.y = velocity
    twist.angular.x = tilt
    twist.angular.z = radius

    last_twist = twist

def main(args):
    global last_twist, last_dt, last_dp, tilt, pan, rev_motor, fire, buttons
    pan_tilt = PanTilt()
    rospy.init_node('turret_teleop_node')
    joy_sub = rospy.Subscriber("/joy", Joy, on_joy)
    #twist_pub = rospy.Publisher("/twist", Twist, queue_size=1)
    twist_pub = rospy.Publisher("/twist", Twist)
    buttons_pub = rospy.Publisher("/buttons", String)
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
	rate.sleep()  # give ROS a chance to run
	# publish buttons first to avoid clobbering twist message
	if buttons is not None:
	    buttons_pub.publish(" ".join([str(b) for b in buttons]))
	    buttons = None
	    rate.sleep()  # give ROS a chance to run
	if abs(last_dt) > 0 or abs(last_dp) > 0:
	    tilt += last_dt
	    if tilt > 100:
		tilt = 100
		last_dt = 0
	    elif tilt < 0:
		tilt = 0
		last_dt = 0
	    pan += last_dp
	    if pan > 100:
		pan = 100
		last_dp = 0
	    elif pan < 0:
		pan = 0
		last_dp = 0
	    if last_twist is not None:
		twist = last_twist
	    else:
		twist = Twist()
	    twist.angular.x = tilt
	    pan_tilt.go(pan, tilt)
	    last_twist = None
	    twist_pub.publish(twist)
	elif last_twist is not None:
	    pan_tilt.go(pan, tilt)
	    twist = last_twist
	    last_twist = None
	    twist_pub.publish(twist)
	    
if __name__ == '__main__':
    main(sys.argv)
