#! /usr/bin/env python
import numpy as np
import math
import time
import rospy
import roslib
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32, Int32, Bool, Int32MultiArray, Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

frame_w = rospy.get_param("/usb_cam/image_width")
frame_h = rospy.get_param("/usb_cam/image_height")
half_frame_w = (frame_w / 2) #+ 50
half_frame_h = (frame_h / 2) + 100

green_centre_x = -1
green_centre_y = -1
green_y_min = -1
green_y_max = -1
green_x_min = -1
green_x_max = -1

def green_pos_callback(pos_msg):
	global green_centre_x, green_centre_y, \
		green_y_min, green_y_max, green_x_min, green_x_max
	green_centre_x = pos_msg.data[0]
	green_centre_y = pos_msg.data[1]
	green_x_min = pos_msg.data[2]
	green_x_max = pos_msg.data[3]
	green_y_min = pos_msg.data[4]
	green_y_max = pos_msg.data[5]

blue_centre_x = -1
blue_centre_y = -1
blue_y_min = -1
blue_y_max = -1
blue_x_min = -1
blue_x_max = -1

def blue_pos_callback(pos_msg):
	global blue_centre_x, blue_centre_y, \
		blue_y_min, blue_y_max, blue_x_min, blue_x_max
	blue_centre_x = pos_msg.data[0]
	blue_centre_y = pos_msg.data[1]
	blue_x_min = pos_msg.data[2]
	blue_x_max = pos_msg.data[3]
	blue_y_min = pos_msg.data[4]
	blue_y_max = pos_msg.data[5]

line_centre_x = -1
line_centre_y = -1
line_y_min = -1
line_y_max = -1
line_x_min = -1
line_x_max = -1

def line_pos_callback(pos_msg):
	global line_centre_x, line_centre_y, \
		line_y_min, line_y_max, line_x_min, line_x_max
	line_centre_x = pos_msg.data[0]
	line_centre_y = pos_msg.data[1]
	line_x_min = pos_msg.data[2]
	line_x_max = pos_msg.data[3]
	line_y_min = pos_msg.data[4]
	line_y_max = pos_msg.data[5]

obstacle = False
def check_obstacle():
	global obstacle
	if green_y_max <= half_frame_h and \
		blue_y_max <= half_frame_h:
		obstacle = False
	else:
		obstacle = True

line = False
def check_line():
	global line
	if line_y_max <= half_frame_h:
		line = False
	else:
		line = True


button = [0, 0]

def button_callback(pos_msg):
	global button
	button[0] = pos_msg.data[0]
	button[1] = pos_msg.data[1]

def head_move(head_pan, head_tilt):
	global pos_pan, pos_tilt
	pos_pan = head_pan
	pos_tilt = head_tilt
	head_pos = Float32MultiArray()
	head_pos.data = [pos_pan, pos_tilt]
	head_pub.publish(head_pos)

def walk(x, y, a):
	velocity = Twist()
	velocity.linear.x = x
	velocity.linear.y = y
	velocity.linear.z = a
	motion_vel_pub.publish(velocity)

def kill_node():
	rospy.signal_shutdown("shutdown time.") 

def main():
	print("Obstacle Run Player - Running")
	rospy.init_node("united_soccer_player")
	rospy.wait_for_service("/srv_controller")
	
	global head_pub, motion_vel_pub, motion_state_pub
	motion_vel_pub = rospy.Publisher("/motion/cmd_vel", Twist, queue_size=1)
	motion_state_pub = rospy.Publisher("/motion/state", String, queue_size=1)
	head_pub = rospy.Publisher("/head/pos", Float32MultiArray, queue_size=1)
	green_pos_sub = rospy.Subscriber("/obstacle_run/green/position", Int32MultiArray, green_pos_callback)
	blue_pos_sub = rospy.Subscriber("/obstacle_run/blue/position", Int32MultiArray, blue_pos_callback)
	line_pos_sub = rospy.Subscriber("/obstacle_run/line/position", Int32MultiArray, line_pos_callback)
	button_sub = rospy.Subscriber("/button/state", Int32MultiArray, button_callback)

	print("Obstacle Run Player - Running")
	time.sleep(0.3)
	motion_state_pub.publish("stand")
	global freq
	freq = 50
	rate = rospy.Rate(freq)
	state = "initial"
	play = False
	button_pressed = [0, 0]
	no_obstacle = 0
	move = 1
	tilt = -0.6
	head_move(0.0, -0.6)

	while not rospy.is_shutdown():
		if button[0] == 1:
			button_pressed[0] = 1
		else:
			if button_pressed[0] == 1:
				if play:
					motion_state_pub.publish("sit")
					print("Sit")
					play = False
				else:
					motion_state_pub.publish("stand")
					print("Stand")
					state = "initial"
					play = True
				button_pressed[0] = 0
		print("line: ",line)
		print("obstacle: ",obstacle)
		print("move: ",move)
		#///////////////////////////////////////////////////////////////////////
		#//////////////.............Role of execution............///////////////
		#///////////////////////////////////////////////////////////////////////
		# tilt = rospy.get_param("/obstacle_run_params/Head_Tilt")
		# head_move(0.0, tilt)			
		if play :
			print(state)
			check_obstacle()
			check_line()
			if state == "initial":
				if not line and not obstacle:
					no_obstacle = 0
					walk(0.03, 0.00, 0.00)
				elif line:
					if line_centre_x > half_frame_w and line_x_min != -1:
						move = 0
						state = "walk_left"
					elif line_centre_x <= half_frame_w and line_x_max != -1:
						move = 1
						state = "walk_right"
				else:
					# walk(0.00,0.00,0.00)
					if move == 0:
						if not obstacle:
							move = 1
						state = "walk_left"
					else:
						if not obstacle:
							move = 0
						state = "walk_right"

			elif state == "walk_left":
				# if blue_y_max >= (frame_h-1) :
				# back = -0.02
				# else:
					# back = 0.00
				walk(0.00, 0.020, -0.03) # left y
				state = "initial"
					
			elif state == "walk_right":
				# if blue_y_max >= (frame_h-1) :
				# back = -0.02
				# else:
					# back = 0.00
				walk(0.00, -0.020, 0.03) # right y
				state = "initial"
			

			
		rate.sleep()
	print("Obstacle Run Player - Shut Down")
	rospy.on_shutdown(kill_node)

if __name__ == "__main__":
	main()