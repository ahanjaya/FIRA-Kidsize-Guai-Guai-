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
half_frame_w = (frame_w / 2) + 160
half_frame_w1 = (frame_w / 2) - 160
half_frame_h = (frame_h / 2) + 100

cone_centre_x = -1
cone_centre_y = -1
cone_y_min = -1
cone_y_max = -1
cone_x_min = -1
cone_x_max = -1

def cone_pos_callback(pos_msg):
	global cone_centre_x, cone_centre_y, \
		cone_y_min, cone_y_max, cone_x_min, cone_x_max
	cone_centre_x = pos_msg.data[0]
	cone_centre_y = pos_msg.data[1]
	cone_x_min = pos_msg.data[2]
	cone_x_max = pos_msg.data[3]
	cone_y_min = pos_msg.data[4]
	cone_y_max = pos_msg.data[5]

obstacle = False
def check_obstacle():
	global obstacle
	print(obstacle)
	if cone_y_max <= half_frame_h:
		obstacle = False
	else:
		obstacle = True

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
	print("Mission Possible Player - Running")
	rospy.init_node("mission_possible_player")
	rospy.wait_for_service("/srv_controller")
	
	global head_pub, motion_vel_pub, motion_state_pub
	motion_vel_pub = rospy.Publisher("/motion/cmd_vel", Twist, queue_size=1)
	motion_state_pub = rospy.Publisher("/motion/state", String, queue_size=1)
	head_pub = rospy.Publisher("/head/pos", Float32MultiArray, queue_size=1)
	cone_pos_sub = rospy.Subscriber("/mission_possible/cone/position", Int32MultiArray, cone_pos_callback)
	button_sub = rospy.Subscriber("/button/state", Int32MultiArray, button_callback)

	print("Mission Possible Player - Running")
	time.sleep(0.3)
	motion_state_pub.publish("stand")
	global freq
	freq = 50
	rate = rospy.Rate(freq)
	state = "forward"
	last_state = state
	lock_forward = False
	play = False
	button_pressed = [0, 0]
	no_obstacle = 0
	move = 0
	count_forward = 0
	count_not_obstacle = 0
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
					state = "forward"
					play = True
				button_pressed[0] = 0
		
		#///////////////////////////////////////////////////////////////////////
		#//////////////.............Role of execution............///////////////
		#///////////////////////////////////////////////////////////////////////
		tilt = rospy.get_param("/mission_possible_params/Head_Tilt")
		head_move(0.0, tilt)			
		if play :
			# print(state)
			# print(last_state)
			# print(lock_forward)
			print(move)
			check_obstacle()
			if state == "forward":
				# if last_state == "walk_left" or last_state == "walk_right":
				# 	lock_forward = True

				# if lock_forward == True:
				# 	walk(0.03, 0.00, 0.00)
				# 	count_forward += 1
				# 	if count_forward >= 5:
				# 		if move == 0:
				# 			state = "walk_left"
				# 		else:
				# 			state = "walk_right"
				# 		count_forward = 0
				# 		lock_forward = False
				# else:
				print("xxxX")
				if not obstacle:
					if last_state == "walk_left":
						walk(0.03, -0.01, 0.00)
					if last_state == "walk_right":
						walk(0.03, 0.01, 0.00)
					if last_state == "forward":
						walk(0.03, 0.00, 0.00)
				else:
					if move == 0:
						state = "walk_left"
					elif move ==1:
						state = "walk_right"

			elif state == "walk_left":
				walk(0.00, 0.030, 0.00) # left y
				if not obstacle and move == 0:
					count_not_obstacle += 1
				else:
					count_not_obstacle = 0

				if count_not_obstacle > 3:
					move = 1
					count_not_obstacle = 0
					state = "forward"
				last_state = "walk_left"
					
			elif state == "walk_right":
				walk(0.00, -0.030, 0.00) # right y
				if not obstacle and move == 1:
					count_not_obstacle += 1
				else:
					count_not_obstacle = 0

				if count_not_obstacle > 3:
					move = 0
					count_not_obstacle = 0
					state = "forward"
				last_state = "walk_right"
			
			
						
		rate.sleep()
	print("Mission Possible Player - Shut Down")
	rospy.on_shutdown(kill_node)

if __name__ == "__main__":
	main()