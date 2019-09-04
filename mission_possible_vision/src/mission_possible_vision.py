#!/usr/bin/env python
import cv2 as cv
import numpy as np
import math
import rospy
import roslib
import time
from std_msgs.msg import String, Float32, Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()

total_roi = 1
global rgb_img
rgb_img = np.zeros((640, 480, 3), np.uint8)
# img_file = "/home/barelangfc/my_photo-2.jpg"
# rgb_img = cv.imread(img_file)

def nothing(x):
    pass

def image_callback(img_msg):
    global bridge
    try:
        global rgb_img
        rgb_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
        
    except CvBridgeError as e:
        print(e)

def kill_node():
	rospy.signal_shutdown("shutdown time.") 

def main():
    print("Mission Possible Detector - Running")
    rospy.init_node("mission_possible_vision")
    time.sleep(.5)
    global cone_pos_pub, cone_bin_img_pub, blue_pos_pub, blue_bin_img_pub, line_pos_pub, line_bin_img_pub, rslt_img_pub
    image_sub = rospy.Subscriber("/usb_cam/image_raw/", Image, image_callback)
    cone_pos_pub = rospy.Publisher("/mission_possible/cone/position", Int32MultiArray, queue_size=1)
    cone_bin_img_pub = rospy.Publisher("/mission_possible/cone/binary_img", Image, queue_size=1)
    rslt_img_pub = rospy.Publisher("/mission_possible/result_img", Image, queue_size=1)
    kernel = cv.getStructuringElement(cv.MORPH_RECT,(2,2))
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        # no = rospy.get_param("/mission_possible_params/Photo_No")
        # img_file = "/home/barelangfc/my_photo-"+ str(no) + ".jpg"
        
        global rgb_img
        # rgb_img = cv.imread(img_file)
        result_img = rgb_img.copy()
        cone_h_max = rospy.get_param("/mission_possible_params/CH_Max")
        cone_h_min = rospy.get_param("/mission_possible_params/CH_Min")
        cone_s_max = rospy.get_param("/mission_possible_params/CS_Max")
        cone_s_min = rospy.get_param("/mission_possible_params/CS_Min")
        cone_v_max = rospy.get_param("/mission_possible_params/CV_Max")
        cone_v_min = rospy.get_param("/mission_possible_params/CV_Min")
        blob_size = rospy.get_param("/mission_possible_params/Blob_Size")
        
        hsv_img = cv.cvtColor(rgb_img, cv.COLOR_BGR2LAB)

        lower_cone = np.array([cone_h_min, cone_s_min, cone_v_min])
        upper_cone = np.array([cone_h_max, cone_s_max, cone_v_max])
        cone_bin_img = cv.inRange(hsv_img, lower_cone, upper_cone)

        cone_color = (0, 255, 255)
        cone_centre_x = -1
        cone_centre_y = -1
        cone_x_min = -1
        cone_x_max = -1
        cone_y_min = -1
        cone_y_max = -1
      
        _, cone_contours, _ = cv.findContours(cone_bin_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if len(cone_contours) > 0:
            cone_cntr = sorted(cone_contours, key=cv.contourArea, reverse=True)[0]
            cone_cntr_area = cv.contourArea(cone_cntr)
            if cone_cntr_area > blob_size:
                box_x, box_y, box_w, box_h = cv.boundingRect(cone_cntr)
                cone_centre_x = box_x + box_w / 2
                cone_centre_y = box_y + box_h / 2
                cone_y_min = box_y
                cone_y_max = box_y + box_h
                cone_x_min = box_x
                cone_x_max = box_x + box_w
                cv.rectangle(result_img, (box_x, box_y), (box_x + box_w, box_y + box_h), (128,128,0), 1)
                cv.circle(result_img, (cone_centre_x, cone_centre_y), 5, (255, 255, 0), -1)
                cv.circle(result_img, (cone_x_min, cone_centre_y), 5, (255, 0, 0), -1)
                cv.circle(result_img, (cone_x_max, cone_centre_y), 5, (0, 0, 255), -1)
                cv.circle(result_img, (cone_centre_x, cone_y_min), 5, (255, 0, 0), -1)
                cv.circle(result_img, (cone_centre_x, cone_y_max), 5, (0, 0, 255), -1)

        cone_pos = Int32MultiArray()
        cone_pos.data = [cone_centre_x, cone_centre_y, cone_x_min, cone_x_max, cone_y_min, cone_y_max]

        cone_pos_pub.publish(cone_pos)
        cone_bin_img_pub.publish(bridge.cv2_to_imgmsg(cone_bin_img, "mono8"))
        rslt_img_pub.publish(bridge.cv2_to_imgmsg(result_img, "bgr8"))
        rate.sleep()
    # cv.destroyAllWindows()
    print("Mission Possible Detector - Shutdown")
    rospy.on_shutdown(kill_node)

if __name__ == "__main__":
    main()