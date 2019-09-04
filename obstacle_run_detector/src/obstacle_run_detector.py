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
    print("Obstacle Run Detector - Running")
    rospy.init_node("obstacle_run_detector")
    time.sleep(.5)
    global green_pos_pub, green_bin_img_pub, blue_pos_pub, blue_bin_img_pub, line_pos_pub, line_bin_img_pub, rslt_img_pub
    image_sub = rospy.Subscriber("/usb_cam/image_raw/", Image, image_callback)
    green_pos_pub = rospy.Publisher("/obstacle_run/green/position", Int32MultiArray, queue_size=1)
    green_bin_img_pub = rospy.Publisher("/obstacle_run/green/binary_img", Image, queue_size=1)
    blue_pos_pub = rospy.Publisher("/obstacle_run/blue/position", Int32MultiArray, queue_size=1)
    blue_bin_img_pub = rospy.Publisher("/obstacle_run/blue/binary_img", Image, queue_size=1)
    line_pos_pub = rospy.Publisher("/obstacle_run/line/position", Int32MultiArray, queue_size=1)
    line_bin_img_pub = rospy.Publisher("/obstacle_run/line/binary_img", Image, queue_size=1)
    rslt_img_pub = rospy.Publisher("/obstacle_run/result_img", Image, queue_size=1)
    kernel = cv.getStructuringElement(cv.MORPH_RECT,(2,2))
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        # no = rospy.get_param("/obstacle_run_params/Photo_No")
        # img_file = "/home/barelangfc/my_photo-"+ str(no) + ".jpg"
        
        global rgb_img
        # rgb_img = cv.imread(img_file)
        result_img = rgb_img.copy()
        green_h_max = rospy.get_param("/obstacle_run_params/YH_Max")
        green_h_min = rospy.get_param("/obstacle_run_params/YH_Min")
        green_s_max = rospy.get_param("/obstacle_run_params/YS_Max")
        green_s_min = rospy.get_param("/obstacle_run_params/YS_Min")
        green_v_max = rospy.get_param("/obstacle_run_params/YV_Max")
        green_v_min = rospy.get_param("/obstacle_run_params/YV_Min")
        blue_h_max = rospy.get_param("/obstacle_run_params/BH_Max")
        blue_h_min = rospy.get_param("/obstacle_run_params/BH_Min")
        blue_s_max = rospy.get_param("/obstacle_run_params/BS_Max")
        blue_s_min = rospy.get_param("/obstacle_run_params/BS_Min")
        blue_v_max = rospy.get_param("/obstacle_run_params/BV_Max")
        blue_v_min = rospy.get_param("/obstacle_run_params/BV_Min")
        line_h_max = rospy.get_param("/obstacle_run_params/LH_Max")
        line_h_min = rospy.get_param("/obstacle_run_params/LH_Min")
        line_s_max = rospy.get_param("/obstacle_run_params/LS_Max")
        line_s_min = rospy.get_param("/obstacle_run_params/LS_Min")
        line_v_max = rospy.get_param("/obstacle_run_params/LV_Max")
        line_v_min = rospy.get_param("/obstacle_run_params/LV_Min")
        line_dilate = rospy.get_param("/obstacle_run_params/LDilate")
        blob_size = rospy.get_param("/obstacle_run_params/Blob_Size")
        

        hsv_img = cv.cvtColor(rgb_img, cv.COLOR_BGR2LAB)

        lower_green = np.array([green_h_min, green_s_min, green_v_min])
        upper_green = np.array([green_h_max, green_s_max, green_v_max])
        green_bin_img = cv.inRange(hsv_img, lower_green, upper_green)

        green_color = (0, 255, 0)
        green_centre_x = -1
        green_centre_y = -1
        green_y_min = -1
        green_y_max = -1
        green_x_min = -1
        green_x_max = -1
      
        _, green_contours, _ = cv.findContours(green_bin_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if len(green_contours) > 0:
            green_cntr = sorted(green_contours, key=cv.contourArea, reverse=True)[0]
            green_cntr_area = cv.contourArea(green_cntr)
            if green_cntr_area > blob_size:
                box_x, box_y, box_w, box_h = cv.boundingRect(green_cntr)
                green_centre_x = box_x + box_w / 2
                green_centre_y = box_y + box_h / 2
                green_y_min = box_y
                green_y_max = box_y + box_h
                green_x_min = box_x
                green_x_max = box_x + box_w
                cv.rectangle(result_img, (box_x, box_y), (box_x + box_w, box_y + box_h), green_color, 1)
                cv.circle(result_img, (green_centre_x, green_centre_y), 5, (255, 255, 0), -1)
                cv.circle(result_img, (green_x_min, green_centre_y), 5, (255, 0, 0), -1)
                cv.circle(result_img, (green_x_max, green_centre_y), 5, (0, 0, 255), -1)
                cv.circle(result_img, (green_centre_x, green_y_min), 5, (255, 0, 0), -1)
                cv.circle(result_img, (green_centre_x, green_y_max), 5, (0, 0, 255), -1)


        green_pos = Int32MultiArray()
        green_pos.data = [green_centre_x, green_centre_y, green_x_min, green_x_max, green_y_min, green_y_max]

        lower_blue = np.array([blue_h_min, blue_s_min, blue_v_min])
        upper_blue = np.array([blue_h_max, blue_s_max, blue_v_max])
        blue_bin_img = cv.inRange(hsv_img, lower_blue, upper_blue)

        blue_color = (255, 0, 0)
        blue_centre_x = -1
        blue_centre_y = -1
        blue_y_min = -1
        blue_y_max = -1
        blue_x_min = -1
        blue_x_max = -1
      
        _, blue_contours, _ = cv.findContours(blue_bin_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if len(blue_contours) > 0:
            blue_cntr = sorted(blue_contours, key=cv.contourArea, reverse=True)[0]
            blue_cntr_area = cv.contourArea(blue_cntr)
            if blue_cntr_area > blob_size:
                box_x, box_y, box_w, box_h = cv.boundingRect(blue_cntr)
                blue_centre_x = box_x + box_w / 2
                blue_centre_y = box_y + box_h / 2
                blue_y_min = box_y
                blue_y_max = box_y + box_h
                blue_x_min = box_x
                blue_x_max = box_x + box_w
                cv.rectangle(result_img, (box_x, box_y), (box_x + box_w, box_y + box_h), blue_color, 1)
                cv.circle(result_img, (blue_centre_x, blue_centre_y), 5, (255, 255, 0), -1)
                cv.circle(result_img, (blue_x_min, blue_centre_y), 5, (255, 0, 0), -1)
                cv.circle(result_img, (blue_x_max, blue_centre_y), 5, (0, 0, 255), -1)
                cv.circle(result_img, (blue_centre_x, blue_y_min), 5, (255, 0, 0), -1)
                cv.circle(result_img, (blue_centre_x, blue_y_max), 5, (0, 0, 255), -1)

        blue_pos = Int32MultiArray()
        blue_pos.data = [blue_centre_x, blue_centre_y, blue_x_min, blue_x_max, blue_y_min, blue_y_max]

        lower_line = np.array([line_h_min, line_s_min, line_v_min])
        upper_line = np.array([line_h_max, line_s_max, line_v_max])
        line_bin_img = cv.inRange(hsv_img, lower_line, upper_line)
        line_bin_img = cv.dilate(line_bin_img, kernel, iterations = line_dilate)


        line_color = (0, 255, 255)
        line_centre_x = -1
        line_centre_y = -1
        line_y_min = -1
        line_y_max = -1
        line_x_min = -1
        line_x_max = -1
      
        _, line_contours, _ = cv.findContours(line_bin_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if len(line_contours) > 0:
            line_cntr = sorted(line_contours, key=cv.contourArea, reverse=True)[0]
            line_cntr_area = cv.contourArea(line_cntr)
            if line_cntr_area > blob_size:
                box_x, box_y, box_w, box_h = cv.boundingRect(line_cntr)
                line_centre_x = box_x + box_w / 2
                line_centre_y = box_y + box_h / 2
                line_y_min = box_y
                line_y_max = box_y + box_h
                line_x_min = box_x
                line_x_max = box_x + box_w
                cv.rectangle(result_img, (box_x, box_y), (box_x + box_w, box_y + box_h), line_color, 1)
                cv.circle(result_img, (line_centre_x, line_centre_y), 5, (255, 255, 0), -1)
                cv.circle(result_img, (line_x_min, line_centre_y), 5, (255, 0, 0), -1)
                cv.circle(result_img, (line_x_max, line_centre_y), 5, (0, 0, 255), -1)
                cv.circle(result_img, (line_centre_x, line_y_min), 5, (255, 0, 0), -1)
                cv.circle(result_img, (line_centre_x, line_y_max), 5, (0, 0, 255), -1)

        line_pos = Int32MultiArray()
        line_pos.data = [line_centre_x, line_centre_y, line_x_min, line_x_max, line_y_min, line_y_max]

        green_pos_pub.publish(green_pos)
        green_bin_img_pub.publish(bridge.cv2_to_imgmsg(green_bin_img, "mono8"))
        blue_pos_pub.publish(blue_pos)
        blue_bin_img_pub.publish(bridge.cv2_to_imgmsg(blue_bin_img, "mono8"))
        line_pos_pub.publish(line_pos)
        line_bin_img_pub.publish(bridge.cv2_to_imgmsg(line_bin_img, "mono8"))
        rslt_img_pub.publish(bridge.cv2_to_imgmsg(result_img, "bgr8"))
        rate.sleep()
    # cv.destroyAllWindows()
    print("Obstacle Run Detector - Shutdown")
    rospy.on_shutdown(kill_node)

if __name__ == "__main__":
    main()