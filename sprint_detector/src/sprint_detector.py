#!/usr/bin/env python
import numpy as np
import cv2 as cv
from cv2 import aruco
import time
import rospy
import roslib
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
global rgb_img
rgb_img = np.zeros((rospy.get_param("/usb_cam/image_height"), rospy.get_param("/usb_cam/image_width"), 3), np.uint8)

def img_sub_callback(img_msg):
    try:
        global rgb_img
        rgb_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        rospy.loginfo(e)

def kill_node():
    rospy.signal_shutdown("shutdown time.")

def make_coordinate(img, parameter):
    try:
        slope = parameter[0]
        intercept = parameter[1]
        y1 = img.shape[0]
        y2 = 0 #int(y1*1/100)
        x1 = int( (y1-intercept)/slope )
        x2 = int( (y2-intercept)/slope )
    except:
        x1 = x2 = y1 = y2 = 0
    return np.array([x1,y1,x2,y2])

def mid_points(line):
    cx = (line[0] + line[2]) / 2
    cy = (line[1] + line[3]) / 2
    return (int(cx), int(cy))

def main():
    rospy.loginfo("Sprint Detector - Running")
    rospy.init_node("sprint_detector")
    aruco_img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_sub_callback)
    #global aruco_img_pub, aruco_pos_pub
    aruco_img_pub = rospy.Publisher("/sprint/marker/image", Image, queue_size=1)
    lines_img_pub = rospy.Publisher("/sprint/lines/image", Image, queue_size=1)
    aruco_pos_pub = rospy.Publisher("/sprint/marker/position", Int32MultiArray, queue_size=1)
    lines_pos_pub = rospy.Publisher("/sprint/lines/position", Int32MultiArray, queue_size=1)

    lineParam = np.zeros(3, dtype=int)
    rate = rospy.Rate(50)
    time.sleep(0.3)
    while not rospy.is_shutdown():
        global rgb_img
        result_img = rgb_img.copy()
        gray_img = cv.cvtColor(rgb_img, cv.COLOR_BGR2GRAY)
        blur_img = cv.GaussianBlur(gray_img,(3,3),0)        
        med_val = np.median(blur_img) 
        lower = int(max(0, 0.7* med_val))
        upper = int(min(255,1.3 * med_val))
        edges_img = cv.Canny(blur_img, lower, upper)  

        lineParam[0] = rospy.get_param("/sprint_params/vision/Vote")
        lineParam[1] = rospy.get_param("/sprint_params/vision/Min_Length")
        lineParam[2] = rospy.get_param("/sprint_params/vision/Max_Gap")
        lines = cv.HoughLinesP(edges_img, 1, np.pi/180.0, lineParam[0], minLineLength=lineParam[1], maxLineGap=lineParam[2])
        centre = [-1, -1]
        if lines is not None:
            left_line = []
            right_line = []
            for line in lines:
                x1, y1, x2, y2 = line.reshape(4)
                # cv.line(result_img, (x1, y1), (x2, y2), (0,0,255), 1)

                parameters = np.polyfit((x1, x2), (y1, y2), 1)
                slope = parameters[0]
                intercept = parameters[1]
                # slope = (y2-y1)/(x2-x1)
                # intercept = y1-(slope*x1)
                # parameters = np.array([slope, intercept])
                if slope < 0:
                    # cv.line(result_img, (x1, y1), (x2, y2), (0,255,0), 2)
                    left_line.append(parameters)
                else:
                    # cv.line(result_img, (x1, y1), (x2, y2), (255,0,0), 2)
                    right_line.append(parameters)

            left_avg = np.average(left_line, axis=0)
            right_avg = np.average(right_line, axis=0)
            l1 = make_coordinate(result_img, left_avg)
            l2 = make_coordinate(result_img, right_avg)
            cv.line(result_img, (l1[0], l1[1]), (l1[2], l1[3]), (0,255,0), 2)
            cv.line(result_img, (l2[0], l2[1]), (l2[2], l2[3]), (255,0,0), 2)

            if l1.any() != 0 and l2.any() != 0:
                mid_l1 = mid_points(l1)
                mid_l2 = mid_points(l2)
                # cv.circle(result_img, mid_l1, 5, (0,0,0), -1)
                # cv.circle(result_img, mid_l2, 5, (0,0,0), -1)

                # middle lines
                l = mid_l1+mid_l2
                cv.line(result_img, (l[0], l[1]), (l[2], l[3]), (0,0,255), 2)
                centre = mid_points(l)
                cv.circle(result_img, centre, 5, (0,0,0), -1)

        lines_pos = Int32MultiArray()
        lines_pos.data = [centre[0], centre[1]]
        print(lines_pos)

        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_img, aruco_dict, parameters=parameters)
        cx = -1
        cy = -1
        size = -1
        if len(corners) > 0 and ids[0,0] == 1:
            M = cv.moments(corners[0])
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            size = int(M["m00"])
        marker_pos = Int32MultiArray()
        marker_pos.data = [cx, cy, size]
        result_img = aruco.drawDetectedMarkers(result_img, corners, ids)
        # rospy.loginfo("ArUco Marker : \n" + str(marker_pos))

        #global aruco_img_pub, aruco_pos_pub
        aruco_img_pub.publish(bridge.cv2_to_imgmsg(result_img, "bgr8"))
        lines_img_pub.publish(bridge.cv2_to_imgmsg(edges_img, "mono8"))
        aruco_pos_pub.publish(marker_pos)
        lines_pos_pub.publish(marker_pos)
        # rate.sleep()
    rospy.loginfo("Sprint Detector - Shut Down")
    rospy.on_shutdown(kill_node)

if __name__ == "__main__":
    main()
