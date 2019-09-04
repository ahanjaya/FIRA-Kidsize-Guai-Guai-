#!/usr/bin/env python
import rospy
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

def callback(config, level):
    #rospy.loginfo("Received reconf call: " + str(config))
    return config

if __name__ == '__main__':
    rospy.loginfo("United Soccer Parameters - Running")
    rospy.init_node('united_soccer_params')

    # Create a D(ynamic)DynamicReconfigure
    player = DDynamicReconfigure("player")

    # Add variables (name, description, default value, min, max, edit_method)
    # ddynrec.add_variable("decimal", "float/double variable", 0.0, -1.0, 1.0)  
    player.add_variable("Pan_KP", "Pan KP", 0.0, 0, 1)
    player.add_variable("Pan_KI", "Pan KI", 0.0, 0, 1)
    player.add_variable("Pan_KD", "Pan KD", 0.0, 0, 0.000010)
    player.add_variable("Tilt_KP", "Tilt KP", 0.0, 0, 1)
    player.add_variable("Tilt_KI", "Tilt KI", 0.0, 0, 1)
    player.add_variable("Tilt_KD", "Tilt KD", 0.0, 0, 0.000010)
    player.add_variable("Pan_Step", "Pan Step", 0.0, 0, 0.2)
    player.add_variable("Tilt_Step", "Tilt Step", 0.0, 0, 0.5)
    player.add_variable("Pan_Kick", "Pan Kick", 0.0, -1.5, 0)
    player.add_variable("Tilt_Kick", "Tilt Kick", 0.0, -2, 0)
    player.add_variable("Body_KP", "Body KP", 0.0, 0, 1)
    player.add_variable("KP_Ball_Pos_X", "KP Ball Pos X", 0.0, 0, 1)
    player.add_variable("KP_Ball_Pos_Y", "KP Ball Pos Y", 0.0, 0, 1)
    player.add_variable("KP_Compass_X", "KP Compass X", 0.0, 0, 1)
    player.add_variable("KP_Compass_Y", "KP Compass Y", 0.0, 0, 1)
    player.add_variable("KP_Compass_A", "KP Compass A", 0.0, 0, 1)

    ball = DDynamicReconfigure("vision/ball")    
    ball.add_variable("H_Max", "H Max", 255, 0, 255)
    ball.add_variable("H_Min", "H Min", 0, 0, 255)
    ball.add_variable("S_Max", "S Max", 255, 0, 255)
    ball.add_variable("S_Min", "S Min", 0, 0, 255)
    ball.add_variable("V_Max", "V Max", 255, 0, 255)
    ball.add_variable("V_Min", "V Min", 0, 0, 255)
    ball.add_variable("Erode", "Erode", 0, 0, 15)
    ball.add_variable("Dilate", "Dilate", 0, 0, 15)

    ballD = DDynamicReconfigure("vision/ball/debug")
    ballD.add_variable("debug_ball", "Debug Ball", False)
    ballD.add_variable("ball_no", "Selected Ball", 0, 0, 15)
    ballD.add_variable("ball_area_min", "Ball Area Min", 0.0, 0, 1)
    ballD.add_variable("ball_area_max", "Ball Area Max", 0, 0, 15)
    ballD.add_variable("ball_area_ratio", "Ball Area Ratio", 0.0, 0, 1)
    ballD.add_variable("ball_wh_ratio_min", "Ball White Area Ratio Min", 0.0, 0, 1)
    ballD.add_variable("ball_wh_ratio_max", "Ball White Area Ratio Max", 0.0, 0, 2)
    ballD.add_variable("ball_percent_white", "Ball Percent White", 0, 0, 100)


    field = DDynamicReconfigure("vision/field") 
    field.add_variable("FH_Max", "FH Max", 255, 0, 255)
    field.add_variable("FH_Min", "FH Min", 0, 0, 255)
    field.add_variable("FS_Max", "FS Max", 255, 0, 255)
    field.add_variable("FS_Min", "FS Min", 0, 0, 255)
    field.add_variable("FV_Max", "FV Max", 255, 0, 255)
    field.add_variable("FV_Min", "FV Min", 0, 0, 255)
    field.add_variable("FErode", "FErode", 0, 0, 15)
    field.add_variable("FDilate", "FDilate", 0, 0, 15)

    # Start the server
    player.start(callback)
    ball.start(callback)
    ballD.start(callback)
    field.start(callback)
    rospy.spin()
