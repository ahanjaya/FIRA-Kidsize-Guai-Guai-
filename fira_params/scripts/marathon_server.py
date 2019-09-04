#!/usr/bin/env python
import rospy
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

def callback(config, level):
    #rospy.loginfo("Received reconf call: " + str(config))
    return config

if __name__ == '__main__':
    rospy.loginfo("Marathon Parameters - Running")
    rospy.init_node('marathon_params')

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
    player.add_variable("Tilt_Angle", "Tilt Angle", 0.0, -2, 0)
    player.add_variable("Scan_Rate", "Scan Rate", 0, 0, 10)
    player.add_variable("Body_KP", "Body KP", 0.0, 0, 1)

    line = DDynamicReconfigure("line")
    line.add_variable("H_Max", "H Max", 255, 0, 255)
    line.add_variable("H_Min", "H Min", 0, 0, 255)
    line.add_variable("S_Max", "S Max", 255, 0, 255)
    line.add_variable("S_Min", "S Min", 0, 0, 255)
    line.add_variable("V_Max", "V Max", 255, 0, 255)
    line.add_variable("V_Min", "V Min", 0, 0, 255)
    line.add_variable("Min_Size", "Min Size", 0, 0, 100000)

    marker = DDynamicReconfigure("marker")
    marker.add_variable("H_Max", "H Max", 255, 0, 255)
    marker.add_variable("H_Min", "H Min", 0, 0, 255)
    marker.add_variable("S_Max", "S Max", 255, 0, 255)
    marker.add_variable("S_Min", "S Min", 0, 0, 255)
    marker.add_variable("V_Max", "V Max", 255, 0, 255)
    marker.add_variable("V_Min", "V Min", 0, 0, 255)
    marker.add_variable("Min_Size", "Min Size", 0, 0, 100000)

    # Start the server
    player.start(callback)
    line.start(callback)
    marker.start(callback)
    rospy.spin()
