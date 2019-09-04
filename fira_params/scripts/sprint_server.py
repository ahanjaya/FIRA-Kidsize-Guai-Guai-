#!/usr/bin/env python
import rospy
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

def callback(config, level):
    #rospy.loginfo("Received reconf call: " + str(config))
    return config

if __name__ == '__main__':
    rospy.loginfo("Sprint Parameters - Running")
    rospy.init_node('sprint_params')

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
    player.add_variable("Scan_Rate", "Scan Rate", 0, 0, 10)
    player.add_variable("Body_Forward_KP", "Body Forward KP", 0.0, 0, 1)
    player.add_variable("Body_Backward_KP", "Body Backward KP", 0.0, 0, 1)

    vision = DDynamicReconfigure("vision")
    vision.add_variable("Vote", "Accumulator", 0, 0, 100)
    vision.add_variable("Min_Length", "Min Length", 0, 0, 300)
    vision.add_variable("Max_Gap", "Max Gap", 0, 0, 300)

    # Start the server
    player.start(callback)
    vision.start(callback)
    rospy.spin()
