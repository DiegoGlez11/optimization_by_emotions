#!/usr/bin/env python3

import rospy, os, sys
from simulationController import simulationController

if __name__ == "__main__":
    rospy.init_node("robot_controller")

    sim = simulationController()
    rospy.spin()

