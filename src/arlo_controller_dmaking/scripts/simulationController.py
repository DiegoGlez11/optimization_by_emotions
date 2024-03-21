#!/usr/bin/env python3


import rospy
import numpy as np
from arlo_controller_dmaking.srv import EvaluateDriver
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf

simulationState = {"hasTimeRunOut": False, "stuck": False, "finishLineCrossed": False, "targetReached": False, "crashingRisk":0}

class simulationController:
    is_driving = False
    stop_driving = False

    def __init__(self) :
        print(self)

        # connect to topic for control robot
        self.vel_pub_ = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        odom_sub_ = rospy.Subscriber("odom", Odometry, self.checkModelPosition)
        # clock_sub_ = rospy.Subscriber("clock", 1, &SimulationController::checkSimulationTime)
        # sonar_l_sub_ = rospy.Subscriber("arlo/laser/scan_left", 100, &SimulationController::checkSonarLeftValues, this)
        # sonar_c_sub_ = rospy.Subscriber("arlo/laser/scan_center", 100, &SimulationController::checkSonarCenterValues, this)
        # sonar_r_sub_ = rospy.Subscriber("arlo/laser/scan_right", 100, &SimulationController::checkSonarRightValues, this)
        # service = rospy.advertiseService("evaluate_driver", &SimulationController::evaluateDriver, this)
        # srvControl_robot = rospy.advertiseService("control_robot", &SimulationController::control_robot, this)

    def checkModelPosition(self, msg):
        print(".............")
        # print(msg.pose.pose.orientation)

        if simulationState["targetReached"]:
            return
        
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        
        euler = tf.transformations.euler_from_quaternion(quaternion)
        print(euler)


# class SimulationState:

#     distanceToGo = 1

#     def __init__(self,):
#         pass


# # +2 para dar la distancia a la meta como entrada. */
# sensorValues = np.array([0] * (NUM_RAYS * NUM_SONARS + 2))

# closestHitDistance = 1.3
# arloState = SimulationState()

# def checkSonarLeftValues(msg):
#     global NUM_RAYS, closestHitDistance, arloState
#     # print(msg)

#     for i in range(len(msg.ranges)):
#         sensorValues[i + 0 * NUM_RAYS] = (3 - msg.ranges[i]) if msg.ranges[i] <= 3 else 0

#         if msg.ranges[i] < closestHitDistance and arloState.distanceToGo > 1:
#             closestHitDistance = msg.ranges[i]


# def checkSonarCenterValues(msg):
#     global NUM_RAYS, closestHitDistance, arloState
#     # print(msg)

#     for i in range(len(msg.ranges)):
#         sensorValues[i + 1 * NUM_RAYS] = (3 - msg.ranges[i]) if msg.ranges[i] <= 3 else 0
        
#         if msg.ranges[i] < closestHitDistance and arloState.distanceToGo > 1:
#             closestHitDistance = msg.ranges[i]


# def checkSonarRightValues(msg):
#     global NUM_RAYS, closestHitDistance, arloState
#     # print(msg)

#     for i in range(len(msg.ranges)):
#         sensorValues[i + 2 * NUM_RAYS] = (3 - msg.ranges[i]) if msg.ranges[i] <= 3 else 0
        
#         if msg.ranges[i] < closestHitDistance and arloState.distanceToGo > 1:
#             closestHitDistance = msg.ranges[i]



# if __name__ == "__main__":
    # rospy.init_node("arlocontroller_py_node")

    # rospy.Subscriber("/arlo/laser/scan_left", LaserScan, checkSonarLeftValues)
    # rospy.Subscriber("/arlo/laser/scan_center", LaserScan, checkSonarCenterValues)
    # rospy.Subscriber("/arlo/laser/scan_right", LaserScan, checkSonarRightValues)

    # rospy.spin()