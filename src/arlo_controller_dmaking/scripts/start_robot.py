#!/usr/bin/env python3

import rospy, os, sys
from simulationController import simulationController
from NeuroControllerDriver import NeuroControllerDriver 

if __name__ == "__main__":
    rospy.init_node("neurocontroller")

    
    sim = simulationController()
    # num_output = sim.getNumActuators()
    # num_input = sim.getNumInputs()

    # Velocidad lineal, Velocidad angular
    # r = [[-0.8, 1.5],[-0.35, 0.35]]


    driver = NeuroControllerDriver()
    driver.save_model("test-1", 1)
    driver.load_weights("test-1", 1)
    sim.setDriver(driver)

    rospy.spin()

