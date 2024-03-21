/*
 * startNode.cpp
 *
 *  Created on: Apr 15, 2020
 *      Author: antonio
 */

#include "SimulationController.h"
#include "ArloDriver.h"
#include "NeuroControllerDriver.h"
#include <ros/ros.h>
#include <vector>
#include <utility>

using namespace std;

int main(int argc, char **argv)
{
   ros::init(argc, argv, "neurocontroller_node");

   if (argc < 2)
   {
      cout << "\nUsage: " << argv[0] << " <weightsfile>\n"
           << endl;
      return 0;
   }

   string pesos(argv[1]);

   SimulationController sim;

   int numRays = sim.getNumSensors();
   int numActuators = sim.getNumActuators();

   vector<pair<double, double>> outputRanges;
   outputRanges.push_back(make_pair(-0.25, 2));
   outputRanges.push_back(make_pair(-0.5, 0.5));

   ArloDriver *driver = new NeuroControllerDriver(numRays, numActuators, outputRanges);
   driver->setParameters(pesos.c_str()); // Carga el archivo de pesos.

   sim.startSimulation(driver, 50);

   ROS_INFO("The simulation has ended.");

   return (0);
}
