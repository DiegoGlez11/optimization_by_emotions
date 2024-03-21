/*
 * startServerNode.cpp
 *
 *  Created on: Jun 27, 2020
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

   SimulationController sim;

   int numRays = sim.getNumSensors();
   int numActuators = sim.getNumActuators();
   int numInputs = sim.getNumInputs();

   vector<pair<double, double>> outputRanges;
   outputRanges.push_back(make_pair(-0.8, 1.5));   // Velocidad lineal
   outputRanges.push_back(make_pair(-0.35, 0.35)); // Velocidad angular

   /* +2, si la dif. x, y a la meta son una entrada*/
   ArloDriver *driver = new NeuroControllerDriver(numInputs, numActuators, outputRanges);

   // If there is no parameters, then the system will run in server mode.
   // That is, it will stay running for any request to evaluate an arlo driver.
   if (argc <= 1)
   {
      sim.setDriver(driver);
      ROS_INFO("Ready to evaluate Xolobot Drivers, give me the weights...");
      ros::spin(); // Use this to keep the server running.
   }
   else
   {
      // If there are two or more parameters, the server will only run the
      // given driver as determined by the weights file.
      ifstream file(argv[1]);
      if (!file.good())
      {
         cerr << "\nThe file " << argv[1] << " does no exist.\n";
         cerr << "Bye.\n\n";

         return 0;
      }

      string weightsfile(argv[1]);
      driver->setParameters(weightsfile.c_str()); // Carga el archivo de pesos.
      sim.startSimulation(driver, 12, 0.65);
   }

   ROS_INFO("The simulation has ended.\n");

   return (0);
}
