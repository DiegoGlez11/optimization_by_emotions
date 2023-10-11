/*
 * SimulationController.h
 *
 *  Created on: Apr 14, 2020
 *      Author: antonio
 */

#ifndef SRC_SIMULATIONCONTROLLER_H_
#define SRC_SIMULATIONCONTROLLER_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>
#include "arlo_controller_dmaking/EvaluateDriver.h"
#include "arlo_controller_dmaking/control_robot.h"
#include "arlo_controller_dmaking/play_song.h"
#include "user_interaction/change_color.h"
#include <string>
#include <iostream>
#include <fmt/core.h>
#include <fmt/ranges.h>
#include <fmt/color.h>
#include "SimulationState.h"
#include "ArloDriver.h"
#include "NeuroControllerDriver.h"

#define NUM_RAYS 16
// #define NUM_RAYS 32
#define NUM_SONARS 3
#define NUM_ACTUATORS 2 // Velocity linear and angular

using namespace std;

class SimulationController
{
public:
	SimulationController(double maxSTime = 300, int tRate = 1);
	virtual ~SimulationController();
	void setDriver(ArloDriver *driver);
	SimulationState startSimulation(ArloDriver *driver, int maxtime, double t_threshold);
	void checkSonarLeftValues(const sensor_msgs::LaserScan::ConstPtr &msg);
	void checkSonarCenterValues(const sensor_msgs::LaserScan::ConstPtr &msg);
	void checkSonarRightValues(const sensor_msgs::LaserScan::ConstPtr &msg);
	void checkModelPosition(const nav_msgs::Odometry::ConstPtr &msg);
	void checkSimulationTime(const rosgraph_msgs::Clock::ConstPtr &msg);
	int getNumSensors();
	int getNumActuators();
	int getNumInputs();
	void computeSimStats();
	bool showMustGoOn();
	void control_sound(string name);
	void simulation_color(string ctl_color);

	bool
	control_robot(arlo_controller_dmaking::control_robot::Request &req,
				  arlo_controller_dmaking::control_robot::Response &res);

	bool evaluateDriver(arlo_controller_dmaking::EvaluateDriver::Request &req,
						arlo_controller_dmaking::EvaluateDriver::Response &res);

private:
	double dist2Go(double x, double y);
	void printR_C(int ren, int col, string text, int color);

	double distance(double x1, double y1, double x2, double y2);
	SimulationState arloState;
	double prev_x, prev_y;
	double deltaX, deltaY; // Difference between target and robot.
	double deltaAngle;	   // Difference between the target direction and the robot direction.
	long int stuckCounter;
	bool stuck;
	string inputFile;
	string outputFile;
	double maxSimTime;	   /* Maximum time allowed for the robot to get the goal */
	double touchThreshold; // Distance to consider the palma has touch the coke.
	double goalDistance;
	double tauInicial;
	double alphaValue;
	double newTau;
	double closestHitDistance;
	bool is_driving;
	bool stop_driving;

	ArloDriver *aDriver;
	int ticsRate;	 /* How often the driver is ask for a decision */
	double linear_;	 /* Linear velocity to send to the robot */
	double angular_; /* Angular velocity to send to the robot */
	double l_scale_; /* Factor to the scale the linear velocity value */
	double a_scale_; /* Factor to the scale the angular velocity value */
	ros::NodeHandle nh_;
	ros::Publisher vel_pub_;
	ros::Subscriber odom_sub_;
	ros::Subscriber clock_sub_;
	ros::Subscriber sonar_l_sub_;
	ros::Subscriber sonar_c_sub_;
	ros::Subscriber sonar_r_sub_;
	ros::ServiceServer service;
	ros::ServiceServer srvControl_robot;
	ros::ServiceClient play_song;
	ros::ServiceClient change_color;

	vector<double> actuatorValues;
	vector<double> sensorValues;
	// double currentTime;
	// bool hasTimeRunOut;
	// bool finishLineCrossed;
};

#endif /* SRC_SIMULATIONCONTROLLER_H_ */
