/*
 * SimulationController.cpp
 *
 *  Created on: Apr 14, 2020
 *      Author: antonio
 */

#include "SimulationController.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

SimulationController::SimulationController(double maxSTime, int tRate) : linear_{0},
                                                                         angular_{0},
                                                                         l_scale_{1.0},
                                                                         a_scale_{1.0},
                                                                         tauInicial{0.5},
                                                                         alphaValue{0.4},
                                                                         maxSimTime{maxSTime},
                                                                         ticsRate{tRate},
                                                                         actuatorValues{NUM_ACTUATORS, 0.0}
{
   /* +2 para dar la distancia a la meta como entrada. */
   sensorValues.resize(NUM_RAYS * NUM_SONARS + 2);

   prev_x = 0;
   prev_y = 0;
   deltaX = 100;
   deltaY = 100;
   deltaAngle = 3.1459;
   stuck = false;
   stuckCounter = 0;
   closestHitDistance = 100;
   nh_.param("scale_angular", a_scale_, a_scale_);
   nh_.param("scale_linear", l_scale_, l_scale_);
   is_driving = false;

   // inicilización
   stop_driving = false;

   vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
   odom_sub_ = nh_.subscribe("odom", 5, &SimulationController::checkModelPosition, this);
   clock_sub_ = nh_.subscribe("clock", 1, &SimulationController::checkSimulationTime, this);
   sonar_l_sub_ = nh_.subscribe("arlo/laser/scan_left", 100, &SimulationController::checkSonarLeftValues, this);
   sonar_c_sub_ = nh_.subscribe("arlo/laser/scan_center", 100, &SimulationController::checkSonarCenterValues, this);
   sonar_r_sub_ = nh_.subscribe("arlo/laser/scan_right", 100, &SimulationController::checkSonarRightValues, this);
   service = nh_.advertiseService("evaluate_driver", &SimulationController::evaluateDriver, this);
   srvControl_robot = nh_.advertiseService("control_robot", &SimulationController::control_robot, this);
}

SimulationController::~SimulationController() {}

void SimulationController::control_sound(string name)
{
   ros::NodeHandle n;
   play_song = n.serviceClient<arlo_controller_dmaking::play_song>("play_song");

   arlo_controller_dmaking::play_song param;
   param.request.name = name;

   if (play_song.call(param))
   {
      if (name.find("motor_play") == std::string::npos)
         cout << "play_song " << name << endl;
   }
   else
   {
      cerr << "Error al reproducir el sonido " << name << endl;
   }
}

void SimulationController::simulation_color(string ctl_color)
{
   ros::NodeHandle n;
   change_color = n.serviceClient<user_interaction::change_color>("change_color");

   user_interaction::change_color param;
   param.request.color = ctl_color;

   if (change_color.call(param))
   {
      cout << "change_color " << ctl_color << endl;
   }
   else
   {
      cerr << "Error al cambiar el color de la ventana de estimulacion" << ctl_color << endl;
   }
}

void SimulationController::setDriver(ArloDriver *driver)
{
   aDriver = driver;
}

bool SimulationController::control_robot(arlo_controller_dmaking::control_robot::Request &req,
                                         arlo_controller_dmaking::control_robot::Response &res)
{
   string control = req.control;

   res.is_driving = is_driving;
   res.max_time = maxSimTime;

   if (control.find("stop") != std::string::npos)
   {
      cout << "Deteniendo robot" << endl;
      stop_driving = true;
      // arloState.hasTimeRunOut = true;
   }

   if (control.find("pause") != std::string::npos)
   {
      cout << "Pausando robot" << endl;
      // se detiene la simulación
      std_srvs::Empty gazeboParams;
      ros::service::call("/gazebo/pause_physics", gazeboParams);
      cout << "pause gazebo" << endl;
      // se pausa el sonido (incluyendo el de fondo)
      control_sound("pause_world");
      cout << "pause sound" << endl;
   }

   if (control.find("unpause") != std::string::npos)
   {
      cout << "Conduciedo el robot" << endl;
      // se continua la simulación
      std_srvs::Empty gazeboParams;
      ros::service::call("/gazebo/unpause_physics", gazeboParams);

      // se reproduce el sonido (incluyendo el de fondo)
      if (is_driving)
      {
         control_sound("unpause_world");
      }
      else
      {
         control_sound("unpause_world_min_volume");
      }
   }

   return true;
}

// void SimulationController::drive()
// {
// }

bool SimulationController::evaluateDriver(arlo_controller_dmaking::EvaluateDriver::Request &req,
                                          arlo_controller_dmaking::EvaluateDriver::Response &res)
{
   // TODO: Revisar que Gazebo este corriendo
   string pesos(req.weightsfile);
   cout << "pesos: " << req.weightsfile << endl;
   aDriver->setParameters(pesos.c_str(), req.num_ind);

   startSimulation(aDriver, req.maxtime, req.touchthreshold);

   res.time = arloState.distanceTravelled; // arloState.finishTime;
   res.dist2go = arloState.distanceToGo;
   res.damage = arloState.robotDamage;
   res.energy = arloState.distanceTravelled; // Energía usada.
   res.risk = arloState.crashingRisk;
   res.contactSpeed = fabs(arloState.finalSpeed);

   return true;
}

SimulationState SimulationController::startSimulation(ArloDriver *aDriver, int maxtime, double t_threshold)
{
   // string pesos(req.weightsfile);
   // aDriver->setParameters(pesos.c_str());

   puts("\nStarting the simulation of a new driver...");
   cerr << "---------------------------" << endl;

   std_srvs::Empty gazeboParams;
   ros::service::call("/gazebo/reset_simulation", gazeboParams);
   // inicia la musica del ambiente (incluye motor)
   control_sound("unpause_world");
   // se quita el color de la ventana de efectos
   simulation_color("hide");
   ros::Duration(0.1).sleep(); // sleep for half a second

   /* Maximum simulation time allowed */
   maxSimTime = maxtime;
   touchThreshold = t_threshold;

   ros::Rate loop_rate(40); // Frecuencia Hz con la que le robot debe tomar una decisión.

   stop_driving = false;
   linear_ = angular_ = 0;

   arloState.resetState();
   stuckCounter = 0;
   closestHitDistance = 100;
   newTau = tauInicial;

   int iter = 0;
   while (ros::ok() && showMustGoOn() && !stop_driving)
   {
      is_driving = true;
      // Send sensor values to the driver and get its answer to move the robot.
      aDriver->driveArlo(sensorValues, actuatorValues);

      if (iter++ % 10 == 0)
      {
         // sonido de los servomotores
         control_sound("motor_play");
         // string distMsg = fmt::format("D,A=[{:6.2f}, {:6.2f}] ", sensorValues[3*NUM_RAYS], sensorValues[3*NUM_RAYS+1]);
         // // printR_C(8, 50, distMsg, 32);
         // cout << distMsg << endl;
         // string outMsg = fmt::format("L,A=[{:6.2f}, {:6.2f}] ", actuatorValues[0], actuatorValues[1]);
         // // printR_C(9, 50, outMsg, 32);
         // cout << outMsg << endl;
      }
      //     << ", angular= " << actuatorValues[1] << "\n" << endl;

      linear_ = actuatorValues[0];
      angular_ = actuatorValues[1];

      // Set values in the twist object to send the actuator values to the robot in Gazebo.
      geometry_msgs::Twist twist;
      twist.angular.z = a_scale_ * angular_;
      twist.linear.x = l_scale_ * linear_;
      vel_pub_.publish(twist); // Publish the event for the twist plugin.

      ros::spinOnce();
      loop_rate.sleep();
   }

   cerr << "\033[2J";
   is_driving = false;
   stop_driving = false;

   // // se detiene el robot
   // geometry_msgs::Twist twist;
   // twist.angular.z = 0;
   // twist.linear.x = 0;
   // vel_pub_.publish(twist);
   // ros::spinOnce();

   // estados finales
   computeSimStats();

   ros::Duration(1.).sleep(); // sleep for half a second

   // se reinicia la simulación
   ros::service::call("/gazebo/reset_simulation", gazeboParams);

   // solo se deteiene el sonido del motor, los efectos de sonido se mantienen
   // control_sound("motor_stop");

   // se deteienen los efectos de sonido (incluido el motor)
   control_sound("stop_effects");
   // se ocultan los afectos
   simulation_color("hide");

   return arloState;
}

bool SimulationController::showMustGoOn()
{
   return (arloState.hasTimeRunOut == false &&
           arloState.targetReached == false);
}

void SimulationController::computeSimStats()
{

   arloState.finishTime = arloState.currentTime;
   arloState.crashingRisk = 1.0 / (closestHitDistance + 1);

   cerr << "hasTimeRunOut= " << arloState.hasTimeRunOut << " -> tiempo:" << arloState.currentTime << ", max= " << maxSimTime << "\n";
   cerr << "targetReached= " << arloState.targetReached << "\n";

   if (arloState.targetReached == true)
   {
      fmt::print(stderr, fg(fmt::color::green), "\nArlo has NAILED it :)\n");
      cerr << "finishTime = " << arloState.finishTime << "\n";
      cerr << "minDist2Go = " << arloState.distanceToGo << " <= ";
      cerr << touchThreshold << " (threshold)"
           << "\n";
      arloState.robotEnergy = 100;

      // se muestra el efecto
      simulation_color("green");
      // efecto de aplausos
      control_sound("sucess");
   }
   else if (arloState.hasTimeRunOut == true)
   {
      fmt::print(stderr, fg(fmt::color::red), "\nArlo has failed :(\n");
      cerr << "minDistance2Go = " << arloState.distanceToGo << "\n";
      cerr << "currentTime    = " << arloState.currentTime << "\n";

      // se muestra el efecto
      simulation_color("red");
      // efecto de abucheo
      control_sound("loss");
   }

   if (arloState.stuck == true)
   {
      fmt::print(stderr, fg(fmt::color::red), "\n---->>> ATASCADO <<<-----\n");
      cerr << " \t Counter = " << stuckCounter << endl;

      // se muestra el efecto
      simulation_color("red");
      // efecto de abucheo
      control_sound("loss");
   }

   cerr << "x = " << arloState.position[0] << ", y = " << arloState.position[1] << endl;
   cerr << "Contact SPEED   = " << arloState.finalSpeed << endl;
   cerr << "Crash Risk      = " << arloState.crashingRisk << endl;
   // cerr << "d2Go            = " << arloState.distanceToGo << endl;
   // err << "Dist. travelled = " << arloState.distanceTravelled << endl;

   //	res.time = arloState.finishTime;
   //   res.dist2go = arloState.distanceToGo;
   //	res.damage = arloState.robotDamage ;
   //	res.energy = arloState.distanceTravelled;
}

/*
 * Next three methods update sensor values periodically. That is, each time
 * the Platform node publishes new sensor values, these method are triggered
 * to store sensor values on the vector sensorValues.
 */

/* Method that update values from the Left Sensor. */
void SimulationController::checkSonarLeftValues(const sensor_msgs::LaserScan::ConstPtr &msg)
{
   // ROS_INFO("Tamano ranges left= %lu", msg->ranges.size());

   // cerr << "sensor= " << msg->ranges[0] << endl;
   for (int i = 0; i < msg->ranges.size(); ++i)
   {
      sensorValues[i + 0 * NUM_RAYS] = (msg->ranges[i] <= 3.0) ? (3.0 - msg->ranges[i]) : 0.0; // 0 para el sensor izq.
      if (msg->ranges[i] < closestHitDistance && arloState.distanceToGo > 1)
         closestHitDistance = msg->ranges[i];
   }
   // cerr << "sensor= " << sensorValues[0] << endl;
}

/* Method that update values from the Center Sensor. */
void SimulationController::checkSonarCenterValues(const sensor_msgs::LaserScan::ConstPtr &msg)
{
   // ROS_INFO("Tamano ranges center= %lu", msg->ranges.size());

   for (int i = 0; i < msg->ranges.size(); ++i)
   {
      sensorValues[i + 1 * NUM_RAYS] = (msg->ranges[i] <= 3.0) ? (3.0 - msg->ranges[i]) : 0.0; // 1 para el sensor central.
      if (msg->ranges[i] < closestHitDistance && arloState.distanceToGo > 1)
         closestHitDistance = msg->ranges[i];
   }
}

/* Method that update values from the Right Sensor. */
void SimulationController::checkSonarRightValues(const sensor_msgs::LaserScan::ConstPtr &msg)
{
   // ROS_INFO("Tamano ranges right= %lu", msg->ranges.size());

   for (int i = 0; i < msg->ranges.size(); ++i)
   {
      sensorValues[i + 2 * NUM_RAYS] = (msg->ranges[i] <= 3.0) ? (3.0 - msg->ranges[i]) : 0.0; // 2 para el sensor der.
      if (msg->ranges[i] < closestHitDistance && arloState.distanceToGo > 1)
         closestHitDistance = msg->ranges[i];
   }

   // sensorValues[3*NUM_RAYS+0] = deltaX;
   // sensorValues[3*NUM_RAYS+1] = deltaY;
   sensorValues[3 * NUM_RAYS + 0] = arloState.distanceToGo - 1.4;
   sensorValues[3 * NUM_RAYS + 1] = deltaAngle;
}

/* This method periodically updates the current position of the platform */
void SimulationController::checkModelPosition(const nav_msgs::Odometry::ConstPtr &msg)
{
   if (arloState.targetReached == true)
      return;

   // ROS_INFO("Seq: [%d]", msg->header.seq);
   // ROS_INFO("Position-> x: [%f]", msg->pose.pose.position.x);
   //  ROS_INFO("Vel-> Linear: [%f], Angular: [%f]",
   //  		msg->twist.twist.linear.x,
   //    	   msg->twist.twist.angular.z);
   //  arloState.currentPosition = msg->pose.pose.position.x;
   //  if (arloState.currentPosition >= goalDistance)   // Esta es la distancia del pasillo en Gazebo.
   //  	arloState.finishLineCrossed = true;

   tf2::Quaternion quat_dir;

   quat_dir.setX(msg->pose.pose.orientation.x);
   quat_dir.setY(msg->pose.pose.orientation.y);
   quat_dir.setZ(msg->pose.pose.orientation.z);
   quat_dir.setW(msg->pose.pose.orientation.w);

   double roll, pitch, yaw;
   tf2::Matrix3x3(quat_dir).getRPY(roll, pitch, yaw);

   // cout << "roboto dir.= " << yaw*180 / M_PI << endl;

   // msg->pose.pose.orientation.
   /* Calcular la velocidad promedio usando el promedio exponencial (darle más peso a valores recientes de velocidad) */
   newTau = alphaValue * msg->twist.twist.linear.x + (1 - alphaValue) * newTau;
   arloState.finalSpeed = newTau;
   // cout << "\nvelocidad = " << newTau << endl;

   /* Obtener la posición actual de robot. */
   double new_x = msg->pose.pose.position.x;
   double new_y = msg->pose.pose.position.y;

   // Compute differente between target and robot in each dimension (kinda distance)
   deltaX = 0.0 - new_x;
   deltaY = 0.0 - new_y;
   // Angle in format (0, PI), (0,-PI)
   double targetDirection = atan(fabs(deltaY / deltaX));
   if (deltaX < 0 && deltaY > 0)
   { // 2o cuadrante -,+
      targetDirection = M_PI - targetDirection;
      // cerr << "\n2o: ";
   }
   else if (deltaX < 0 && deltaY < 0)
   { // 3er cuadrante -,-
      targetDirection = -M_PI + targetDirection;
      // cerr << "\n3er: ";
   }
   else if (deltaX > 0 && deltaY < 0)
   { // 4o cuadrante +,-
      targetDirection = -targetDirection;
      // cerr << "\n4o: ";
   }

   deltaAngle = atan2(sin(targetDirection - yaw), cos(targetDirection - yaw));
   // cout << "targetAng= " << targetDirection*180 / M_PI << endl;
   // cout << "deltaAngle= " << deltaAngle*180 / M_PI << endl;

   // arloState.currentPosition = msg->pose.pose.position.x;
   arloState.position[0] = new_x;
   arloState.position[1] = new_y;
   arloState.distanceToGo = dist2Go(new_x, new_y);

   // Como los mensajes de la posición actual están en una cola, es posible que al comenzar
   // una nueva simulación, en la cola se hayan quedado valores de una posición cercana a
   // la meta. De esta manera, es posible que al inicio de una simulación, de manera incorrecta
   // se detecte que el robot llegó a la meta.
   // Cuando esto pasa, el tiempo es casi cero, así que se puede usar esto para ignorar algún
   // posible falso positivo. Si el tiempo es menor que 2 y se alcanzó el objeto, quiere decir que
   // se está leyendo un valor de la cola y no la posición actual.
   // cout << "touchThreshold " << touchThreshold << "   " << arloState.currentTime << endl;
   if (arloState.distanceToGo <= touchThreshold && arloState.currentTime > 1)
   {
      arloState.targetReached = true;
      cerr << "\n\nTARGET REACHED: " << arloState.distanceToGo << endl;
      return;
   }

   /* Determinar si el robot ya se quedó atorado. */
   /* Si la distancia que ha avanzado es menor que un umbral pequeño */
   double distanceBefore = arloState.distanceTravelled;

   // Compute the new distance travelled.
   arloState.distanceTravelled += distance(prev_x, prev_y, new_x, new_y);

   // Check if the robot has moved or not.
   if (abs(distanceBefore - arloState.distanceTravelled) < 0.001)
   {
      stuckCounter++;
      // cout << "Stuck counter: " << stuckCounter << "\n";
      if (stuckCounter > 280)
      {
         arloState.stuck = true;
         arloState.hasTimeRunOut = true;
      }
   }
   else
      stuckCounter = 0;

   // Swap values for next iteration.
   prev_x = new_x;
   prev_y = new_y;
}

/* Calcula la distancia de la plataforma (x,y) al punto meta. */
double SimulationController::dist2Go(double x, double y)
{
   /* En el escenario sencillo, suponemos que la meta está en el punto 0,0 */
   double xTarget = -2.013;
   double yTarget = 2.586;

   // cout << "x robot = " << x << ", y robot" << y << endl;
   double distToGo = distance(xTarget, yTarget, x, y);
   // cout << "distance" << distToGo << " x" << x << " y" << y << endl;
   return distToGo;
}

double SimulationController::distance(double x1, double y1, double x2, double y2)
{
   double sum = (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
   return sqrt(sum);
}

void SimulationController::checkSimulationTime(const rosgraph_msgs::Clock::ConstPtr &msg)
{
   arloState.currentTime = msg->clock.toSec();
   // cerr << "\nTiempo simulacion: " <<  msg->clock.toSec() << ", max= " << maxSimTime << endl;

   if (arloState.currentTime >= maxSimTime)
   {
      arloState.hasTimeRunOut = true;
      // cerr << "\nFIN: tiempo simulacion: " <<  arloState.currentTime << ", max= " << maxSimTime << endl;
   }
}

int SimulationController::getNumSensors()
{
   return NUM_RAYS * NUM_SONARS;
}

int SimulationController::getNumActuators()
{
   return NUM_ACTUATORS;
}

int SimulationController::getNumInputs()
{
   return getNumSensors() + 2; // Delta X y Delta Y
}

void SimulationController::printR_C(int ren, int col, string text, int color)
{
   cerr << "\033 7";
   cout << "\033[38;5;" << color << "m";
   cout << "\033[" << ren << ";" << col << "H";
   cout << text << endl;
   cerr << "\033 8" << endl;
}
