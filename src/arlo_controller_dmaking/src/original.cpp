#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <stdio.h>
#include <iostream>

#define AVANZAR 1
#define GIRAR 2

bool maxTime = false;
bool finishLineCrossed = false;

/**
 * Callback function executes when new topic data comes.
 * Task of the callback function is to print data to screen.
 */
void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("Seq: [%d]", msg->header.seq);
  ROS_INFO("Position-> x: [%f]", msg->pose.pose.position.x);
//   ,
//            msg->pose.pose.position.y, 
//            msg->pose.pose.position.z);
//   ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", 
//            msg->pose.pose.orientation.x, 
//            msg->pose.pose.orientation.y, 
//            msg->pose.pose.orientation.z, 
//            msg->pose.pose.orientation.w);
  ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", 
           msg->twist.twist.linear.x,
           msg->twist.twist.angular.z);
}
  
void clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg)
{
   double elapseTime = msg->clock.toSec();
   std::cout << "Tiempo simulacion1: " <<  msg->clock << std::endl;
   
   if (elapseTime >= 10) {
      maxTime = true;
   }
}
   
class SimpleNN
{
public:
   SimpleNN();
   void drive();
   
private:
   ros::NodeHandle nh_;
   double linear_, angular_, l_scale_, a_scale_;
   ros::Publisher vel_pub_;
   ros::Subscriber odom_sub_;
   ros::Subscriber clock_sub_;
};

SimpleNN::SimpleNN():
   linear_(0),
   angular_(0),
   l_scale_(0.25),
   a_scale_(1.0)
{
   nh_.param("scale_angular", a_scale_, a_scale_);
   nh_.param("scale_linear", l_scale_, l_scale_);
   
   vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
   odom_sub_ = nh_.subscribe("odom", 1000, chatterCallback);
   clock_sub_ = nh_.subscribe("clock", 1000, clockCallback);
}

void SimpleNN::drive()
{
   puts("Sending values for arlo actuators to go and return.");
   puts("---------------------------");
   
   ros::Rate loop_rate(1);
   
   int counter = 0;
   linear_ = angular_ = 0;
   
   int state = AVANZAR;
   while (ros::ok() && !maxTime)
   {
      switch (state) {
         case AVANZAR:
            puts("Avanzando...");
            linear_ = 1.0;
            angular_ = 0;
            counter++;
            if (counter > 5) {
               counter = 0;
               linear_ = angular_ = 0;
               state = GIRAR;
            }               
            break;
            
         case GIRAR:
            puts("Girando...");
            linear_ = 0.0;
            angular_ = 0.39;
            counter++;
            if (counter > 4) {
               counter = 0;
               linear_ = angular_ = 0;
               state = AVANZAR;
            }
            break;
      }
      
      geometry_msgs::Twist twist;
      twist.angular.z = a_scale_*angular_;
      twist.linear.x = l_scale_*linear_;
      vel_pub_.publish(twist);    
      
      ros::spinOnce();
      loop_rate.sleep();    
   }
   
   std::cout << "Bye" << std::endl;
   
   return;
}


int main(int argc, char** argv)
{
   ros::init(argc, argv, "neurocontroller_node");
   SimpleNN nnc;
      
   nnc.drive();
   
   return(0);
}


