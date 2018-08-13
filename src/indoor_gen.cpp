/*
 * indoor_gen.c
 *
 *  Created on: Aug 13, 2018
 *      Author: knmcguire
 */


#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Trigger.h"

#include <sstream>

#include "random_environment_generator.h"

  RandomEnvironmentGenerator randomEnvironmentGenerator;

bool indoorGenCallback(std_srvs::Trigger::Request  &req,
		std_srvs::Trigger::Response &res)
{
	float pos_bot_x[1] = {-6};
	float pos_bot_y[1] = {-6};
	float pos_tower[2] = {6,6};
	  randomEnvironmentGenerator.Init(20,20,pos_bot_x,pos_bot_y,1,pos_tower);
	  randomEnvironmentGenerator.generateEnvironment();
	 // randomEnvironmentGenerator.Reset();
return true;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");


  ros::NodeHandle n;

 // ros::Subscriber sub = n.subscribe("indoor_gen", 1000, indoorGenCallback);
  ros::ServiceServer service = n.advertiseService("indoor_gen", indoorGenCallback);


  ros::spin();

  return 0;
}
