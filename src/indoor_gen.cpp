/*
 * indoor_gen.c
 *
 *  Created on: Aug 13, 2018
 *      Author: knmcguire
 */


#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/PoseStamped.h"

#include <sstream>

#include "random_environment_generator.h"

using namespace std;

  RandomEnvironmentGenerator randomEnvironmentGenerator;

bool indoorGenCallback(std_srvs::Trigger::Request  &req,
		std_srvs::Trigger::Response &res)
{
	// Get position bot from gazebo
	int num_bots = 2;

	std::string topic_name_pos_x;
	std::string topic_name_pos_y;

	float pos_bot_x[num_bots];
	float pos_bot_y[num_bots];

	for(int it = 1;it<num_bots+1;it++)
	{
		topic_name_pos_x = "/UAV" + std::to_string(it)+"/ground_truth_to_tf/pose";
		//topic_name_pos_y = 'UAV' + std::to_string(it)+'ground_truth_to_tf/pose';
		const geometry_msgs::PoseStamped::ConstPtr& msg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(topic_name_pos_x);


		pos_bot_x[it-1] = roundf(msg->pose.position.x);
		pos_bot_y[it-1] =roundf(msg->pose.position.y);
		//cout<<"robot pos cb "<<it<<" "<<pos_bot_x[it-1]<<"  "<<pos_bot_y[it-1]<<endl;

	}




	//float pos_bot_x[1] = {-6};
	//float pos_bot_y[1] = {-6};
	float pos_tower[2] = {5,5};
	  randomEnvironmentGenerator.Init(20,20,pos_bot_x,pos_bot_y,num_bots,pos_tower);
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
