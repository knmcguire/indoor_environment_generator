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
#include "std_msgs/Float32.h"
#include <tf/transform_datatypes.h>
#include <sstream>

#include "random_environment_generator.h"


using namespace std;

RandomEnvironmentGenerator randomEnvironmentGenerator;
const int num_bots = 8;


float pos_bot_x[num_bots];
float pos_bot_y[num_bots];
float pos_bot_heading[num_bots];

bool random_environment_available = false;

#define SIMULATOR_IS_GAZEBO false


void poseUAV1CallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);


	pos_bot_heading[0]=(float)yaw;
	pos_bot_x[0] = msg->pose.position.x;
	pos_bot_y[0] = msg->pose.position.y;

}

void poseUAV2CallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	pos_bot_heading[1]=(float)yaw;
	pos_bot_x[1] =msg->pose.position.x;
	pos_bot_y[1] =msg->pose.position.y;
}

void poseUAV3CallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	pos_bot_heading[2]=(float)yaw;
	pos_bot_x[2] = msg->pose.position.x;
	pos_bot_y[2] =msg->pose.position.y;
}

void poseUAV4CallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	pos_bot_heading[2]=(float)yaw;
	pos_bot_x[3] = msg->pose.position.x;
	pos_bot_y[3] =msg->pose.position.y;
}

void poseUAV5CallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	pos_bot_heading[2]=(float)yaw;
	pos_bot_x[4] = msg->pose.position.x;
	pos_bot_y[4] =msg->pose.position.y;
}

void poseUAV6CallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	pos_bot_heading[2]=(float)yaw;
	pos_bot_x[5] = msg->pose.position.x;
	pos_bot_y[5] =msg->pose.position.y;
}

void poseUAV7CallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	pos_bot_heading[2]=(float)yaw;
	pos_bot_x[6] = msg->pose.position.x;
	pos_bot_y[6] =msg->pose.position.y;
}

void poseUAV8CallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	pos_bot_heading[2]=(float)yaw;
	pos_bot_x[7] = msg->pose.position.x;
	pos_bot_y[7] =msg->pose.position.y;
}

void poseUAV9CallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	pos_bot_heading[2]=(float)yaw;
	pos_bot_x[8] = msg->pose.position.x;
	pos_bot_y[8] =msg->pose.position.y;
}

void getRSSITowerCallback(std_srvs::Trigger::Request  &req,
		std_srvs::Trigger::Response &res)
{

}


bool indoorGenCallback(std_srvs::Trigger::Request  &req,
		std_srvs::Trigger::Response &res)
{
	// Get position bot from gazebo


// place holder
	/*float pos_bot_x_temp[2] = {-3, 6};
	float pos_bot_y_temp[2] = {-6,-3};*/


	float pos_bot_x_temp[num_bots] = {-8, -8,8,8,0,0,0,0};
	float pos_bot_y_temp[num_bots] = {-8,8,-8,8,0,0,0,0};

  //  cout<<pos_bot_x[0]<<" "<<pos_bot_y[0]<<endl;
  //  cout<<pos_bot_x[1]<<" "<<pos_bot_y[1]<<endl;

	float pos_tower[2] = {0,0};


	  //randomEnvironmentGenerator.Init(20,20,pos_bot_x,pos_bot_y,num_bots,pos_tower);
	  randomEnvironmentGenerator.Init(20,20,pos_bot_x_temp,pos_bot_y_temp,num_bots,pos_tower);

	  randomEnvironmentGenerator.generateEnvironment();
	 // randomEnvironmentGenerator.Reset();
	  random_environment_available = true;

	  res.message = "jeej";
	  res.success = true;

	  return true;
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "listener");
	ros::NodeHandle n;


	//ros::Subscriber sub = n.subscribe("/UAV1/ground_truth_to_tf/pose", 1000, poseUAV1CallBack);


	ros::Subscriber sub1,sub2,sub3,sub4,sub5,sub6,sub7,sub8;
  //subscribe for position
	for(int it = 1;it<num_bots+1;it++)
	{
#if SIMULATOR_IS_GAZEBO == true
		std::string topic_name_pos = "/UAV" + std::to_string(it)+"/ground_truth_to_tf/pose";
#else
		std::string topic_name_pos = "/bot" + std::to_string(it)+"/position";
#endif
		cout<<"topic names "<<topic_name_pos<<endl;
		//topic_name_pos_y = 'UAV' + std::to_string(it)+'ground_truth_to_tf/pose';
		//const geometry_msgs::PoseStamped::ConstPtr& msg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(topic_name_pos_x);
		//ros::Subscriber sub = n.subscribe("/UAV1/ground_truth_to_tf/pose", 1000, poseUAV1CallBack);

		if(it==1)
			sub1 = n.subscribe(topic_name_pos, 1000, poseUAV1CallBack);
		else if(it==2)
			sub2 = n.subscribe(topic_name_pos, 1000, poseUAV2CallBack);
		else if (it==3)
			sub3 = n.subscribe(topic_name_pos, 1000, poseUAV3CallBack);
		else if (it==4)
			sub4 = n.subscribe(topic_name_pos, 1000, poseUAV4CallBack);
		else if (it==5)
			sub5 = n.subscribe(topic_name_pos, 1000, poseUAV5CallBack);
		else if (it==6)
			sub6 = n.subscribe(topic_name_pos, 1000, poseUAV6CallBack);
		else if (it==7)
			sub7 = n.subscribe(topic_name_pos, 1000, poseUAV7CallBack);
		else if (it==8)
			sub8 = n.subscribe(topic_name_pos, 1000, poseUAV8CallBack);

		//cout<<"robot pos cb "<<it<<" "<<pos_bot_x[it-1]<<"  "<<pos_bot_y[it-1]<<endl;

	}





 // ros::Subscriber sub = n.subscribe("indoor_gen", 1000, indoorGenCallback);
  ros::ServiceServer service = n.advertiseService("indoor_gen", indoorGenCallback);
 // ros::ServiceServer service2 = n.advertiseService("get_rssi_to_tower", getRSSITowerCallback);


  ros::Publisher rssi_tower_array[num_bots];
	for(int it = 1;it<num_bots+1;it++)
	{
		std::string topic;
#if SIMULATOR_IS_GAZEBO == true
		topic = "/UAV" + std::to_string(it)+"/RSSI_to_tower";
#else
		topic = "/bot" + std::to_string(it)+"/RSSI_to_tower";
#endif
		rssi_tower_array[it-1]= n.advertise<std_msgs::Float32>(topic,1000);
	}

  ros::Rate loop_rate(200);
  while(ros::ok())
  {
	//  std_msgs::Float msg;

		for(int it = 1;it<num_bots+1;it++)
		{

			std_msgs::Float32 msg;
			//if(random_environment_available)
			msg.data = randomEnvironmentGenerator.getRSSITower(pos_bot_x[it-1],pos_bot_y[it-1],pos_bot_heading[it-1]);
			rssi_tower_array[it-1].publish(msg);

		}



  ros::spinOnce();
  loop_rate.sleep();
  }


  return 0;
}
