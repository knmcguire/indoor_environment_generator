/*
 * indoor_gen.c
 *
 *  Created on: Aug 13, 2018
 *      Author: knmcguire
 */


#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

void indoorGenCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");


  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("indoor_gen", 1000, indoorGenCallback);


  ros::spin();

  return 0;
}
