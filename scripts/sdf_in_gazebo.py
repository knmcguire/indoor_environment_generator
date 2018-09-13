#!/usr/bin/python
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
import tf


rospy.init_node('sdf_in_gazebo',log_level=rospy.INFO)

initial_pose = Pose()
initial_pose.position.x = 0
initial_pose.position.y = 0
initial_pose.position.z = 0
quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
initial_pose.orientation.x = quaternion[0]
initial_pose.orientation.y = quaternion[1]
initial_pose.orientation.z = quaternion[2]
initial_pose.orientation.w = quaternion[3]

f = open('/home/knmcguire/Software/catkin_ws/src/indoor_environment_generator/models/random_generated_environment/test_model.sdf','r')
sdff = f.read()

rospy.wait_for_service('gazebo/spawn_sdf_model')
spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
spawn_model_prox("some_robo_name", sdff, "robotos_name_space", initial_pose, "world")