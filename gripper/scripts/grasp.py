#!/usr/bin/python

import rospy
from graspit_commander import GraspitCommander
import numpy as np
from geometry_msgs.msg import Pose
from transformation import object_world_transformation

epsilon_quality = []
pose = Pose()

# Pose at which the graspable body will be imported onto to the GraspIt! scene.
pose.position.x = 0.0454457
pose.position.y = 0.0111662
pose.position.z = 0.107
pose.orientation.x = 0
pose.orientation.y = 0
pose.orientation.z = 0
pose.orientation.w = 1

def grasp():
	# Set up GraspIt! scene and planner by importing the gripper and the graspable body to a desired pose.
	# TODO: Combine table and gripper import locations as a single world file.
	GraspitCommander.clearWorld()
	GraspitCommander.importRobot("RobotIQ")
	GraspitCommander.importGraspableBody("/home/asl-7/graspit/models/objects/cylinder.xml", pose)
	rospy.sleep(0.1)

	# Plan grasps
	planned_grasps = GraspitCommander.planGrasps()

	# Arranging the planned grasp qualities in the order of epsilon quality.
	for grasp in planned_grasps.grasps:
		epsilon_quality.append(grasp.epsilon_quality)

	# Determining the argument of the best candidate grasp.
	arg_max = np.argmax(np.array(epsilon_quality))

	# The variable best_pose contains information of the best grasp like the pose of the gripper, approach direction and dofs of the joints.
	best_pose = planned_grasps.grasps[arg_max]
	transformed_pose = object_world_transformation(best_pose.pose, pose)

	rospy.loginfo(transformed_pose)
	rospy.loginfo(best_pose.dofs)	
  

if __name__ == "__main__":	
	try:
		grasp()
	except rospy.ROSInterruptException:
		pass
