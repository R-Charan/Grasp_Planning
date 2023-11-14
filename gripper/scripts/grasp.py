#!/usr/bin/python

import rospy
from graspit_commander import GraspitCommander
import numpy as np
from geometry_msgs.msg import Pose
from transformation import object_world_transformation

epsilon_quality = []
object_pose = Pose()

# Pose at which the graspable body will be imported onto to the GraspIt! scene.
object_pose.position.x = 0.0
object_pose.position.y = 0.0
object_pose.position.z = 0
object_pose.orientation.x = 0
object_pose.orientation.y = 0
object_pose.orientation.z = 0
object_pose.orientation.w = 1


def grasp():

	# Clear Graspit World and import the world file that contains the table and the gripper
	GraspitCommander.clearWorld()
	# Combined import file of table and gripper.
	GraspitCommander.loadWorld("grasp")
	# GraspitCommander.importRobot("RobotIQ")
	
	GraspitCommander.importGraspableBody("/home/asl-7/Downloads/Part1(m).STL", object_pose)
	# GraspitCommander.importGraspableBody("cylinder", object_pose)

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
	# Get the transforms of the gripper wrt the world coordinates.
	transformed_pose = object_world_transformation(best_pose.pose, object_pose)

	rospy.loginfo(transformed_pose)
	rospy.loginfo(best_pose.dofs)	
  

if __name__ == "__main__":	
	try:
		grasp()
	except rospy.ROSInterruptException:
		pass
