#!/usr/bin/python

import rospy
from graspit_commander import GraspitCommander
import numpy as np
from geometry_msgs.msg import Pose
import time

epsilon_quality = []
object_pose = Pose()
robot_pose = Pose()

# Pose at which the graspable body will be imported to the GraspIt! scene.
object_pose.position.x = 0.6
object_pose.position.y = 0
object_pose.position.z = 0.1
object_pose.orientation.x = 0 #0.7071068
object_pose.orientation.y = 0
object_pose.orientation.z = 0
object_pose.orientation.w = 1 #0.7071068

def grasp():

	# Clear Graspit World and import the world file that contains the table and the gripper
	GraspitCommander.clearWorld()

	# Combined import file of table and gripper.
	GraspitCommander.loadWorld("grasp")	

	#GraspitCommander.importGraspableBody("/home/asl-7/Downloads/Part1(m).STL", object_pose)
	GraspitCommander.importGraspableBody("/home/asl-7/graspit/models/objects/cube.xml", object_pose)

	rospy.sleep(0.1)

	# To measure time taken to plan for the grasp
	start_time =time.time()

	# Plan grasps
	planned_grasps = GraspitCommander.planGrasps()
	
	# Printing the time taken to plan the grasp
	print ("Planning time = ", time.time() - start_time)

	# Arranging the planned grasp qualities in the order of epsilon quality.
	for grasp in planned_grasps.grasps:
		epsilon_quality.append(grasp.epsilon_quality)

	# Determining the argument of the best candidate grasp.
	arg_max = np.argmax(np.array(epsilon_quality))
    
	# The variable best_pose contains information of the best grasp like the pose of the gripper, approach direction and dofs of the joints.
	best_pose = planned_grasps.grasps[arg_max]
  
	rospy.loginfo(best_pose)


if __name__ == "__main__":	
	try:
		grasp()
	except rospy.ROSInterruptException:
		pass
