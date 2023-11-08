#! /usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Pose
from server.srv import pose_dof, pose_dofResponse
from graspit_commander import GraspitCommander

# Perform transformation of gripper pose from object frame to world frame with transformation.py file
from transformation import object_world_transformation

epsi = []

def callback(request):
    # rospy.loginfo(req.object_pose)
    if request.counter == 0:
        global planned_grasps
        # Import all the files in the world of graspit and perform the grasps
        GraspitCommander.clearWorld()
        # Spawn the object at the correct object_pose
        GraspitCommander.importGraspableBody("/home/asl-7/graspit/models/objects/cylinder.xml", request.obj_pose)

        GraspitCommander.importRobot("RobotIQ")
        rospy.sleep(1)
        planned_grasps = GraspitCommander.planGrasps()

        for grasp in planned_grasps.grasps:
            epsi.append(grasp.epsilon_quality)

        arg_max = np.argmax(np.array(epsi))
        best_pose = planned_grasps.grasps[arg_max].pose

        # The best_pose is in the objects frame of reference -> transforming the pose to world frame in the next step
        transformed_pose = object_world_transformation(best_pose, request.object_pose)
        rospy.loginfo(best_pose)
        dofs = planned_grasps.grasps[arg_max].dofs
        epsi[arg_max] = -10.0
        response = pose_dofResponse()
        response.gripper_pose = transformed_pose
        response.dofs = dofs
        return response
    
    # The counter variable value is 0 at the start. After the grasp is planned, the counter variable is changed. In the next iteration
    # All information regarding the grasp is already available, just finding the best grasp pose and passing it to the execution system.
    elif request.counter > 0:
        arg_max = np.argmax(np.array(epsi))
        best_pose = planned_grasps.grasps[arg_max].pose
        transformed_pose = object_world_transformation(best_pose, request.object_pose)
        rospy.loginfo(best_pose)
        dofs = planned_grasps.grasps[arg_max].dofs
        epsi[arg_max] = -10.0
        response = pose_dofResponse()
        response.gripper_pose = transformed_pose
        response.dofs = dofs
        return response

        

def main():
    rospy.init_node("gripper_pose_server")
    s = rospy.Service('gripper_pose', pose_dof, callback)
    print("server_running")
    rospy.spin()
    
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass