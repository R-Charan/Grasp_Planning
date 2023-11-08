#!usr/bin/env python

import rospy
from tf.transformations import quaternion_matrix, quaternion_from_matrix
import numpy as np
from geometry_msgs.msg import Pose

def object_world_transformation(gripper_pose, object_pose):
    pos = []
    gripper_rotation_matrix = quaternion_matrix(gripper_pose.orientation)
    object_rotation_matrix = quaternion_matrix(object_pose.orientation)

    for i in range(3):
        gripper_rotation_matrix[i,3] = gripper_pose.position[i]
        object_rotation_matrix[i,3] = object_pose.position[i]
    
    final_transformation = np.matmul(gripper_rotation_matrix, object_rotation_matrix)
    quat = quaternion_from_matrix(final_transformation)
    for i in range(3):
        pos[i] = final_transformation[i,3]
    
    final_gripper_pose = Pose()
    final_gripper_pose.position.x = pos[0]
    final_gripper_pose.position.y = pos[1]
    final_gripper_pose.position.z = pos[2]
    final_gripper_pose.orientation.x = quat[0]
    final_gripper_pose.orientation.y = quat[1]
    final_gripper_pose.orientation.z = quat[2]
    final_gripper_pose.orientation.w = quat[3]

    rospy.loginfo(final_gripper_pose)

    return final_gripper_pose

if __name__ == "__main__":
    gripper = Pose()
    object = Pose()

    gripper.position.x = 0
    gripper.position.y = 0
    gripper.position.z = 0
    gripper.orientation.x = 0
    gripper.orientation.y = 0
    gripper.orientation.z = 0
    gripper.orientation.w = 1

    object.position.x = 0.5
    object.position.y = 0
    object.position.z = 0
    object.orientation.x = 0
    object.orientation.y = 0
    object.orientation.z = 0
    object.orientation.w = 1

    world_gripper_pose = object_world_transformation(gripper, object)
    print(world_gripper_pose)




