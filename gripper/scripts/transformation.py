#!/usr/bin/env python
import rospy
from tf.transformations import quaternion_matrix, quaternion_from_matrix
import numpy as np
from geometry_msgs.msg import Pose

def object_world_transformation(gripper_pose, object_pose):
    pos = []
    # Isolating the quaternion angles alone into a numpy array.
    gripper_quaternion = np.array([gripper_pose.orientation.x, gripper_pose.orientation.y, gripper_pose.orientation.z, gripper_pose.orientation.w])
    object_quaternion = np.array([object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z, object_pose.orientation.w])

    # Isolating the positions into a numpy array.
    gripper_position = np.array([gripper_pose.position.x, gripper_pose.position.y, gripper_pose.position.z])
    object_position = np.array([object_pose.position.x, object_pose.position.y, object_pose.position.z])

    # Obtain the 4x4 homogeneous rotation matrix without the translation information.
    gripper_rotation_matrix = quaternion_matrix(gripper_quaternion)
    object_rotation_matrix = quaternion_matrix(object_quaternion)

    # Adding the translation information to the homogeneous matrix.
    for i in range(3):
        gripper_rotation_matrix[i,3] = gripper_position[i]
        object_rotation_matrix[i,3] = object_position[i]
    
    # Matrix multiplying to get the final transformation matrix of the gripper from the world frame.
    final_transformation = np.matmul(gripper_rotation_matrix, object_rotation_matrix)
    # print(final_transformation)

    # Finding the quaternions from the homogeneous matrix
    quat = quaternion_from_matrix(final_transformation)

    # Getting the translation information from the last column of the homogeneous matrix
    for i in range(3):
        pos.append(final_transformation[i][3])

    # Adding all the determined elements into the final_gripper_pose variable
    final_gripper_pose = Pose()
    final_gripper_pose.position.x = pos[0]
    final_gripper_pose.position.y = pos[1]
    final_gripper_pose.position.z = pos[2]
    final_gripper_pose.orientation.x = quat[0]
    final_gripper_pose.orientation.y = quat[1]
    final_gripper_pose.orientation.z = quat[2]
    final_gripper_pose.orientation.w = quat[3]

    return final_gripper_pose

if __name__ == "__main__":

    gripper = Pose()
    object = Pose()

    gripper.position.x = 0.5
    gripper.position.y = 0
    gripper.position.z = 0
    gripper.orientation.x = 0
    gripper.orientation.y = 0
    gripper.orientation.z = 0
    gripper.orientation.w = 1

    object.position.x = 0.5
    object.position.y = 0.5
    object.position.z = 0
    object.orientation.x = 0
    object.orientation.y = 0
    object.orientation.z = 0
    object.orientation.w = 1

    world_gripper_pose = object_world_transformation(gripper, object)
    print(world_gripper_pose)




