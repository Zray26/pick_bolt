#!/usr/bin/env python
import rospy
import sys
import copy
import tf
from hrclib_client_v6 import odyssey_Interface
import geometry_msgs.msg 
import math
import moveit_commander
from movo_action_clients.gripper_action_client import GripperActionClient
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
import time
# import zl_ods_contact
pos={'h': (9.853695701167453e-06, -0.6027460694313049), 'lin': 0.4585753381252289, 'l': (-0.13635384781205717, 0.9114743399306715, 0.850791817648235, 1.2031456067289128, -0.3104654609270394, -1.3138586727237413, -0.8174240070861964), 'r': (0.2505741638044654, -0.9006784175037083, -0.4091691351442108, -1.670779516757796, 0.6212879185982039, 1.0429657882244618, -2.4777225202159308)}
_upper_body_joints = ["right_shoulder_pan_joint",
                      "right_shoulder_lift_joint",
                      "right_arm_half_joint",
                      "right_elbow_joint",
                      "right_wrist_spherical_1_joint",
                      "right_wrist_spherical_2_joint",
                      "right_wrist_3_joint",
                      "left_shoulder_pan_joint",
                      "left_shoulder_lift_joint",
                      "left_arm_half_joint",
                      "left_elbow_joint",
                      "left_wrist_spherical_1_joint",
                      "left_wrist_spherical_2_joint",
                      "left_wrist_3_joint",
                      "linear_joint",
                      "pan_joint",
                      "tilt_joint"]
                      
# Head looking straight
default_pose_tucked = [-1.595, -1.5, 0.1, -2.612, 0.0, 0.496, -1.69,
                       1.595, 1.5, -0.1, 2.612, 0.0, -0.496, 1.69,
                       0.14, 0, 0]

# class temp_pickbolt_client(object):
#     def __init__(self):
#         rospy.init_node('transform_subscriber', anonymous= True)
#         rospy.Subscriber("/marker",geometry_msgs.msg.Transform,self.marker_CallBack)
        
#         self.msg=None
#     def marker_CallBack(self,msg):
#         print(msg)
#         self.msg=msg


# class move_to_tuck(object):
#     def __init__(self):
#         _upper_body_joints = ["right_shoulder_pan_joint",
#                       "right_shoulder_lift_joint",
#                       "right_arm_half_joint",
#                       "right_elbow_joint",
#                       "right_wrist_spherical_1_joint",
#                       "right_wrist_spherical_2_joint",
#                       "right_wrist_3_joint",
#                       "left_shoulder_pan_joint",
#                       "left_shoulder_lift_joint",
#                       "left_arm_half_joint",
#                       "left_elbow_joint",
#                       "left_wrist_spherical_1_joint",
#                       "left_wrist_spherical_2_joint",
#                       "left_wrist_3_joint",
#                       "linear_joint",
#                       "pan_joint",
#                       "tilt_joint"]
                      
# # Head looking straight
#         default_pose_tucked = [-1.595, -1.5, 0.1, -2.612, 0.0, 0.496, -1.69,
#                             1.595, 1.5, -0.1, 2.612, 0.0, -0.496, 1.69,
#                             0.14, 0, 0]
#         lgripper = GripperActionClient('left')
#         rgripper = GripperActionClient('right')
#         gripper_closed = 0.00
#         gripper_open = 0.165
        
#         larm_group = moveit_commander.MoveGroupCommander("left_arm")
#         rarm_group = moveit_commander.MoveGroupCommander("right_arm")
#         upper_body = moveit_commander.MoveGroupCommander("upper_body")

#         move_group = MoveGroupInterface("upper_body", "base_link")
#         lmove_group = MoveGroupInterface("left_arm", "base_link")
#         rmove_group = MoveGroupInterface("right_arm", "base_link")
#         self._upper_body_joints=_upper_body_joints
#         self.default_pose_tucked=default_pose_tucked
#         self.move_group=move_group

#     def move_to_initial_pose(self):


#         print("Done spinning up MoveIt!")

        
#         while not rospy.is_shutdown():
#             print("doing L0_goto_upper_body_joints")
#             result = self.move_group.moveToJointPosition(self._upper_body_joints, self.default_pose_tucked, 0.005, wait=True)

#             print("error code: ", result.error_code.val)
#             if result.error_code.val == MoveItErrorCodes.SUCCESS:
#                 break

#         print("successful")

#     def __del__(self):
#         pass
#temp_pickbolt_client.marker_CallBack()

if __name__=="__main__":
    rospy.init_node("Pickbolt_CFRP",
                    anonymous=False)
    # moveit_commander.roscpp_initialize(sys.argv)
    # scene = moveit_commander.PlanningSceneInterface()
    # tuck = move_to_tuck()
    # tuck.move_to_initial_pose()
    # del tuck

    # upper_body.go(joints=[])
    # larm_group.
    
    ods = odyssey_Interface()
    ods._L0_upper_jp_move_safe(jpl=[1.595, 1.5, -0.4, 2.612, 0.0, -0.496, 1.69],jpr=[1.3, -1.45, 0.2, -1.7, 0.2, 1.15, -1.67],jplinear=0.14,
                               jph=[0, 0],wait=True,hard=False,
                               lforce=[30 for i in range(6)],
                               rforce=[30 for i in range(6)],
                               duration=8
                               )
    ods.grip("right",1)
    arc._L0_dual_task_move_safe_relate(
        rmove=[0.25, 0, 0], rmaxforce=[30 for i in range(6)],
        lmove=[0, 0, 0], lmaxforce=[30 for i in range(6)],
        time=4)
    arc._L0_dual_task_move_safe_relate(
        rmove=[0, 0, -0.15], rmaxforce=[30 for i in range(6)],
        lmove=[0, 0, 0], lmaxforce=[30 for i in range(6)],
        time=4)
    ods.grip("right",1)
    arc._L0_dual_task_move_safe_relate(
        rmove=[0, 0, 0.15], rmaxforce=[30 for i in range(6)],
        lmove=[0, 0, 0], lmaxforce=[30 for i in range(6)],
        time=4)
    arc._L0_dual_task_move_safe_relate(
        rmove=[-0.15, -0.1, 0], rmaxforce=[30 for i in range(6)],
        lmove=[0, 0, 0], lmaxforce=[30 for i in range(6)],
        time=4)
    
# self.default_pose_tucked = [-1.595, -1.5, 0.40, -2.612, 0.0, 0.496, -1.69,
#                                     1.595, 1.5, -0.4, 2.612, 0.0, -0.496, 1.69,
#   