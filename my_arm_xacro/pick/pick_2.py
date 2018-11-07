#!/usr/bin/env python

import sys
import rospy
import copy, math

from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random

ROBOT_NAME = "test_robot"

if ROBOT_NAME == "test_robot":
    GROUP_NAME_ARM = 'arm'
    GROUP_NAME_GRIPPER = 'gripper'

    GRIPPER_FRAME = 'claw_base'

    FIXED_FRAME = 'base_cylinder'

    GRIPPER_CLOSED = 0.3
    GRIPPER_OPEN = 0.0

    GRIPPER_JOINT_NAMES = ['joint6','joint7']
   
    GRIPPER_EFFORT = [1.0]

class TestPick():
    def __init__(self):

        roscpp_initialize(sys.argv)
        rospy.init_node('moveit_py_demo', anonymous=True)
      
        scene = PlanningSceneInterface()
        robot = RobotCommander()
       
        right_arm = MoveGroupCommander(GROUP_NAME_ARM)
        right_gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
        #right_arm.set_goal_position_tolerance(0.005)
        right_arm.set_goal_orientation_tolerance(0.005)
	right_arm.set_planning_time(5)
	right_arm.set_num_planning_attempts(5) 
        eef = right_arm.get_end_effector_link()

        rospy.sleep(2)
        # Allow replanning to increase the odds of a solution
        right_arm.allow_replanning(True)
        #scene.remove_attached_object(GRIPPER_FRAME, "part")

   
        # clean the scene
        #scene.remove_world_object("table")
        #scene.remove_world_object("part")
   
        right_arm.set_named_target("default")
        right_arm.go(wait=True)
      
        right_gripper.set_named_target("open")
        right_gripper.go(wait=True)
      
        rospy.sleep(1)
   
        # publish a demo scene
        p = PoseStamped()
        p.header.frame_id = robot.get_planning_frame()
   

        # add an object to be grasped
        p.pose.position.x = 0.170
        p.pose.position.y = 0.04
        p.pose.position.z = 0.3
        #scene.add_box("part", p, (0.07, 0.01, 0.2))
      
        rospy.sleep(1)
             
        start_pose = PoseStamped()
        start_pose.header.frame_id = FIXED_FRAME
   
        # start the gripper in a neutral pose part way to the target
        start_pose.pose.position.x = -0.00756585784256
        start_pose.pose.position.y = -0.225419849157
        start_pose.pose.position.z = 0.117192693055
        start_pose.pose.orientation.x = 0.95493721962
        start_pose.pose.orientation.y = -0.0160209629685
        start_pose.pose.orientation.z = -0.00497157918289
        start_pose.pose.orientation.w = 0.296333402395
	print("going to pick up pose")


          
        right_arm.set_pose_target(start_pose)
	right_gripper.set_named_target("close")
        right_arm.go(wait=True)
        right_gripper.go(wait=True)
       
        rospy.sleep(1)

        right_arm.set_named_target("default")
        right_arm.go(wait=True)


        next_pose = PoseStamped()
        next_pose.header.frame_id = FIXED_FRAME
        next_pose.pose.position.x = -0.100732862949
        next_pose.pose.position.y = -0.210876911879
        next_pose.pose.position.z = 0.244678631425
        next_pose.pose.orientation.x = 0.784905433655
        next_pose.pose.orientation.y = -0.177844554186
        next_pose.pose.orientation.z = -0.131161093712
        next_pose.pose.orientation.w = 0.578870952129



	right_arm.set_pose_target(next_pose)
        right_gripper.set_named_target("open")
        right_arm.go(wait=True)
        right_gripper.go(wait=True)

        rospy.sleep(3)

        right_arm.set_named_target("default")
        right_arm.go(wait=True)
          
        rospy.spin()
        roscpp_shutdown()
       
       
 

if __name__=='__main__':
    TestPick()

