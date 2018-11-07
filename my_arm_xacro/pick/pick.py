#!/usr/bin/env python

"""
    moveit_pick_and_place_demo.py - Version 0.1 2014-01-14
    
    Command the gripper to grasp a target object and move it to a new location, all
    while avoiding simulated obstacles.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy

GROUP_NAME_ARM = 'arm'
GROUP_NAME_GRIPPER = 'gripper'

GRIPPER_FRAME = 'claw_base'

GRIPPER_OPEN = [0.3,0.3]
GRIPPER_CLOSED = [0.0,0.0]
GRIPPER_NEUTRAL = [0.009,0.009]
GRIPPER_GRASP = [0.017,0.017]

GRIPPER_JOINT_NAMES = ['joint6','joint7']

GRIPPER_EFFORT = [1.0]

REFERENCE_FRAME = 'base_cylinder'

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node('moveit_demo')
        
        # Use the planning scene object to add or remove objects
        scene = PlanningSceneInterface()
        
        # Create a scene publisher to push changes to the scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
        
        # Create a publisher for displaying gripper poses
        self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped, queue_size=5)
        
        # Create a dictionary to hold object colors
        self.colors = dict()
                        
        # Initialize the move group for the right arm
        right_arm = MoveGroupCommander(GROUP_NAME_ARM)
        
        # Initialize the move group for the right gripper
        right_gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
        
        # Get the name of the end-effector link
        end_effector_link = right_arm.get_end_effector_link()
	print(end_effector_link)
 
        # Allow some leeway in position (meters) and orientation (radians)
        right_arm.set_goal_position_tolerance(0.1)
        right_arm.set_goal_orientation_tolerance(0.1)

        # Allow replanning to increase the odds of a solution
        right_arm.allow_replanning(True)
        
        # Set the right arm reference frame
        right_arm.set_pose_reference_frame(REFERENCE_FRAME)
        
        # Allow 5 seconds per planning attempt
        right_arm.set_planning_time(5)
        
        # Set a limit on the number of pick attempts before bailing
        max_pick_attempts = 2
        
        # Set a limit on the number of place attempts
        max_place_attempts = 5
                
        # Give the scene a chance to catch up
        rospy.sleep(2)



   
        # clean the scene
        scene.remove_world_object("table")
        scene.remove_world_object("part")
        
        # Remove any attached objects from a previous session
        scene.remove_attached_object(GRIPPER_FRAME, "part")
        
        # Give the scene a chance to catch up    
        rospy.sleep(1)
        
        # Start the arm in the "grasp" pose stored in the SRDF file
        right_arm.set_named_target('default')
        right_arm.go()
        
        # Open the gripper to the neutral position
        right_gripper.set_joint_value_target(GRIPPER_OPEN)
        right_gripper.go()
       
        rospy.sleep(1)


        place_pose = PoseStamped()
        place_pose.header.frame_id = REFERENCE_FRAME
        place_pose.pose.position.x = 0.240
        place_pose.pose.position.y = 0.01
        place_pose.pose.position.z = 0.3
        scene.add_box("part", place_pose, (0.07, 0.01, 0.2))
        
        # Specify a pose to place the target after being picked up
        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        # start the gripper in a neutral pose part way to the target
        target_pose.pose.position.x = 0.0733923763037
        target_pose.pose.position.y = 0.0129100652412
        target_pose.pose.position.z = 0.3097191751
        target_pose.pose.orientation.x = -0.524236500263
        target_pose.pose.orientation.y = 0.440069645643
        target_pose.pose.orientation.z = -0.468739062548
        target_pose.pose.orientation.w = 0.558389186859

        # Initialize the grasp pose to the target pose
        grasp_pose = target_pose
	print("going to start pose")
	right_arm.set_pose_target(target_pose)
        right_arm.go()

        rospy.sleep(2)

        
        # Shift the grasp pose by half the width of the target to center it
        #grasp_pose.pose.position.y -= target_size[1] / 2.0
        #grasp_pose.pose.position.x = 0.12792118579
        #grasp_pose.pose.position.y = -0.285290879999
        #grasp_pose.pose.position.z = 0.120301181892
                
        # Generate a list of grasps
        grasps = self.make_grasps(grasp_pose, 'part')

        # Publish the grasp poses so they can be viewed in RViz
        print "Publishing grasps"
        for grasp in grasps:
            self.gripper_pose_pub.publish(grasp.grasp_pose)
            rospy.sleep(0.2)

        # Track success/failure and number of attempts for pick operation
        result = None
        n_attempts = 0
        
        # Repeat until we succeed or run out of attempts
        while result != MoveItErrorCodes.SUCCESS and n_attempts < max_pick_attempts:
            n_attempts += 1
            rospy.loginfo("Pick attempt: " +  str(n_attempts))
            result = right_arm.pick('part',grasps)
            rospy.sleep(0.2)
        
        # If the pick was successful, attempt the place operation   
        if result == MoveItErrorCodes.SUCCESS:
            result = None
            n_attempts = 0
            
            # Generate valid place poses
            places = self.make_places(place_pose)
            
            # Repeat until we succeed or run out of attempts
            while result != MoveItErrorCodes.SUCCESS and n_attempts < max_place_attempts:
                n_attempts += 1
                rospy.loginfo("Place attempt: " +  str(n_attempts))
                for place in places:
                    result = right_arm.place('part', place)
                    if result == MoveItErrorCodes.SUCCESS:
                        break
                rospy.sleep(0.2)
                
            if result != MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Place operation failed after " + str(n_attempts) + " attempts.")
            else:
                # Return the arm to the "resting" pose stored in the SRDF file
                right_arm.set_named_target('right_arm_rest')
                right_arm.go()
                
                # Open the gripper to the open position
                right_gripper.set_joint_value_target(GRIPPER_OPEN)
                right_gripper.go()
        else:
            rospy.loginfo("Pick operation failed after " + str(n_attempts) + " attempts.")

        rospy.sleep(1)

        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit the script
        moveit_commander.os._exit(0)
        
    # Get the gripper posture as a JointTrajectory
    def make_gripper_posture(self, joint_positions):
        # Initialize the joint trajectory for the gripper joints
        t = JointTrajectory()
        
        # Set the joint names to the gripper joint names
        t.joint_names = GRIPPER_JOINT_NAMES
        
        # Initialize a joint trajectory point to represent the goal
        tp = JointTrajectoryPoint()
        
        # Assign the trajectory joint positions to the input positions
        tp.positions = joint_positions
        
        # Set the gripper effort
        tp.effort = GRIPPER_EFFORT
        
        tp.time_from_start = rospy.Duration(1.0)
        
        # Append the goal point to the trajectory points
        t.points.append(tp)
        
        # Return the joint trajectory
        return t
    
    # Generate a gripper translation in the direction given by vector
    def make_gripper_translation(self, min_dist, desired, vector):
        # Initialize the gripper translation object
        g = GripperTranslation()
        
        # Set the direction vector components to the input
        g.direction.vector.x = vector[0]
        g.direction.vector.y = vector[1]
        g.direction.vector.z = vector[2]
        
        # The vector is relative to the gripper frame
        g.direction.header.frame_id = GRIPPER_FRAME
        
        # Assign the min and desired distances from the input
        g.min_distance = min_dist
        g.desired_distance = desired
        
        return g

    # Generate a list of possible grasps
    def make_grasps(self, initial_pose_stamped, allowed_touch_objects):
        # Initialize the grasp object
        g = Grasp()
        
        # Set the pre-grasp and grasp postures appropriately
        g.pre_grasp_posture = self.make_gripper_posture(GRIPPER_OPEN)
        g.grasp_posture = self.make_gripper_posture(GRIPPER_GRASP)
                
        # Set the approach and retreat parameters as desired
        g.pre_grasp_approach = self.make_gripper_translation(0.1, 0.1, [0, 1, 0])
        g.post_grasp_retreat = self.make_gripper_translation(0.1, 0.15, [1, 0, 0])
        
        # Set the first grasp pose to the input pose
        g.grasp_pose = initial_pose_stamped
    
        ideal_roll = 0
        ideal_pitch = 0
        ideal_yaw = 0
        
        step_size = 0.1
        idx = 0.1
        idx_roll = ideal_roll + idx
        idx_pitch = ideal_pitch + idx
        idx_yaw = ideal_yaw + idx
        roll_vals = []
        pitch_vals = []
        yaw_vals = []
        while idx >= -0.1:
            roll_vals.append(idx_roll)
            pitch_vals.append(idx_pitch)
            yaw_vals.append(idx_yaw)
            idx -= step_size
            idx_roll -= step_size
            idx_pitch -= step_size
            idx_yaw -= step_size

        # A list to hold the grasps
        grasps = []
        
        print "Generating Poses"
        
        # Generate a grasp for each roll pitch and yaw angle
        for r in roll_vals:
            for y in yaw_vals:
                for p in pitch_vals:
                    # Create a quaternion from the Euler angles
                    q = quaternion_from_euler(r, p, y)
                    
                    # Set the grasp pose orientation accordingly
                    g.grasp_pose.pose.orientation.x = q[0]
                    g.grasp_pose.pose.orientation.y = q[1]
                    g.grasp_pose.pose.orientation.z = q[2]
                    g.grasp_pose.pose.orientation.w = q[3]
                    
                    # Set and id for this grasp (simply needs to be unique)
                    g.id = str(len(grasps))
                    
                    # Set the allowed touch objects to the input list
                    g.allowed_touch_objects = allowed_touch_objects
                    
                    # Don't restrict contact force
                    g.max_contact_force = 0
                    
                    # Degrade grasp quality for increasing pitch angles
                    g.grasp_quality = 1.0 - abs(p)
                    
                    # Append the grasp to the list
                    grasps.append(deepcopy(g))
                    
        print "Generated " + g.id + " poses"
        # Return the list
        return grasps
    
    # Generate a list of possible place poses
    def make_places(self, init_pose):
        # Initialize the place location as a PoseStamped message
        place = PoseStamped()
        
        # Start with the input place pose
        place = init_pose
        
        # A list of x shifts (meters) to try
        x_vals = [0, 0.005, 0.01, 0.015, -0.005, -0.01, -0.015]
        
        # A list of y shifts (meters) to try
        y_vals = [0, 0.005, 0.01, 0.015, -0.005, -0.01, -0.015]
        
        pitch_vals = [0, 0.1, -0.1, 0.2, -0.2, 0.3, -0.3]
        
        # A list of yaw angles to try
        yaw_vals = [0, 0.1, -0.1, 0.2, -0.2, 0.3, -0.3]

        # A list to hold the places
        places = []
        
        # Generate a place pose for each angle and translation
        for y in yaw_vals:
            for p in pitch_vals:
                for y in y_vals:
                    for x in x_vals:
                        place.pose.position.x = init_pose.pose.position.x + x
                        place.pose.position.y = init_pose.pose.position.y + y
                        
                        # Create a quaternion from the Euler angles
                        q = quaternion_from_euler(0, p, y)
                        
                        # Set the place pose orientation accordingly
                        place.pose.orientation.x = q[0]
                        place.pose.orientation.y = q[1]
                        place.pose.orientation.z = q[2]
                        place.pose.orientation.w = q[3]
                        
                        # Append this place pose to the list
                        places.append(deepcopy(place))
        
        # Return the list
        return places
    
    # Set the color of an object
    def setColor(self, name, r, g, b, a = 0.9):
        # Initialize a MoveIt color object
        color = ObjectColor()
        
        # Set the id to the name given as an argument
        color.id = name
        
        # Set the rgb and alpha values given as input
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        
        # Update the global color dictionary
        self.colors[name] = color

    # Actually send the colors to MoveIt!
    def sendColors(self):
        # Initialize a planning scene object
        p = PlanningScene()

        # Need to publish a planning scene diff        
        p.is_diff = True
        
        # Append the colors from the global color dictionary 
        for color in self.colors.values():
            p.object_colors.append(color)
        
        # Publish the scene diff
        self.scene_pub.publish(p)

if __name__ == "__main__":
    MoveItDemo()
