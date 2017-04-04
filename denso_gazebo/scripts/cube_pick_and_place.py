#!/usr/bin/env python
import sys
import copy
import rospy
import numpy as np
import moveit_commander
# Messages
from geometry_msgs.msg import Point, PoseStamped

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('cube_pick_and_place')
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm = robot.get_group('denso_robotiq_85_gripper')
gripper = robot.get_group('robotiq_85_gripper')
# Clean the planning scene
scene.remove_world_object('cube01')
scene.remove_world_object('cube02')
scene.remove_world_object('cube03')
# Add the cubes to the planning scene
p1 = PoseStamped()
p1.header.frame_id = robot.get_planning_frame()
p1.pose.position.x = 0.4
p1.pose.position.y = -0.2
p1.pose.position.z = 0.775-0.59+0.025
p1.pose.orientation.w = 1.0
scene.add_box("cube01", p1, (0.05, 0.05, 0.05))
p2 = copy.deepcopy(p1)
p2.pose.position.y = 0.0
scene.add_box("cube02", p2, (0.05, 0.05, 0.05))
p3 = copy.deepcopy(pose1)
p3.pose.position.y = 0.2
scene.add_box("cube03", p3, (0.05, 0.05, 0.05))
