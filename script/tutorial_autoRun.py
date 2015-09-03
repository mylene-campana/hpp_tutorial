#/usr/bin/env python
# autorun script to visualize faster the set position in the hands of PR2

import time
from hpp.gepetto import Viewer, PathPlayer
from hpp.corbaserver.pr2_with_set import Robot
robot = Robot ('pr2_with_set') # will contain the set fixed on the left hand... (moving with l_wrist_flex_joint)
robot.setJointBounds ("base_joint_xy", [-4, -0.5, -7, -1.5])

from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)
Viewer.withFloor = True
r = Viewer (ps)
r.loadObstacleModel ("iai_maps","kitchen_area","kitchen_area") # visual kitchen

q1 = robot.getCurrentConfig ()
q1 [0:2] = [-3.4, -6]
q1 [robot.rankInConfiguration ['torso_lift_joint']] = 0.13
q1 [robot.rankInConfiguration ['l_shoulder_pan_joint']] = 0.5 # spread arms
q1 [robot.rankInConfiguration ['r_shoulder_pan_joint']] = -0.5
q1 [robot.rankInConfiguration ['l_upper_arm_roll_joint']] = 1.57 # turn arm on itself
q1 [robot.rankInConfiguration ['r_upper_arm_roll_joint']] = -1.57
q1 [robot.rankInConfiguration ['l_elbow_flex_joint']] = -0.49 # bend elbow
q1 [robot.rankInConfiguration ['r_elbow_flex_joint']] = -0.50
q1 [robot.rankInConfiguration ['l_wrist_flex_joint']] = -0.2 # bend wrist
q1 [robot.rankInConfiguration ['r_wrist_flex_joint']] = -0.2
q1 [robot.rankInConfiguration ['l_gripper_r_finger_joint']] = 0.09 # open clamp
q1 [robot.rankInConfiguration ['l_gripper_l_finger_joint']] = 0.12
q1 [robot.rankInConfiguration ['r_gripper_r_finger_joint']] = 0.13
q1 [robot.rankInConfiguration ['r_gripper_l_finger_joint']] = 0.095

q2 = q1 [::]
q2 [0:2] = [-1.5, -4] # x, y
q2 [2:4] = [-1, 0] # theta
q2 [robot.rankInConfiguration ['torso_lift_joint']] = 0.02

ps.createPositionConstraint ("posConstraint1", "r_gripper_r_finger_joint", "j_marker_set", [0,0,0], [0.8455, -0.089, 0.01], [1,1,1])
ps.setNumericalConstraints ("constraints", ["posConstraint1"])

res = ps.applyConstraints (q1)
if res [0]:
    q1proj = res [1]
else:
    raise RuntimeError ("Failed to apply constraint.")

res = ps.applyConstraints (q2)
if res [0]:
    q2proj = res [1]
else:
    raise RuntimeError ("Failed to apply constraint.")

r(q1proj)
