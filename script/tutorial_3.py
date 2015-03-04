#/usr/bin/env python

import time
from hpp.gepetto import Viewer, PathPlayer
from hpp.corbaserver.pr2 import Robot
robot = Robot ('pr2_with_set') # will containt the set fixed on the left hand... (moving with l_wrist_flex_joint)
robot.setJointBounds ("base_joint_xy", [-4, -0.5, -7, -1.5])

from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)
Viewer.withFloor = True
r = Viewer (ps)
r.loadObstacleModel ("iai_maps","kitchen_area","") # visual kitchen

q1 = robot.getCurrentConfig ()
q1 [0:2] = [-3.4, -6]
q1 [robot.rankInConfiguration ['torso_lift_joint']] = 0.12
q1 [robot.rankInConfiguration ['l_shoulder_pan_joint']] = 0.5 # ecarte les bras
q1 [robot.rankInConfiguration ['r_shoulder_pan_joint']] = -0.5
q1 [robot.rankInConfiguration ['l_upper_arm_roll_joint']] = 1.57 # tourne bras sur lui-mm
q1 [robot.rankInConfiguration ['r_upper_arm_roll_joint']] = -1.57
q1 [robot.rankInConfiguration ['l_elbow_flex_joint']] = -0.49 # plier coude
q1 [robot.rankInConfiguration ['r_elbow_flex_joint']] = -0.49
q1 [robot.rankInConfiguration ['l_wrist_flex_joint']] = -0.2 # plie poignet
q1 [robot.rankInConfiguration ['r_wrist_flex_joint']] = -0.2
q1 [robot.rankInConfiguration ['l_gripper_r_finger_joint']] = 0.09 # ouvrir pince
q1 [robot.rankInConfiguration ['l_gripper_l_finger_joint']] = 0.12
q1 [robot.rankInConfiguration ['r_gripper_r_finger_joint']] = 0.13
q1 [robot.rankInConfiguration ['r_gripper_l_finger_joint']] = 0.075
r(q1)

q2 = q1 [::]
q2 [0:2] = [-1.5, -4] # x, y
#q2 [2:4] = [-0.707, 0.707] # theta
q2 [2:4] = [-1, -1] # theta
q2 [robot.rankInConfiguration ['torso_lift_joint']] = 0.02
r (q2)


## CONSTRAINTS ##
# Relative position constraint between PR2's right hand and the set's right handle.
ps.createPositionConstraint ("posConstraint", "r_wrist_flex_joint", "l_wrist_flex_joint", [0,0,0], [0.74, 0.31, -0.04], [1,1,1]) # TODO adjust pos. values ...

# Glogal orientation constraint of the set that has to stay horizontal.
ps.createOrientationConstraint ("orConstraint", "l_wrist_flex_joint", "", [1,0,0,0], [1,1,0]) 
ps.setNumericalConstraints ("constraints", ["posConstraint","orConstraint"])

ps.loadObstacleFromUrdf ("iai_maps", "kitchen_area", "") # environment

ps.setInitialConfig (q1)
ps.addGoalConfig (q2)
#ps.selectPathPlanner ("PRM")

ps.solve ()
ps.optimizePath(0)

begin=time.time()
end=time.time()
print "Solving time: "+str(end-begin)


pp = PathPlayer (robot.client, r)

pp (0)
pp (1)

len(ps.nodes ())
ps.pathLength(0)
cl.problem.optimizePath(0)

robot.getJointNames ()
robot.getJointPosition ("torso_lift_joint")
robot.setCurrentConfig (q_init)

## RESULTS ##
# RRT-connect :
# 30 nodes, 7.57s solution length, 79.3s solving time

# visib-PRM :
# 6 nodes, 14.17s solution length, 160.8s solving time

rank = robot.rankInConfiguration ['l_wrist_flex_joint']
q_init [rank] = 0.2
r (q_init)
