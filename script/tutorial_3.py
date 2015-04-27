#/usr/bin/env python

import time
from hpp.gepetto import Viewer, PathPlayer
from hpp.corbaserver.pr2_with_set import Robot
robot = Robot ('pr2_with_set') # will containt the set fixed on the left hand... (moving with l_wrist_flex_joint)
robot.setJointBounds ("base_joint_xy", [-4, -0.5, -7, -1.5])

from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)
Viewer.withFloor = True
r = Viewer (ps)
r.loadObstacleModel ("iai_maps","kitchen_area","kitchen_area") # visual kitchen
r.loadObstacleModel ("iai_maps","floor","floor")

q1 = robot.getCurrentConfig ()
q1 [0:2] = [-3.4, -6]
q1 [robot.rankInConfiguration ['torso_lift_joint']] = 0.13
q1 [robot.rankInConfiguration ['l_shoulder_pan_joint']] = 0.5 # ecarte les bras
q1 [robot.rankInConfiguration ['r_shoulder_pan_joint']] = -0.5
q1 [robot.rankInConfiguration ['l_upper_arm_roll_joint']] = 1.57 # tourne bras sur lui-mm
q1 [robot.rankInConfiguration ['r_upper_arm_roll_joint']] = -1.57
q1 [robot.rankInConfiguration ['l_elbow_flex_joint']] = -0.49 # plier coude
q1 [robot.rankInConfiguration ['r_elbow_flex_joint']] = -0.50
q1 [robot.rankInConfiguration ['l_wrist_flex_joint']] = -0.2 # plie poignet
q1 [robot.rankInConfiguration ['r_wrist_flex_joint']] = -0.2
q1 [robot.rankInConfiguration ['l_gripper_r_finger_joint']] = 0.09 # ouvrir pince
q1 [robot.rankInConfiguration ['l_gripper_l_finger_joint']] = 0.12
q1 [robot.rankInConfiguration ['r_gripper_r_finger_joint']] = 0.13
q1 [robot.rankInConfiguration ['r_gripper_l_finger_joint']] = 0.095
r(q1)

q2 = q1 [::]
q2 [0:2] = [-1.5, -4] # x, y
#q2 [2:4] = [-0.707, 0.707] # theta
q2 [2:4] = [-1, 0] # theta
q2 [robot.rankInConfiguration ['torso_lift_joint']] = 0.02
r (q2)


## CONSTRAINTS ##
# Relative position constraint between PR2's right hand and the set's right handle.
ps.createPositionConstraint ("posConstraint1", "r_gripper_r_finger_joint", "j_marker_set", [0,0,0], [0.8455, -0.089, 0.01], [1,1,1])
#ps.createPositionConstraint ("posConstraint2", "r_gripper_l_finger_joint", "j_marker_set", [0,0,0], [0.841, 0.001, -0.0248], [1,1,1]) #WRONG!

# Glogal orientation constraint of the set that has to stay horizontal.
ps.createOrientationConstraint ("orConstraint", "j_marker_set", "", [0.707106781,0,0,-0.707106781], [1,1,0])

ps.setNumericalConstraints ("constraints", ["posConstraint1","orConstraint"])


res = ps.applyConstraints (q1)
if res [0]:
    q1proj = res [1]
else:
    raise RuntimeError ("Failed to apply constraint.")

if not(robot.isConfigValid(q1proj)):
    raise RuntimeError ("Projected config non valid.")

res = ps.applyConstraints (q2)
if res [0]:
    q2proj = res [1]
else:
    raise RuntimeError ("Failed to apply constraint.")

if not(robot.isConfigValid(q2proj)):
    raise RuntimeError ("Projected config non valid.")

ps.setInitialConfig (q1proj); ps.addGoalConfig (q2proj)
ps.loadObstacleFromUrdf ("iai_maps", "kitchen_area", "") # environment

#RRT-connect: path of 36s with 68 waypoints (2600 var to optimize)
#ps.selectPathPlanner ("VisibilityPrmPlanner")

ps.solve ()
ps.optimizePath(0)


pp = PathPlayer (robot.client, r)

pp (0)
pp (1)

len(ps.getWaypoints (0))
robot.client.problem.getIterationNumber ()
ps.pathLength(0)
ps.pathLength(1)


## Video recording
r.startCapture ("capture","png")
pp(0)
r.stopCapture ()

## DEBUG commands
cl.obstacle.getObstaclePosition('obstacle_base')
cl.robot.getJointOuterObjects('CHEST_JOINT1')
cl.robot.getCurrentConfig()
robot.isConfigValid(q1)
cl.problem.pathLength(1)
r(cl.problem.configAtParam(1,5))
cl.problem.clearRoadmap ()
cl.problem.optimizePath (2)
cl.problem.directPath(q1,q2)
from numpy import *
robot.distancesToCollision()[1][argmin(robot.distancesToCollision()[0])]
robot.distancesToCollision()[2][argmin(robot.distancesToCollision()[0])]
robot.getJointNames ()
robot.getJointPosition ("torso_lift_joint")
begin=time.time()
end=time.time(); print "Solving time: "+str(end-begin)
