#/usr/bin/env python

from hpp.corbaserver import ProblemSolver
from hpp.corbaserver.pr2 import Robot
import time
from hpp.gepetto import Viewer, PathPlayer
from viewer_library import *
import numpy as np



robot = Robot ('pr2') #35 DOF
robot.setJointBounds ("base_joint_xy", [-4.5, -2.8, -7, -3])
ps = ProblemSolver (robot)
cl = robot.client

r = Viewer (ps); gui = r.client.gui
pp = PathPlayer (robot.client, r)
r.loadObstacleModel ("iai_maps","kitchen_area","kitchen_area")
r.loadObstacleModel ("iai_maps","floor","floor")
r.loadObstacleModel ("iai_maps","chair","chair")
r.loadObstacleModel ("iai_maps","set","set")
r.loadObstacleModel ("iai_maps","flower-vase","flower-vase")

lightName = "li"
r.client.gui.addLight (lightName, r.windowId, 0.005, [0.5,0.5,0.5,0.5])
r.client.gui.addToGroup (lightName, r.sceneName)
r.client.gui.applyConfiguration (lightName, [-4,-6,1,1,0,0,0])
r.client.gui.refresh ()

q1 = robot.getCurrentConfig ()
q2 = q1 [::]
q1 [0:2] = [-3.365, -5.8]
q1 [robot.rankInConfiguration ['torso_lift_joint']] = 0.2
q1 [robot.rankInConfiguration ['torso_lift_joint']] = 0.13
q1 [robot.rankInConfiguration ['l_shoulder_pan_joint']] = 0.5 # spread arms
q1 [robot.rankInConfiguration ['r_shoulder_pan_joint']] = -0.5
q1 [robot.rankInConfiguration ['l_upper_arm_roll_joint']] = 1.57 # turn arm on itself
q1 [robot.rankInConfiguration ['r_upper_arm_roll_joint']] = -1.57
q1 [robot.rankInConfiguration ['l_elbow_flex_joint']] = -0.49 # bend elbow
q1 [robot.rankInConfiguration ['r_elbow_flex_joint']] = -0.50
q1 [robot.rankInConfiguration ['l_wrist_flex_joint']] = -0.2 # bend wrist
q1 [robot.rankInConfiguration ['r_wrist_flex_joint']] = -0.2
q1 [robot.rankInConfiguration ['l_gripper_r_finger_joint']] = 0.09 # open gripper
q1 [robot.rankInConfiguration ['l_gripper_l_finger_joint']] = 0.12
q1 [robot.rankInConfiguration ['r_gripper_r_finger_joint']] = 0.13
q1 [robot.rankInConfiguration ['r_gripper_l_finger_joint']] = 0.095
r (q1)
robot.isConfigValid(q1)


q2 [0:2] = [-3.9, -3.8]
q2 [2:4] = [-1, 0]
q2 [robot.rankInConfiguration ['l_shoulder_pan_joint']] = 0.2 # spread arms
q2 [robot.rankInConfiguration ['r_shoulder_pan_joint']] = -0.7
q2 [robot.rankInConfiguration ['l_shoulder_lift_joint']] = 0.9 # up/down arms
q2 [robot.rankInConfiguration ['r_shoulder_lift_joint']] = -0.1
q2 [robot.rankInConfiguration ['l_upper_arm_roll_joint']] = 1.57 # turn arm on itself
q2 [robot.rankInConfiguration ['r_upper_arm_roll_joint']] = -1.2
q2 [robot.rankInConfiguration ['l_elbow_flex_joint']] = -0.3 # plier coude
q2 [robot.rankInConfiguration ['r_elbow_flex_joint']] = -1.15
q2 [robot.rankInConfiguration ['l_wrist_flex_joint']] = 0 # bend wrist
q2 [robot.rankInConfiguration ['r_wrist_flex_joint']] = -0.3
q2 [robot.rankInConfiguration ['r_wrist_roll_joint']] = -1.57 # turn wrist
q2 [robot.rankInConfiguration ['l_gripper_r_finger_joint']] = 0 # close gripper
q2 [robot.rankInConfiguration ['l_gripper_l_finger_joint']] = 0
q2 [robot.rankInConfiguration ['r_gripper_r_finger_joint']] = 0.08 # open gripper
q2 [robot.rankInConfiguration ['r_gripper_l_finger_joint']] = 0.08
r (q2)
robot.isConfigValid(q2)


ps.setInitialConfig (q1); ps.addGoalConfig (q2)

ps.selectPathValidation ("Dichotomy", 0.)
ps.selectPathPlanner ("VisibilityPrmPlanner")


#ps.directPath (q1,q2,False); pathIdDirect = ps.numberPaths()-1 # generate an invalid direct path

"""
ps.solve ()
pathIdInit = ps.numberPaths()-1
ps.pathLength(pathIdInit)
len(ps.getWaypoints (pathIdInit))


ps.clearPathOptimizers()
cl.problem.setAlphaInit (0.2)
ps.addPathOptimizer("GradientBased")
ps.optimizePath (pathIdInit)
pathIdGB = ps.numberPaths()-1
ps.pathLength(pathIdGB)

#pp(pathIdGB)

"""



### BLENDER EXPORTS ###

#gui.setCaptureTransform ("pr2_kitchen5_configs.yaml", ['pr2'])
#q = q1; r (q); ps.robot.setCurrentConfig(q); gui.refresh (); gui.captureTransform ()

"""
waypoints = ps.getWaypoints (pathIdInit)
for q in waypoints:
	r (q); ps.robot.setCurrentConfig(q); gui.refresh (); gui.captureTransform ()

"""

"""
from viewer_library import *

pathToYamlFile (ps, r, "pr2_kitchen5_init.yaml", 'pr2', pathIdInit, q2, 0.06)
pathToYamlFile (ps, r, "pr2_kitchen5_direct.yaml", 'pr2', pathIdDirect, q2, 0.01)
pathToYamlFile (ps, r, "pr2_kitchen5_GB.yaml", 'pr2', pathIdGB, q2, 0.02)

"""



