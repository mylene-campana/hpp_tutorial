#/usr/bin/env python
#blender/urdf_to_blender.py -p pr2/ -i /local/mcampana/devel/hpp/src/hpp_tutorial/urdf/pr2_full.urdf -o pr2_full_blend.py

from hpp.corbaserver import ProblemSolver
from hpp.corbaserver.pr2 import Robot
from hpp.gepetto import Viewer, PathPlayer
from viewer_library import *
import time
import numpy as np


robot = Robot ('pr2') #35 DOF
robot.setJointBounds ("base_joint_xy", [-4, -3, -5, -3])
ps = ProblemSolver (robot)


r = Viewer (ps); cl = robot.client; gui = r.client.gui

pp = PathPlayer (robot.client, r)
r.loadObstacleModel ("iai_maps","kitchen_area","kitchen_area") # visual kitchen
r.loadObstacleModel ("iai_maps","old_floor","old_floor")

lightName = "li"; r.client.gui.addLight (lightName, r.windowId, 0.005, [0.5,0.5,0.5,0.5]); r.client.gui.addToGroup (lightName, r.sceneName); r.client.gui.applyConfiguration (lightName, [-5,-6,2,1,0,0,0])
lightName = "li2"; r.client.gui.addLight (lightName, r.windowId, 0.005, [0.5,0.5,0.5,0.5]); r.client.gui.addToGroup (lightName, r.sceneName); r.client.gui.applyConfiguration (lightName, [-4,-4,5,1,0,0,0])
r.client.gui.refresh ()

q_init = robot.getCurrentConfig ()
q_goal = q_init [::]
q_init [0:2] = [-3.2, -4]
rank = robot.rankInConfiguration ['torso_lift_joint']
q_init [rank] = 0.2
r (q_init)
robot.isConfigValid(q_init)

q_goal [0:2] = [-3.2, -4]
rank = robot.rankInConfiguration ['l_shoulder_lift_joint']
q_goal [rank] = 0.5
rank = robot.rankInConfiguration ['l_elbow_flex_joint']
q_goal [rank] = -0.5
rank = robot.rankInConfiguration ['r_shoulder_lift_joint']
q_goal [rank] = 0.5
rank = robot.rankInConfiguration ['r_elbow_flex_joint']
q_goal [rank] = -0.5
r (q_goal)
robot.isConfigValid(q_goal)


# HandInTable configs for Blender:
q = [-3.5, -4, 1.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

q [0:2] = [-3.5, -4]; r(q)
q [robot.rankInConfiguration ['torso_lift_joint']] = 0.1; r(q)
q [robot.rankInConfiguration ['r_shoulder_lift_joint']] = 0.41; r(q)
q [robot.rankInConfiguration ['r_elbow_flex_joint']] = -0.4; r(q)
q [robot.rankInConfiguration ['r_wrist_flex_joint']] = -0.1; r(q)
q [robot.rankInConfiguration ['r_gripper_r_finger_joint']] = 0.1; r(q)
q [robot.rankInConfiguration ['r_gripper_l_finger_joint']] = 0.1; r(q)
q [robot.rankInConfiguration ['l_shoulder_lift_joint']] = -0.1; r(q)

q_handInTable = [-3.5, -4, 1.0, 0.0, 0.1, 0.0, 0.0, 0.0, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.41, 0.0, -0.4, 0.0, -0.1, 0.0, 0.1, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0]

q [0:2] = [-3.56, -4]; r(q)
q [robot.rankInConfiguration ['r_shoulder_lift_joint']] = 0.58; r(q)
q [robot.rankInConfiguration ['r_elbow_flex_joint']] = -0.7; r(q)
q [robot.rankInConfiguration ['r_wrist_flex_joint']] = -0.5; r(q)

q_handNotInTable = [-3.56, -4, 1.0, 0.0, 0.1, 0.0, 0.0, 0.0, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.58, 0.0, -0.7, 0.0, -0.5, 0.0, 0.1, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0]


gui.setCaptureTransform ("pr2_kitchen_handInTable_configs.yaml", ['pr2'])
q = q_handInTable; r (q); ps.robot.setCurrentConfig(q); gui.refresh (); gui.captureTransform ()
q = q_handNotInTable; r (q); ps.robot.setCurrentConfig(q); gui.refresh (); gui.captureTransform ()



"""
ps.setInitialConfig (q_init); ps.addGoalConfig (q_goal)

ps.selectPathPlanner ("VisibilityPrmPlanner")
ps.solve ()
ps.pathLength(0)
len(ps.getWaypoints (0))
pp(0)
"""


"""
ps.clearPathOptimizers()
ps.addPathOptimizer('RandomShortcut')
ps.optimizePath (0)
ps.pathLength(ps.numberPaths()-1)
pp(ps.numberPaths()-1)


ps.clearPathOptimizers()
cl.problem.setAlphaInit (0.2)
ps.addPathOptimizer("GradientBased")
ps.optimizePath (0)
ps.numberPaths()
ps.pathLength(ps.numberPaths()-1)
pp(ps.numberPaths()-1)
"""


### BLENDER EXPORTS ###

#gui.setCaptureTransform ("pr2_kitchen5_configs.yaml", ['pr2'])
#q = q_init; r (q); ps.robot.setCurrentConfig(q); gui.refresh (); gui.captureTransform ()

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



