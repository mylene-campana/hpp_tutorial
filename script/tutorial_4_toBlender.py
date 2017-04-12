#/usr/bin/env python
# Script: PR2 is alone (no obstacle) and crossing its arms.

from hpp.corbaserver import ProblemSolver
from hpp.corbaserver.pr2 import Robot
robotName = 'pr2'
robot = Robot (robotName) #35 DOF
robot.setJointBounds ("base_joint_xy", [-3, 3, -3, 3])
ps = ProblemSolver (robot)

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps); gui = r.client.gui
pp = PathPlayer (robot.client, r)
q_0 = robot.getCurrentConfig(); r(q_0)

lightName = "li1"; r.client.gui.addLight (lightName, r.windowId, 0.001, [0.4,0.4,0.4,0.5]); r.client.gui.addToGroup (lightName, r.sceneName); r.client.gui.applyConfiguration (lightName, [0,1,1,1,0,0,0])
lightName = "li2"; r.client.gui.addLight (lightName, r.windowId, 0.001, [0.4,0.4,0.4,0.5]); r.client.gui.addToGroup (lightName, r.sceneName); r.client.gui.applyConfiguration (lightName, [1,0,2,1,0,0,0])
r.client.gui.refresh ()

q1 = robot.getCurrentConfig ()
q1 [robot.rankInConfiguration ['l_shoulder_pan_joint']] = -0.02 # un-spread arms
q1 [robot.rankInConfiguration ['r_shoulder_pan_joint']] = 0.02
q1 [robot.rankInConfiguration ['l_shoulder_lift_joint']] = -0.16 # up/down arms
q1 [robot.rankInConfiguration ['r_shoulder_lift_joint']] = 0.14
q1 [robot.rankInConfiguration ['l_upper_arm_roll_joint']] = 1.57 # turn arm on itself
q1 [robot.rankInConfiguration ['r_upper_arm_roll_joint']] = -1.57
q1 [robot.rankInConfiguration ['l_elbow_flex_joint']] = -0.85 # bend elbow
q1 [robot.rankInConfiguration ['r_elbow_flex_joint']] = -0.85
q1 [robot.rankInConfiguration ['l_wrist_flex_joint']] = -0.3 # bend wrist
q1 [robot.rankInConfiguration ['r_wrist_flex_joint']] = -0.3
r(q1)
robot.isConfigValid(q1)

q2=q1[::]
q2 [robot.rankInConfiguration ['l_shoulder_lift_joint']] = 0.14 # up/down arms
q2 [robot.rankInConfiguration ['r_shoulder_lift_joint']] = -0.16
r(q2)
robot.isConfigValid(q2)

ps.setInitialConfig (q1); ps.addGoalConfig (q2)


ps.selectPathPlanner ("VisibilityPrmPlanner")

ps.solve ()
pathIdInit = ps.numberPaths()-1
print("initial length= " + str(ps.pathLength(pathIdInit))) # 20.3
print("number of waypoints= " + str(len(ps.getWaypoints (pathIdInit)))) # 5
pp.speed=2.
#pp(pathIdInit)


ps.clearPathOptimizers()
ps.addPathOptimizer("GradientBased")
ps.client.problem.setAlphaInit (0.2) # default 0.25
ps.optimizePath (pathIdInit)

ps.numberPaths()
pathIdGB = ps.numberPaths()-1
print("length after GB= " + str(ps.pathLength(pathIdGB)))
print("duration of optim GB= " + str(ps.client.problem.getTimeGB ())
pp.speed=1.
#pp(pathIdGB)

"""
ps.clearPathOptimizers()
ps.addPathOptimizer('RandomShortcut')
ps.optimizePath (pathIdInit)
pathIdRS = ps.numberPaths()-1
print("length after RS= " + str(ps.pathLength(pathIdRS)))
pp.speed=1.
#pp(pathIdRS)


ps.clearPathOptimizers()
ps.addPathOptimizer('PartialShortcut')
ps.optimizePath (pathIdInit)
pathIdPRS = ps.numberPaths()-1
print("length after PRS= " + str(ps.pathLength(pathIdPRS)))
pp.speed=1.
#pp(pathIdPRS)
"""

### BLENDER EXPORTS ###

#gui.setCaptureTransform ("pr2_crossingArms_initConfig.yaml", [robotName]) # FOR BLENDER EXPORT ONLY
#q = q1; r (q); ps.robot.setCurrentConfig(q); gui.refresh (); gui.captureTransform () # BLENDER EXPORT

"""
from viewer_library import *

pathToYamlFile (ps, r, "pr2_crossingArms_init", robotName, pathIdInit, q2, 0.06)
pathToYamlFile (ps, r, "pr2_crossingArms_GB", robotName, pathIdGB, q2, 0.01)
pathToYamlFile (ps, r, "pr2_crossingArms_RS", robotName, pathIdRS, q2, 0.01)
pathToYamlFile (ps, r, "pr2_crossingArms_PRS", robotName, pathIdPRS, q2, 0.01)

"""


