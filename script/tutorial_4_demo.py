#/usr/bin/env python
# Script: PR2 is alone (no obstacle) and crossing its arms.

from hpp.corbaserver import ProblemSolver
from hpp.corbaserver.pr2 import Robot
robot = Robot ('pr2') #35 DOF
robot.setJointBounds ("base_joint_xy", [-3, 3, -3, 3])
ps = ProblemSolver (robot)

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps); cl = robot.client
pp = PathPlayer (robot.client, r)

lightName = "li1"
r.client.gui.addLight (lightName, r.windowId, 0.001, [0.4,0.4,0.4,0.5])
r.client.gui.addToGroup (lightName, r.sceneName)
r.client.gui.applyConfiguration (lightName, [0,1,1,1,0,0,0])
r.client.gui.refresh ()

lightName = "li2"
r.client.gui.addLight (lightName, r.windowId, 0.001, [0.4,0.4,0.4,0.5])
r.client.gui.addToGroup (lightName, r.sceneName)
r.client.gui.applyConfiguration (lightName, [1,0,2,1,0,0,0])
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
ps.pathLength(0)
len(ps.getWaypoints (0))
pp.speed=2.
pp(0)


ps.clearPathOptimizers()
ps.addPathOptimizer("GradientBased")
cl.problem.setAlphaInit (0.2)
ps.optimizePath (0)
ps.numberPaths()
ps.pathLength(ps.numberPaths()-1)
pp.speed=1.
pp(ps.numberPaths()-1)


ps.clearPathOptimizers()
ps.addPathOptimizer('RandomShortcut')
ps.optimizePath (0)
ps.pathLength(ps.numberPaths()-1)
pp.speed=1.
pp(ps.numberPaths()-1)

