#/usr/bin/env python

import time
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver.pr2 import Robot
robot = Robot ('pr2') #35 DOF
robot.setJointBounds ("base_joint_xy", [-4, -3, -5, -3])
ps = ProblemSolver (robot)

from hpp.gepetto import Viewer, PathPlayer
#Viewer.withFloor = True
r = Viewer (ps)
pp = PathPlayer (robot.client, r)
r.loadObstacleModel ("iai_maps","kitchen_area","kitchen_area") # visual kitchen

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

## CONSTRAINTS ##
# "torso_lift_joint" : do not constraint z axis because different for q_init and q_goal
# constraint torso to be at position [-3.25, -4.0, 0.790675] in workspace frame
#ps.createPositionConstraint ("posConstraint", "torso_lift_joint", "", [0,0,0], [-3.25, -4.0, 0.790675], [1,1,0])
#ps.setNumericalConstraints ("constraints", ["posConstraint"])

ps.setInitialConfig (q_init); ps.addGoalConfig (q_goal)

#ps.selectPathValidation ("Dichotomy", 0.)
#ps.selectPathPlanner ("VisibilityPrmPlanner")
#ps.readRoadmap ('/local/mcampana/devel/hpp/data/pr2-in-kitchen.rdm')
ps.solve ()
ps.pathLength(0)
# ps.interruptPathPlanning ()

ps.addPathOptimizer('RandomShortcut')
ps.optimizePath (0)
ps.pathLength(1)

ps.clearPathOptimizers()
ps.addPathOptimizer("GradientBased")
ps.optimizePath (0)
ps.numberPaths()
ps.pathLength(ps.numberPaths()-1)

pp(ps.numberPaths()-1)




len(ps.getWaypoints (0))


# Add light to scene
lightName = "li2"
r.client.gui.addLight (lightName, r.windowId, 0.005, [0.5,0.5,0.5,0.5])
r.client.gui.addToGroup (lightName, r.sceneName)
#r.client.gui.applyConfiguration (lightName, [-2,-4.5,2,1,0,0,0])
r.client.gui.applyConfiguration (lightName, [-4,-4,2,1,0,0,0]) # second light
r.client.gui.refresh ()

## Video recording
pp.dt = 0.02
r.startCapture ("capture","png")
pp(0)
#pp(ps.numberPaths()-1)
r.stopCapture ()

## ffmpeg commands
ffmpeg -r 50 -i capture_0_%d.png -r 25 -vcodec libx264 video.mp4
x=0; for i in *png; do counter=$(printf %03d $x); ln "$i" new"$counter".png; x=$(($x+1)); done
ffmpeg -r 50 -i new%03d.png -r 25 -vcodec libx264 video.mp4

## DEBUG commands
robot.getJointNames ()
r(ps.configAtParam(0,5))
robot.isConfigValid(q1)
from numpy import *
argmin(robot.distancesToCollision()[0])
robot.distancesToCollision()[0][argmin(robot.distancesToCollision()[0])]
ps.resetGoalConfigs ()

