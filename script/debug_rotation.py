#/usr/bin/env python

import time
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver.pr2 import Robot
robot = Robot ('pr2') #35 DOF
robot.setJointBounds ("base_joint_xy", [-1, 1, -1, 1])
ps = ProblemSolver (robot)

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (robot.client, r)

q1 = robot.getCurrentConfig ()
q2 = q1 [::]
r (q1)

q2 [2:4] = [-1, 0]
r (q2)

ps.setInitialConfig (q1); ps.addGoalConfig (q2)

#ps.selectPathValidation ("Dichotomy", 0.)

ps.solve ()
ps.pathLength(0)
# ps.interruptPathPlanning ()

#ps.addPathOptimizer('RandomShortcut')
#ps.optimizePath (0)
#ps.pathLength(1)
#ps.clearPathOptimizers()

ps.addPathOptimizer("GradientBased")
ps.optimizePath (0)
ps.numberPaths()
ps.pathLength(ps.numberPaths()-1)

pp(ps.numberPaths()-1)




len(ps.getWaypoints (0))


# Add light to scene
lightName = "li"
r.client.gui.addLight (lightName, r.windowId, 0.005, [0.5,0.5,0.5,0.5])
r.client.gui.addToGroup (lightName, r.sceneName)
r.client.gui.applyConfiguration (lightName, [-5,-2,2,1,0,0,0])
#r.client.gui.applyConfiguration (lightName, [-4,-4,2,1,0,0,0]) # second light
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
robot.getJointNames ()

