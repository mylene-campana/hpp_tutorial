#/usr/bin/env python

from hpp.corbaserver import ProblemSolver
from hpp.corbaserver.pr2 import Robot
robot = Robot ('pr2') #35 DOF
robot.setJointBounds ("base_joint_xy", [-4.5, -2.8, -7, -3])
ps = ProblemSolver (robot)

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (robot.client, r)
r.loadObstacleModel ("iai_maps","kitchen_area","kitchen_area")
r.loadObstacleModel ("iai_maps","floor","floor")
r.loadObstacleModel ("iai_maps","chair","chair")
r.loadObstacleModel ("iai_maps","set","set")
r.loadObstacleModel ("iai_maps","flower-vase","flower-vase")

q1 = robot.getCurrentConfig ()
q2 = q1 [::]
q1 [0:2] = [-3.36, -5.8]
rank = robot.rankInConfiguration ['torso_lift_joint']
q1 [rank] = 0.2
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
q2 [robot.rankInConfiguration ['r_wrist_roll_joint']] = 0 # turn wrist [CONTINUOUS]
q2 [robot.rankInConfiguration ['r_wrist_roll_joint']+1] = 1 # turn wrist poignet
q2 [robot.rankInConfiguration ['l_gripper_r_finger_joint']] = 0 # open gripper
q2 [robot.rankInConfiguration ['l_gripper_l_finger_joint']] = 0
r (q2)
robot.isConfigValid(q2)


ps.setInitialConfig (q1); ps.addGoalConfig (q2)

#ps.selectPathValidation ("Dichotomy", 0.)
#ps.selectPathPlanner ("VisibilityPrmPlanner")
#ps.readRoadmap ('/local/mcampana/devel/hpp/data/pr2-in-sexy-kitchen.rdm')
ps.readRoadmap ('/local/mcampana/devel/hpp/data/pr2-in-sexy-kitchen-PRM.rdm')
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
lightName = "li3"
r.client.gui.addLight (lightName, r.windowId, 0.005, [0.5,0.5,0.5,0.5])
r.client.gui.addToGroup (lightName, r.sceneName)
#r.client.gui.applyConfiguration (lightName, [-6,-2,2,1,0,0,0])
#r.client.gui.applyConfiguration (lightName, [-4,-4,2,1,0,0,0]) # second light
r.client.gui.applyConfiguration (lightName, [-3,-3,2,1,0,0,0]) # third light
#r.client.gui.applyConfiguration (lightName, [-4,-7,2,1,0,0,0]) # third light
r.client.gui.refresh ()
#r.client.gui.removeFromGroup (lightName, r.sceneName)

## Video recording
pp.dt = 0.02
r.startCapture ("capture","png")
pp(0)
#pp(ps.numberPaths()-1)
r.stopCapture ()

r.startCapture ("capture","png")
r(q2)
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

