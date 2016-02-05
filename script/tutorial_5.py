#/usr/bin/env python

from hpp.corbaserver import ProblemSolver
from hpp.corbaserver.pr2 import Robot
import time
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
q2 [robot.rankInConfiguration ['r_wrist_roll_joint']] = -1.57 # turn wrist
q2 [robot.rankInConfiguration ['l_gripper_r_finger_joint']] = 0 # close gripper
q2 [robot.rankInConfiguration ['l_gripper_l_finger_joint']] = 0
q2 [robot.rankInConfiguration ['r_gripper_r_finger_joint']] = 0.08 # open gripper
q2 [robot.rankInConfiguration ['r_gripper_l_finger_joint']] = 0.08
r (q2)
robot.isConfigValid(q2)


ps.setInitialConfig (q1); ps.addGoalConfig (q2)

ps.selectPathValidation ("Dichotomy", 0.)
#ps.selectPathPlanner ("VisibilityPrmPlanner")
#ps.readRoadmap ('/local/mcampana/devel/hpp/data/pr2-in-sexy-kitchen.rdm')
#ps.readRoadmap ('/local/mcampana/devel/hpp/data/pr2-in-sexy-kitchen-PRM.rdm') # continuous joints
#ps.readRoadmap ('/local/mcampana/devel/hpp/data/pr2-in-sexy-kitchen-PRM.rdm')
#ps.readRoadmap ('/local/mcampana/devel/hpp/data/pr2-in-sexy-kitchen-PRM2.rdm')
ps.solve ()
ps.pathLength(0)
len(ps.getWaypoints (0))
# ps.interruptPathPlanning ()


cl = robot.client
import numpy as np

ps.addPathOptimizer("Prune")
ps.optimizePath (0)
ps.numberPaths()
ps.pathLength(ps.numberPaths()-1)
len(ps.getWaypoints (ps.numberPaths()-1))

ps.clearPathOptimizers()
cl.problem.setAlphaInit (0.3)
ps.addPathOptimizer("GradientBased")
ps.optimizePath (0)
ps.numberPaths()
ps.pathLength(ps.numberPaths()-1)
tGB = cl.problem.getTimeGB ()
timeValuesGB = cl.problem.getTimeValues ()
gainValuesGB = cl.problem.getGainValues ()
newGainValuesGB = ((1-np.array(gainValuesGB))*100).tolist() #percentage of initial length-value

ps.clearPathOptimizers()
ps.addPathOptimizer('RandomShortcut')
ps.optimizePath (0)
ps.pathLength(ps.numberPaths()-1)

ps.clearPathOptimizers()
ps.addPathOptimizer('PartialShortcut')
ps.optimizePath (0)
ps.pathLength(ps.numberPaths()-1)

pp(ps.numberPaths()-1)


## -------------------------------------
import matplotlib.pyplot as plt
from plotfunctions import optAndConcatenate, getValIndex, computeMeansVector, reducedVectors, curvPlot, curvSdPlot
# OPTIMIZE AND Concatenate RS PRS values:
globalTimeValuesRS = []; globalGainValuesRS = []
globalTimeValuesPRS = []; globalGainValuesPRS = []
nbOpt = 10 # number of launchs of RS and PRS
optAndConcatenate (cl, ps, 1, nbOpt, 'RandomShortcut', globalTimeValuesRS, globalGainValuesRS)
optAndConcatenate (cl, ps, 1, nbOpt, 'PartialShortcut', globalTimeValuesPRS, globalGainValuesPRS)

nbPoints = 100 # number of points in graph
tVec = np.arange(0,tGB,tGB/nbPoints)
moyVectorRS = []; sdVectorRS = []; moyVectorPRS = []; sdVectorPRS = [];
computeMeansVector (nbOpt, tVec, moyVectorRS, sdVectorRS, globalTimeValuesRS, globalGainValuesRS)
computeMeansVector (nbOpt, tVec, moyVectorPRS, sdVectorPRS, globalTimeValuesPRS, globalGainValuesPRS)

tReduceVectorRS = []; meanReduceVectorRS = []; sdReduceVectorRS = [];
tReduceVectorPRS = []; meanReduceVectorPRS = []; sdReduceVectorPRS = [];
reducedVectors (tVec, moyVectorRS, sdVectorRS, tReduceVectorRS, meanReduceVectorRS, sdReduceVectorRS)
reducedVectors (tVec, moyVectorPRS, sdVectorPRS, tReduceVectorPRS, meanReduceVectorPRS, sdReduceVectorPRS)

# Plot lengthGain (t);
plt.axis([-0.4, tGB+0.4, 27, 103])
plt.xlabel('t (s)'); plt.ylabel('Relative remaining length (%)')
vectorLengthGB = len (timeValuesGB)
plt.plot([0,tGB], [newGainValuesGB[vectorLengthGB-1],newGainValuesGB[vectorLengthGB-1]], 'b--')
plt.plot(0, 100, 'bo'); plt.plot([0,timeValuesGB[0]], [100,100], 'b', linewidth=1.5)
plt = curvSdPlot (plt, tGB, tReduceVectorRS, meanReduceVectorRS, sdReduceVectorRS, '0.55', 0.8, 0.1)
plt = curvSdPlot (plt, tGB, tReduceVectorPRS, meanReduceVectorPRS, sdReduceVectorPRS, '0.55', 0.8, 0.1)
plt = curvPlot (plt, tGB, timeValuesGB, newGainValuesGB, 'o', 'b', 1.5)
plt.plot([0,tReduceVectorRS[0]], [100,100], 'r', linewidth=1.1)
plt = curvPlot (plt, tGB, tReduceVectorRS, meanReduceVectorRS, '*', 'r', 1.5)
plt.plot([0,tReduceVectorPRS[0]], [100,100], 'g', linewidth=0.8)
plt = curvPlot (plt, tGB, tReduceVectorPRS, meanReduceVectorPRS, '+', 'g', 1.5)
plt.show()

# For different alpha_init
#tmax = max(max(tGB,tGB2),max(tGB3,tGB4))
tmax = max(max(tGB,tGB2),tGB4)
plt.axis([-4, tmax+4, 28, 101])
plt.xlabel('t (s)'); plt.ylabel('Relative remaining length (%)')
vectorLengthGB = len (timeValuesGB)
#plt.plot([0,tGB], [newGainValuesGB[vectorLengthGB-1],newGainValuesGB[vectorLengthGB-1]], 'b--')
plt.plot(0, 100, 'bo'); plt.plot([0,timeValuesGB[0]], [100,100], 'b', linewidth=1.5)
plt = curvPlot (plt, tmax, timeValuesGB, newGainValuesGB, 'o', 'b', 1.5)
vectorLengthGB2 = len (timeValuesGB2)
plt.plot(0, 100, 'g*'); plt.plot([0,timeValuesGB2[0]], [100,100], 'g', linewidth=1.5)
plt = curvPlot (plt, tmax, timeValuesGB2, newGainValuesGB2, '*', 'g', 1.5)
#vectorLengthGB3 = len (timeValuesGB3)
#plt.plot(0, 100, 'r+'); plt.plot([0,timeValuesGB3[0]], [100,100], 'r', linewidth=1.5)
#plt = curvPlot (plt, tmax, timeValuesGB3, newGainValuesGB3, '+', 'r', 1.5)
vectorLengthGB4 = len (timeValuesGB4)
plt.plot(0, 100, 'c+'); plt.plot([0,timeValuesGB4[0]], [100,100], 'c', linewidth=1.5)
plt = curvPlot (plt, tmax, timeValuesGB4, newGainValuesGB4, '+', 'c', 1.5)
plt.show()

## -------------------------------------

ps.clearPathOptimizers(); ps.addPathOptimizer("GradientBased")
cl.problem.setAlphaInit (0.05)
ps.optimizePath (1); tGB2 = cl.problem.getTimeGB ()
timeValuesGB2 = cl.problem.getTimeValues (); gainValuesGB2 = cl.problem.getGainValues ()
newGainValuesGB2 = ((1-np.array(gainValuesGB2))*100).tolist() #percentage of initial length-value

cl.problem.setAlphaInit (0.3) # SEG FAULT with RRT+prune
ps.optimizePath (1); tGB3 = cl.problem.getTimeGB ()
timeValuesGB3 = cl.problem.getTimeValues (); gainValuesGB3 = cl.problem.getGainValues ()
newGainValuesGB3 = ((1-np.array(gainValuesGB3))*100).tolist() #percentage of initial length-value

cl.problem.setAlphaInit (0.4)
ps.optimizePath (1); tGB4 = cl.problem.getTimeGB ()
timeValuesGB4 = cl.problem.getTimeValues (); gainValuesGB4 = cl.problem.getGainValues ()
newGainValuesGB4 = ((1-np.array(gainValuesGB4))*100).tolist() #percentage of initial length-value

## -------------------------------------


# Add light to scene
lightName = "li"
r.client.gui.addLight (lightName, r.windowId, 0.001, [0.5,0.5,0.5,1])
r.client.gui.addToGroup (lightName, r.sceneName)
#r.client.gui.applyConfiguration (lightName, [-6,-2,2,1,0,0,0])
#r.client.gui.applyConfiguration (lightName, [-4,-4,2,1,0,0,0]) # second light
r.client.gui.applyConfiguration (lightName, [-3,-3,2,1,0,0,0]) # third light
#r.client.gui.applyConfiguration (lightName, [-4,-7,2,1,0,0,0]) # third light
#r.client.gui.applyConfiguration (lightName, [-3,-8,0.5,1,0,0,0]) # fourth light
r.client.gui.refresh ()
#r.client.gui.removeFromGroup (lightName, r.sceneName)

## Video recording
import time
pp.dt = 0.02
pp.speed=1
r(q1)
r.startCapture ("capture","png")
r(q1); time.sleep(0.2)
r(q1)
pp(0)
#pp(ps.numberPaths()-1)
r(q2); time.sleep(1);
r.stopCapture ()

## ffmpeg commands
ffmpeg -r 50 -i capture_0_%d.png -r 25 -vcodec libx264 video.mp4
x=0; for i in *png; do counter=$(printf %04d $x); ln "$i" new"$counter".png; x=$(($x+1)); done
ffmpeg -r 50 -i new%04d.png -r 25 -vcodec libx264 video.mp4
mencoder video.mp4 -channels 6 -ovc xvid -xvidencopts fixed_quant=4 -vf harddup -oac pcm -o video.avi
ffmpeg -i untitled.mp4 -vcodec libx264 -crf 24 video.mp4

## DEBUG commands
robot.getJointNames ()
r(ps.configAtParam(0,5))
robot.isConfigValid(q1)
from numpy import *
argmin(robot.distancesToCollision()[0])
robot.distancesToCollision()[0][argmin(robot.distancesToCollision()[0])]
ps.resetGoalConfigs ()
robot.getJointNames ()


## Debug Optimization Tools ##############
num_log = 32571
from parseLog import parseCollConstrPoints, parseNodes

collConstrNodes = parseNodes (num_log, '189: qFree_ = ')
collNodes = parseNodes (num_log, '182: qColl = ')

contactPoints = parseCollConstrPoints (num_log, '77: contact point = (')
x1_J1 = parseCollConstrPoints (num_log, '96: x1 in R0 = (')
x2_J1 = parseCollConstrPoints (num_log, '97: x2 in R0 = (')
x1_J2 = parseCollConstrPoints (num_log, '116: x1 in J2 = (')
x2_J2 = parseCollConstrPoints (num_log, '117: x2 in J2 = (') #x2_J2 <=> contactPoints


## same with viewer !
from viewer_display_library_OPTIM import transformInConfig, plotPoints, plotPointsAndLines, plot2DBaseCurvPath, plotDofCurvPath, plotPointBodyCurvPath, plotBodyCurvPath
contactPointsViewer = transformInConfig (contactPoints)
x1_J1Viewer = transformInConfig (x1_J1)
x2_J1Viewer = transformInConfig (x2_J1)
x1_J2Viewer = transformInConfig (x1_J2)
x2_J2Viewer = transformInConfig (x2_J2)

# Plot points
sphereNamePrefix = "sphereContactPoints_"
plotPoints (r, sphereNamePrefix, contactPointsViewer, 0.02)
sphereSize=0.01
lineNamePrefix = "lineJ1_"; sphereNamePrefix = "sphereJ1_"
plotPointsAndLines (r, lineNamePrefix, sphereNamePrefix, x1_J1Viewer, x2_J1Viewer, sphereSize)
lineNamePrefix = "lineJ2_"; sphereNamePrefix = "sphereJ2_"
plotPointsAndLines (r, lineNamePrefix, sphereNamePrefix, x1_J2Viewer, x2_J2Viewer, sphereSize)


from viewer_display_library_OPTIM import plotPointBodyCurvPath
from hpp.corbaserver import Client
cl = robot.client
dt = 0.03

jointName = 'base_joint_xy'
plotPointBodyCurvPath (r, cl, robot, dt, 0, jointName, [0,0,0.1], 'pathPoint_'+jointName, [1,0.1,0.1,1])
plotPointBodyCurvPath (r, cl, robot, dt, 11, jointName, [0,0,0.1], 'pathPointRS_'+jointName, [0.1,0.1,1,1])
plotPointBodyCurvPath (r, cl, robot, dt, 10, jointName, [0,0,0.1], 'pathPointGB_'+jointName, [0.2,1,0.2,1])

jointPosition = robot.getJointPosition ('base_joint_xy')
pointInJoint = [0,0,0.1]
posAtester = cl.robot.computeGlobalPosition (jointPosition, pointInJoint)

r(q1)
robot.setCurrentConfig (q1)
sphereName = "machin"
r.client.gui.addSphere (sphereName,0.03,[0.1,0.1,0.1,1]) # black
configSphere = posAtester [::]
configSphere.extend ([1,0,0,0])
r.client.gui.applyConfiguration (sphereName,configSphere)
r.client.gui.addToGroup (sphereName, r.sceneName)
r.client.gui.refresh ()

r.client.gui.removeFromGroup (lightName, r.sceneName)
