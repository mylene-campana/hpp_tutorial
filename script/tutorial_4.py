#/usr/bin/env python
# Script: PR2 is alone (no obstacle) and crossing its arms.

from hpp.corbaserver import ProblemSolver
from hpp.corbaserver.pr2 import Robot
robot = Robot ('pr2') #35 DOF
robot.setJointBounds ("base_joint_xy", [-5, 5, -5, 5])
ps = ProblemSolver (robot)

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (robot.client, r)

q1 = robot.getCurrentConfig ()
#[0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
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


#ps.selectPathValidation ("Dichotomy", 0.)
ps.selectPathPlanner ("VisibilityPrmPlanner")

ps.solve ()
ps.pathLength(0)
len(ps.getWaypoints (0))

cl = robot.client
import numpy as np

ps.addPathOptimizer('Prune')
ps.optimizePath (0)
ps.pathLength(ps.numberPaths()-1)
len(ps.getWaypoints (ps.numberPaths()-1))

ps.clearPathOptimizers()
ps.addPathOptimizer("GradientBased")
cl.problem.setAlphaInit (0.2)
ps.optimizePath (0)
ps.numberPaths()
ps.pathLength(ps.numberPaths()-1)
tGB = cl.problem.getTimeGB ()
timeValuesGB = cl.problem.getTimeValues ()
gainValuesGB = cl.problem.getGainValues ()
newGainValuesGB = ((1-np.array(gainValuesGB))*100).tolist() #percentage of initial length-value

ps.clearPathOptimizers()
ps.addPathOptimizer('PartialShortcut')
ps.optimizePath (0)
ps.pathLength(ps.numberPaths()-1)
timeValuesPRS = cl.problem.getTimeValues ()
gainValuesPRS = cl.problem.getGainValues ()
newGainValuesPRS = ((1-np.array(gainValuesPRS))*100).tolist()

ps.clearPathOptimizers()
ps.addPathOptimizer('RandomShortcut')
ps.optimizePath (0)
ps.pathLength(1)
timeValuesRS = cl.problem.getTimeValues ()
gainValuesRS = cl.problem.getGainValues ()
newGainValuesRS = ((1-np.array(gainValuesRS))*100).tolist()

ps.optimizePath (0)
ps.pathLength(ps.numberPaths()-1)
timeValuesRS2 = cl.problem.getTimeValues ()
gainValuesRS2 = cl.problem.getGainValues ()
newGainValuesRS2 = ((1-np.array(gainValuesRS2))*100).tolist()


pp(ps.numberPaths()-1)


## -------------------------------------
import matplotlib.pyplot as plt
from plotfunctions import optAndConcatenate, getValIndex, computeMeansVector, reducedVectors, curvPlot, curvSdPlot
# OPTIMIZE AND Concatenate RS PRS values:
globalTimeValuesRS = []; globalGainValuesRS = []
globalTimeValuesPRS = []; globalGainValuesPRS = []
nbOpt = 10 # number of launchs of RS and PRS
optAndConcatenate (cl, ps, 0, nbOpt, 'RandomShortcut', globalTimeValuesRS, globalGainValuesRS)
optAndConcatenate (cl, ps, 0, nbOpt, 'PartialShortcut', globalTimeValuesPRS, globalGainValuesPRS)

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
plt.axis([-.01, tGB+0.01, 0, 107])
plt.xlabel('t (s)'); plt.ylabel('Relative remaining length (%)')
vectorLengthGB = len (timeValuesGB)
plt.plot([0,tGB], [newGainValuesGB[vectorLengthGB-1],newGainValuesGB[vectorLengthGB-1]], 'b--')
plt.plot(0, 100, 'bo'); plt.plot([0,timeValuesGB[0]], [100,100], 'b', linewidth=1.5)
plt = curvSdPlot (plt, tGB, tReduceVectorRS, meanReduceVectorRS, sdReduceVectorRS, '0.55', 0.8, 0.002)
plt = curvSdPlot (plt, tGB, tReduceVectorPRS, meanReduceVectorPRS, sdReduceVectorPRS, '0.55', 0.8, 0.002)
plt = curvPlot (plt, tGB, timeValuesGB, newGainValuesGB, 'o', 'b', 1.5)
plt.plot([0,tReduceVectorRS[0]], [100,100], 'r', linewidth=1.1)
plt = curvPlot (plt, tGB, tReduceVectorRS, meanReduceVectorRS, '*', 'r', 1.5)
plt.plot([0,tReduceVectorPRS[0]], [100,100], 'g', linewidth=0.8)
plt = curvPlot (plt, tGB, tReduceVectorPRS, meanReduceVectorPRS, '+', 'g', 1.5)
plt.show()

# For different alpha_init
#tmax = max(max(tGB,tGB2),max(tGB3,tGB4)) # PB with tGB2
tmax = max(tGB,max(tGB3,tGB4))
plt.axis([-.004, tmax+0.004, 0, 102])
plt.xlabel('t (s)'); plt.ylabel('Relative remaining length (%)')
vectorLengthGB = len (timeValuesGB)
#plt.plot([0,tGB], [newGainValuesGB[vectorLengthGB-1],newGainValuesGB[vectorLengthGB-1]], 'b--')
plt.plot(0, 100, 'bo'); plt.plot([0,timeValuesGB[0]], [100,100], 'b', linewidth=1.5)
plt = curvPlot (plt, tmax, timeValuesGB, newGainValuesGB, 'o', 'b', 1.5)
#vectorLengthGB2 = len (timeValuesGB2)
#plt.plot(0, 100, 'g*'); plt.plot([0,timeValuesGB2[0]], [100,100], 'g', linewidth=1.5)
#plt = curvPlot (plt, tmax, timeValuesGB2, newGainValuesGB2, '*', 'g', 1.5)
vectorLengthGB3 = len (timeValuesGB3)
plt.plot(0, 100, 'r+'); plt.plot([0,timeValuesGB3[0]], [100,100], 'r', linewidth=1.5)
plt = curvPlot (plt, tmax, timeValuesGB3, newGainValuesGB3, '+', 'r', 1.5)
vectorLengthGB4 = len (timeValuesGB4)
plt.plot(0, 100, 'c+'); plt.plot([0,timeValuesGB4[0]], [100,100], 'c', linewidth=1.5)
plt = curvPlot (plt, tmax, timeValuesGB4, newGainValuesGB4, '+', 'c', 1.5)
plt.show()

## -------------------------------------

ps.clearPathOptimizers(); ps.addPathOptimizer("GradientBased")
cl.problem.setAlphaInit (0.05) # PROBLEM WITH (and without?) PRUNE !!
ps.optimizePath (0); tGB2 = cl.problem.getTimeGB ()
timeValuesGB2 = cl.problem.getTimeValues (); gainValuesGB2 = cl.problem.getGainValues ()
newGainValuesGB2 = ((1-np.array(gainValuesGB2))*100).tolist() #percentage of initial length-value

cl.problem.setAlphaInit (0.3)
ps.optimizePath (0); tGB3 = cl.problem.getTimeGB ()
timeValuesGB3 = cl.problem.getTimeValues (); gainValuesGB3 = cl.problem.getGainValues ()
newGainValuesGB3 = ((1-np.array(gainValuesGB3))*100).tolist() #percentage of initial length-value

cl.problem.setAlphaInit (0.4)
ps.optimizePath (0); tGB4 = cl.problem.getTimeGB ()
timeValuesGB4 = cl.problem.getTimeValues (); gainValuesGB4 = cl.problem.getGainValues ()
newGainValuesGB4 = ((1-np.array(gainValuesGB4))*100).tolist() #percentage of initial length-value

## -------------------------------------


# Add light to scene
lightName = "li4"
r.client.gui.addLight (lightName, r.windowId, 0.001, [0.4,0.4,0.4,0.5])
r.client.gui.addToGroup (lightName, r.sceneName)
#r.client.gui.applyConfiguration (lightName, [1,0,0,1,0,0,0])
#r.client.gui.applyConfiguration (lightName, [1,0,1,1,0,0,0])
r.client.gui.applyConfiguration (lightName, [6,0,1,1,0,0,0])
r.client.gui.refresh ()


## Video recording
import time
pp.dt = 0.02
pp.speed=0.5
r(q1)
r.startCapture ("capture","png")
r(q1); time.sleep(0.2)
r(q1)
pp(20)
#pp(ps.numberPaths()-1)
r(q2); time.sleep(1);
r.stopCapture ()

## ffmpeg commands
ffmpeg -r 50 -i capture_0_%d.png -r 25 -vcodec libx264 video.mp4
x=0; for i in *png; do counter=$(printf %04d $x); ln "$i" new"$counter".png; x=$(($x+1)); done
ffmpeg -r 50 -i new%04d.png -r 25 -vcodec libx264 video.mp4
mencoder video.mp4 -channels 6 -ovc xvid -xvidencopts fixed_quant=4 -vf harddup -oac pcm -o video.avi
ffmpeg -i untitled.mp4 -vcodec libx264 -crf 24 video.mp4


robot.getJointNames ()



## Debug Optimization Tools ##############
num_log = 22768
from parseLog import parseCollConstrPoints, parseNodes

collConstrNodes = parseNodes (num_log, '189: qFree_ = ')
collNodes = parseNodes (num_log, '182: qColl = ')

contactPoints = parseCollConstrPoints (num_log, '77: contact point = (')
x1_J1 = parseCollConstrPoints (num_log, '96: x1 in R0 = (')
x2_J1 = parseCollConstrPoints (num_log, '97: x2 in R0 = (')
x1_J2 = parseCollConstrPoints (num_log, '116: x1 in J2 = (')
x2_J2 = parseCollConstrPoints (num_log, '117: x2 in J2 = (') #x2_J2 <=> contactPoints


## same with viewer !
from viewer_display_library_OPTIM import transformInConfig, plotPoints, plotPointsAndLines, plot2DBaseCurvPath, plotDofCurvPath
contactPointsViewer = transformInConfig (contactPoints)
x1_J1Viewer = transformInConfig (x1_J1)
x2_J1Viewer = transformInConfig (x2_J1)
x1_J2Viewer = transformInConfig (x1_J2)
x2_J2Viewer = transformInConfig (x2_J2)


sphereNamePrefix = "sphereContactPoints_"
plotPoints (r, sphereNamePrefix, contactPointsViewer, 0.02)
sphereSize=0.01
lineNamePrefix = "lineJ1_"; sphereNamePrefix = "sphereJ1_"
plotPointsAndLines (r, lineNamePrefix, sphereNamePrefix, x1_J1Viewer, x2_J1Viewer, sphereSize)
lineNamePrefix = "lineJ2_"; sphereNamePrefix = "sphereJ2_"
plotPointsAndLines (r, lineNamePrefix, sphereNamePrefix, x1_J2Viewer, x2_J2Viewer, sphereSize)

from hpp.corbaserver import Client
cl = robot.client
dt = 0.2
plot2DBaseCurvPath(r, cl, dt, 0, "curvPath"+str(0), [1,0.3,0,1])
plot2DBaseCurvPath (r, cl, dt, ps.numberPaths()-1, "curvPath"+str(ps.numberPaths()-1), [0,1,0.3,1])

jointName = 'l_wrist_flex_joint'
plotDofCurvPath (r, cl, robot, dt, 0, jointName, 'path0_'+jointName, [1,0,0,1])
plotDofCurvPath (r, cl, robot, dt, ps.numberPaths()-1, jointName, 'path1_'+jointName, [0,0,1,1])

jointName = 'r_wrist_flex_joint'

from viewer_display_library_OPTIM import plotPointBodyCurvPath
from hpp.corbaserver import Client
cl = robot.client
dt = 0.06
jointName = 'l_wrist_flex_joint'
plotPointBodyCurvPath (r, cl, robot, dt, 0, jointName, [0,0.12,0], 'pathPoint_1'+jointName, [1,0.1,0.1,1])
plotPointBodyCurvPath (r, cl, robot, dt, 19, jointName, [0,0.12,0], 'pathPointRS_1'+jointName, [0.1,0.1,1,1])
plotPointBodyCurvPath (r, cl, robot, dt, 20, jointName, [0,0.12,0], 'pathPointPRS_1'+jointName, [0.1,0.4,0.5,1])
plotPointBodyCurvPath (r, cl, robot, dt, 18, jointName, [0,0.12,0], 'pathPointGB_1'+jointName, [0.2,0.9,0.2,1])

r.client.gui.removeFromGroup ('pathPointGB_'+jointName, r.sceneName)
r.client.gui.removeFromGroup ('pathPointRS_'+jointName, r.sceneName)
r.client.gui.removeFromGroup ('pathPointPRS_'+jointName, r.sceneName)


# test function in cl.robot
jointPosition = robot.getJointPosition ('l_wrist_flex_joint')
pointInJoint = [0,0.12,0]
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

