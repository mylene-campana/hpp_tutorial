#/usr/bin/env python
#blender/urdf_to_blender.py -p pr2/ -i /local/mcampana/devel/hpp/src/hpp_tutorial/urdf/pr2_full.urdf -o pr2_full_blend.py

from hpp.corbaserver import ProblemSolver
#from hpp.corbaserver.pr2 import Robot
from hpp.corbaserver.pr2_full import Robot
robot = Robot ('pr2') #35 DOF
robot.setJointBounds ("base_joint_xy", [-4, -3, -5, -3])
ps = ProblemSolver (robot)

from hpp.gepetto import Viewer, PathPlayer
#Viewer.withFloor = True
r = Viewer (ps)

pp = PathPlayer (robot.client, r)
r.loadObstacleModel ("iai_maps","kitchen_area","kitchen_area") # visual kitchen
r.loadObstacleModel ("iai_maps","old_floor","old_floor")

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

lightName = "li"
r.client.gui.addLight (lightName, r.windowId, 0.005, [0.5,0.5,0.5,0.5])
r.client.gui.addToGroup (lightName, r.sceneName)
r.client.gui.applyConfiguration (lightName, [-5,-6,2,1,0,0,0])
r.client.gui.refresh ()

lightName = "li2"
r.client.gui.addLight (lightName, r.windowId, 0.005, [0.5,0.5,0.5,0.5])
r.client.gui.addToGroup (lightName, r.sceneName)
r.client.gui.applyConfiguration (lightName, [-2,-4,3,1,0,0,0])
r.client.gui.applyConfiguration (lightName, [-4,-4,5,1,0,0,0])
r.client.gui.refresh ()


## CONSTRAINTS ##
# "torso_lift_joint" : do not constraint z axis because different for q_init and q_goal
# constraint torso to be at position [-3.25, -4.0, 0.790675] in workspace frame
#ps.createPositionConstraint ("posConstraint", "torso_lift_joint", "", [0,0,0], [-3.25, -4.0, 0.790675], [1,1,0])
#ps.setNumericalConstraints ("constraints", ["posConstraint"])

ps.setInitialConfig (q_init); ps.addGoalConfig (q_goal)

#ps.selectPathValidation ("Dichotomy", 0.)
ps.selectPathPlanner ("VisibilityPrmPlanner")
#ps.readRoadmap ('/local/mcampana/devel/hpp/data/pr2-in-kitchen.rdm')
ps.readRoadmap ('/local/mcampana/devel/hpp/data/pr2-in-kitchen-PRM.rdm')
ps.solve ()
ps.pathLength(0)
len(ps.getWaypoints (0))
# ps.interruptPathPlanning ()

cl = robot.client
import numpy as np
"""
ps.addPathOptimizer("Prune")
ps.optimizePath (0)
ps.numberPaths()
ps.pathLength(ps.numberPaths()-1)
len(ps.getWaypoints (ps.numberPaths()-1))
"""
ps.clearPathOptimizers()
cl.problem.setAlphaInit (0.2)
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
plt.axis([-.2, tGB+0.2, 12, 103])
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
tmax = max(tGB,max(tGB3,tGB4))
plt.axis([-.5, tmax+0.5, 5, 101])
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

ps.clearPathOptimizers(); ps.addPathOptimizer("GradientBased") # PROBLEM WITH PRUNE
cl.problem.setAlphaInit (0.05)
ps.optimizePath (1); tGB2 = cl.problem.getTimeGB ()
timeValuesGB2 = cl.problem.getTimeValues (); gainValuesGB2 = cl.problem.getGainValues ()
newGainValuesGB2 = ((1-np.array(gainValuesGB2))*100).tolist() #percentage of initial length-value

cl.problem.setAlphaInit (0.3)
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
r.client.gui.addLight (lightName, r.windowId, 0.005, [0.5,0.5,0.5,0.5])
r.client.gui.addToGroup (lightName, r.sceneName)
#r.client.gui.applyConfiguration (lightName, [-2,-4.5,2,1,0,0,0])
#r.client.gui.applyConfiguration (lightName, [-4,-4,2,1,0,0,0]) # second light
r.client.gui.applyConfiguration (lightName, [-5,-6,2,1,0,0,0]) # second light
r.client.gui.refresh ()

## Video recording
import time
pp.dt = 0.02
pp.speed=0.6
r(q_init)
r.startCapture ("capture","png")
r(q_init); time.sleep(0.2)
r(q_init)
pp(0)
r(q_goal); time.sleep(1);
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
r.client.gui.removeFromGroup ("sphereContactPoints_1", r.sceneName)


## Debug Optimization Tools ##############
num_log = 28777
from parseLog import parseCollConstrPoints, parseNodes
from mutable_trajectory_plot import planarPlot, addNodePlot, addCircleNodePlot, addNodeAndLinePlot, addCorbaPathPlot

collConstrNodes = parseNodes (num_log, '189: qFree_ = ')
collNodes = parseNodes (num_log, '182: qColl = ')


contactPoints = parseCollConstrPoints (num_log, '77: contact point = (')
x1_J1 = parseCollConstrPoints (num_log, '96: x1 in R0 = (')
x2_J1 = parseCollConstrPoints (num_log, '97: x2 in R0 = (')
x1_J2 = parseCollConstrPoints (num_log, '116: x1 in J2 = (')
x2_J2 = parseCollConstrPoints (num_log, '117: x2 in J2 = (') #x2_J2 <=> contactPoints


## same with viewer !
from viewer_display_library_OPTIM import transformInConfig, plotPoints, plotPointsAndLines, plot2DBaseCurvPath
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
plot2DBaseCurvPath (r, cl, dt, 0, "curvPath"+str(0), [1,0.3,0,1])
plot2DBaseCurvPath (r, cl, dt, ps.numberPaths()-1, "curvPath"+str(ps.numberPaths()-1), [0,1,0.3,1])


from viewer_display_library_OPTIM import plotPointBodyCurvPath
from hpp.corbaserver import Client
cl = robot.client
dt = 0.01
jointName = 'r_wrist_flex_joint'
plotPointBodyCurvPath (r, cl, robot, dt, 0, jointName, [0,0.12,0], 'pathPoint2_'+jointName, [1,0.1,0.1,1])
plotPointBodyCurvPath (r, cl, robot, dt, 33, jointName, [0,0.12,0], 'pathPointGB2_'+jointName, [0,0,1,1])

#plotPointBodyCurvPath (r, cl, robot, dt, 35, jointName, [0,0.12,0], 'pathPointRS_'+jointName, [0.1,0.1,1,1])
#plotPointBodyCurvPath (r, cl, robot, dt, 2, jointName, [0,0.12,0], 'pathPointPRS_'+jointName, [0.1,0.4,0.5,1])
#plotPointBodyCurvPath (r, cl, robot, dt, 34, jointName, [0,0.12,0], 'pathPointGB_'+jointName, [0.1,1,0.1,1])

jointName = 'base_joint_xy'
plotPointBodyCurvPath (r, cl, robot, dt, 0, jointName, [0,0,0.1], 'pathPoint_'+str(0)+jointName, [1,0.1,0.1,1])
plotPointBodyCurvPath (r, cl, robot, dt, ps.numberPaths ()-1, jointName, [0,0,0.1], 'pathPointRS_'+str(1)+jointName, [0.1,0.1,1,1])
plotPointBodyCurvPath (r, cl, robot, dt, ps.numberPaths ()-1, jointName, [0,0,0.1], 'pathPointGB_'+str(ps.numberPaths ()-1)+jointName, [0.2,1,0.2,1])


r.client.gui.removeFromGroup ('pathPointGB2_'+jointName, r.sceneName)
r.client.gui.setVisibility('kitchen_area/counter_top_island_link',"OFF")
r.client.gui.setVisibility('kitchen_area/white_counter_top_island_link',"OFF")


gui.writeNodeFile('kitchen_node?','kitchen.obj')

## IMPORT SCENE AND CONFIGS TO BLENDER ##
#gui.removeFromGroup("path0",r.sceneName)
#gui.getNodeList()
#gui.createGroup ('sphere_wp0_group')
#gui.getGroupNodeList ('sphere_wp0_group')
blender/urdf_to_blender.py -p pr2/ -i /local/mcampana/devel/hpp/src/hpp_tutorial/blender/bizu.urdf -o bizu_blend.py # generate robot loadable by Blender

gui = r.client.gui
#pathId = 0; dt = 0.02; gui.setCaptureTransform ("pr2_initial_traj.yaml", ["pr2"])
pathId = 33; dt = 0.005; gui.setCaptureTransform ("pr2_optim_traj.yaml", ["pr2"])
PL = ps.pathLength(pathId)
FrameRange = np.arange(0,PL,dt)
numberFrame = len(FrameRange)

#r (q_init); robot.setCurrentConfig(q_init); gui.refresh (); gui.captureTransform ()
#r (q_goal); robot.setCurrentConfig(q_goal); gui.refresh (); gui.captureTransform ()

for t in FrameRange:
        q = ps.configAtParam (pathId, t)#update robot configuration
        r (q); robot.setCurrentConfig(q)
        gui.refresh (); gui.captureTransform ()

r (q_goal); robot.setCurrentConfig(q_goal); gui.refresh (); gui.captureTransform ()


