#/usr/bin/env python
#blender/urdf_to_blender.py -p pr2/ -i /local/mcampana/devel/hpp/src/hpp_tutorial/urdf/pr2_full.urdf -o pr2_full_blend.py

from hpp.corbaserver import ProblemSolver
from hpp.corbaserver.pr2 import Robot
robot = Robot ('pr2') #35 DOF
robot.setJointBounds ("base_joint_xy", [-4, -3, -5, -3])
ps = ProblemSolver (robot)

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps); cl = robot.client

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


ps.setInitialConfig (q_init); ps.addGoalConfig (q_goal)

ps.selectPathPlanner ("VisibilityPrmPlanner")
ps.solve ()
ps.pathLength(0)
len(ps.getWaypoints (0))
pp(0)




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




ps.clearPathOptimizers()
ps.addPathOptimizer('PartialShortcut')
ps.optimizePath (0)
ps.pathLength(ps.numberPaths()-1)
pp(ps.numberPaths()-1)



