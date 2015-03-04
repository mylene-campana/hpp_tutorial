#/usr/bin/env python

import time
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver.pr2 import Robot
robot = Robot ('pr2')
robot.setJointBounds ("base_joint_xy", [-4, -3, -5, -3])
ps = ProblemSolver (robot)

from hpp.gepetto import Viewer, PathPlayer
Viewer.withFloor = True
r = Viewer (ps)
pp = PathPlayer (robot.client, r)
r.loadObstacleModel ("iai_maps","kitchen_area","kitchen_area") # visual kitchen

q_init = robot.getCurrentConfig ()
q_goal = q_init [::]
q_init [0:2] = [-3.2, -4]
rank = robot.rankInConfiguration ['torso_lift_joint']
q_init [rank] = 0.2
r (q_init)

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

## CONSTRAINTS ##
# "torso_lift_joint" : do not constraint z axis because different for q_init and q_goal
# constraint torso to be at position [-3.25, -4.0, 0.790675] in workspace frame
#ps.createPositionConstraint ("posConstraint", "torso_lift_joint", "", [0,0,0], [-3.25, -4.0, 0.790675], [1,1,0])
#ps.setNumericalConstraints ("constraints", ["posConstraint"])


ps.loadObstacleFromUrdf ("iai_maps", "kitchen_area", "") # environment
ps.setInitialConfig (q_init); ps.addGoalConfig (q_goal)

begin=time.time()
ps.solve ()
end=time.time()
print "Solving time: "+str(end-begin)

begin=time.time()
ps.optimizePath(0)
end=time.time()
print "Optim time: "+str(end-begin)
cl = robot.client
cl.problem.getIterationNumber ()

begin=time.time()
ps.optimizePath(1)
end=time.time()
print "Optim2 time: "+str(end-begin)
cl.problem.getIterationNumber ()

len(ps.getWaypoints (0))
ps.pathLength(0)
ps.pathLength(1)
ps.pathLength(2)

pp (0)
pp (1)

robot.getJointNames ()
robot.getJointPosition ("torso_lift_joint")
r(ps.configAtParam(0,5))

