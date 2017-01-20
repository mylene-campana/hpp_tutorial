#/usr/bin/env python
#blender/urdf_to_blender.py -p pr2/ -i /local/mcampana/devel/hpp/src/hpp_tutorial/urdf/pr2_full.urdf -o pr2_full_blend.py

from hpp.corbaserver import ProblemSolver
#from hpp.corbaserver.pr2 import Robot
from hpp.corbaserver.pr2_full import Robot

robot = Robot ('pr2') #35 DOF
robot.setJointBounds ("base_joint_xy", [-3, 3, -3, 3])
ps = ProblemSolver (robot)
cl = robot.client

"""
from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (robot.client, r)
r.loadObstacleModel ("iai_maps","kitchen_area","kitchen_area") # visual kitchen
r.loadObstacleModel ("iai_maps","old_floor","old_floor")
r.loadObstacleModel ("room_description","room","room")
"""

ps.loadObstacleFromUrdf ("iai_maps", "kitchen_area", ""); ps.loadObstacleFromUrdf ("iai_maps", "old_floor", "")
#ps.loadObstacleFromUrdf ("room_description", "room", "")

q0 = robot.getCurrentConfig ()

# TODO: select config sampler

# release, no log, no viewer
# with or without the kitchen or the room
# loop on N iterations, compute collision-checking and distances
# get times (in C++)

N = 1000
meanCollisionTimeVec = cl.problem.getMeanCollisionTimes(N)
meanCollisionTime = meanCollisionTimeVec[0]; meanCollisionValidConfigsPercent = meanCollisionTimeVec[1]
meanDistanceTimes = cl.problem.getMeanDistanceTimes(N)


print("meanCollisionTime= " + str(meanCollisionTime))
print("meanCollisionValidConfigsPercent= " + str(meanCollisionValidConfigsPercent))
print("meanDistanceTimes= " + str(meanDistanceTimes))
