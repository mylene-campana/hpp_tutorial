import bpy

taggedObjects = list()
def tagObjects ():
  global taggedObjects
  taggedObjects = list ()
  for obj in bpy.data.objects:
    taggedObjects.append (obj.name)

def getNonTaggedObjects ():
  global taggedObjects
  return [obj for obj in bpy.data.objects if obj.name not in taggedObjects]

def setParent (children, parent):
  for child in children:
    child.parent = parent

tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/BODY.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/BODY_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/BODY"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.003]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/RLEG_LINK0.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/RLEG_LINK0_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/RLEG_LINK0"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/RLEG_LINK1.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/RLEG_LINK1_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/RLEG_LINK1"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/RLEG_LINK2.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/RLEG_LINK2_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/RLEG_LINK2"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/RLEG_LINK3.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/RLEG_LINK3_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/RLEG_LINK3"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/RLEG_LINK4.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/RLEG_LINK4_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/RLEG_LINK4"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/RLEG_LINK5.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/r_ankle_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/r_ankle"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/LLEG_LINK0.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/LLEG_LINK0_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/LLEG_LINK0"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/LLEG_LINK1.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/LLEG_LINK1_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/LLEG_LINK1"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/LLEG_LINK2.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/LLEG_LINK2_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/LLEG_LINK2"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/LLEG_LINK3.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/LLEG_LINK3_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/LLEG_LINK3"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/LLEG_LINK4.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/LLEG_LINK4_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/LLEG_LINK4"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/LLEG_LINK5.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/l_ankle_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/l_ankle"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/CHEST_LINK0.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/CHEST_LINK0_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/CHEST_LINK0"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/CHEST_LINK1.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/torso_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/torso"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/HEAD_LINK0.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/HEAD_LINK0_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/HEAD_LINK0"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/HEAD_LINK1.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/HEAD_LINK1_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/HEAD_LINK1"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/RARM_LINK0.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/RARM_LINK0_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/RARM_LINK0"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/RARM_LINK1.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/RARM_LINK1_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/RARM_LINK1"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/RARM_LINK2.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/RARM_LINK2_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/RARM_LINK2"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/RARM_LINK3.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/RARM_LINK3_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/RARM_LINK3"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/RARM_LINK4.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/RARM_LINK4_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/RARM_LINK4"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/RARM_LINK5_full.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/r_wrist_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/r_wrist"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/RARM_LINK6.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/RARM_LINK6_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/RARM_LINK6"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/RHAND_LINK0.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/RHAND_LINK0_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/RHAND_LINK0"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/RHAND_LINK1.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/RHAND_LINK1_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/RHAND_LINK1"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/RHAND_LINK2.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/RHAND_LINK2_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/RHAND_LINK2"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/RHAND_LINK3.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/RHAND_LINK3_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/RHAND_LINK3"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/RHAND_LINK4.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/RHAND_LINK4_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/RHAND_LINK4"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/LARM_LINK0.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/LARM_LINK0_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/LARM_LINK0"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/LARM_LINK1.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/LARM_LINK1_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/LARM_LINK1"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/LARM_LINK2.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/LARM_LINK2_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/LARM_LINK2"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/LARM_LINK3.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/LARM_LINK3_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/LARM_LINK3"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/LARM_LINK4.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/LARM_LINK4_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/LARM_LINK4"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/LARM_LINK5_full.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/l_wrist_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/l_wrist"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/LARM_LINK6.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/LARM_LINK6_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/LARM_LINK6"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 3.141592654]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/LHAND_LINK0.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/LHAND_LINK0_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/LHAND_LINK0"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 3.141592654]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/LHAND_LINK1.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/LHAND_LINK1_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/LHAND_LINK1"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 3.141592654]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/LHAND_LINK2.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/LHAND_LINK2_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/LHAND_LINK2"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 3.141592654]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/LHAND_LINK3.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/LHAND_LINK3_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/LHAND_LINK3"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 3.141592654]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hrp2_14_description/meshes/LHAND_LINK4.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "hrp2/LHAND_LINK4_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "hrp2/LHAND_LINK4"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 3.141592654]
