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

mat = bpy.data.materials.new("Blue")
mat.diffuse_color = [0.0, 0.0, 0.8]
mat.alpha = 1.0
mat = bpy.data.materials.new("Green")
mat.diffuse_color = [0.0, 0.8, 0.0]
mat.alpha = 1.0
mat = bpy.data.materials.new("Grey")
mat.diffuse_color = [0.7, 0.7, 0.7]
mat.alpha = 1.0
mat = bpy.data.materials.new("Grey2")
mat.diffuse_color = [0.9, 0.9, 0.9]
mat.alpha = 1.0
mat = bpy.data.materials.new("Red")
mat.diffuse_color = [0.8, 0.0, 0.0]
mat.alpha = 1.0
mat = bpy.data.materials.new("White")
mat.diffuse_color = [1.0, 1.0, 1.0]
mat.alpha = 1.0
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/base_v0/base.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/base_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/base_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
bpy.ops.mesh.primitive_cube_add ()
currentObj = bpy.context.object
currentObj.dimensions = [0.01, 0.01, 0.01]
bpy.context.object.name = "pr2/base_footprint_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/base_footprint"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
bpy.ops.mesh.primitive_cube_add ()
currentObj = bpy.context.object
currentObj.dimensions = [0.001, 0.001, 0.001]
bpy.context.object.name = "pr2/base_laser_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Red"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/base_laser_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/base_v0/caster.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/fl_caster_rotation_link_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/fl_caster_rotation_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/base_v0/caster.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/fr_caster_rotation_link_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/fr_caster_rotation_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/base_v0/caster.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/bl_caster_rotation_link_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/bl_caster_rotation_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/base_v0/caster.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/br_caster_rotation_link_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/br_caster_rotation_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/torso_v0/torso_lift.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/torso_lift_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Grey2"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/torso_lift_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
bpy.ops.mesh.primitive_cube_add ()
currentObj = bpy.context.object
currentObj.dimensions = [0.001, 0.001, 0.001]
bpy.context.object.name = "pr2/imu_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Red"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/imu_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/head_v0/head_pan.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/head_pan_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Blue"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/head_pan_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/head_v0/head_tilt.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/head_tilt_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Green"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/head_tilt_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
bpy.ops.mesh.primitive_cube_add ()
currentObj = bpy.context.object
currentObj.dimensions = [0.01, 0.01, 0.01]
bpy.context.object.name = "pr2/head_plate_frame_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Blue"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/head_plate_frame"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
bpy.ops.mesh.primitive_cube_add ()
currentObj = bpy.context.object
currentObj.dimensions = [0.01, 0.01, 0.01]
bpy.context.object.name = "pr2/sensor_mount_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Blue"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/sensor_mount_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
bpy.ops.mesh.primitive_cube_add ()
currentObj = bpy.context.object
currentObj.dimensions = [0.04, 0.04, 0.04]
bpy.context.object.name = "pr2/high_def_frame_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Blue"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/high_def_frame"
currentObj.parent = empty
currentObj.location = [-0.02, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
bpy.ops.mesh.primitive_cylinder_add (radius=0.02, depth=0.05)
currentObj = bpy.context.object
bpy.context.object.name = "pr2/high_def_optical_frame_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Blue"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/high_def_optical_frame"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 1.57079632679]
bpy.ops.mesh.primitive_cube_add ()
currentObj = bpy.context.object
currentObj.dimensions = [0.02, 0.12, 0.05]
bpy.context.object.name = "pr2/double_stereo_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Blue"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/double_stereo_link"
currentObj.parent = empty
currentObj.location = [-0.01, 0.0, 0.025]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
bpy.ops.mesh.primitive_cube_add ()
currentObj = bpy.context.object
currentObj.dimensions = [0.001, 0.001, 0.001]
bpy.context.object.name = "pr2/wide_stereo_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Blue"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/wide_stereo_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
bpy.ops.mesh.primitive_cube_add ()
currentObj = bpy.context.object
currentObj.dimensions = [0.01, 0.01, 0.01]
bpy.context.object.name = "pr2/wide_stereo_gazebo_l_stereo_camera_frame_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/wide_stereo_gazebo_l_stereo_camera_frame"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
bpy.ops.mesh.primitive_cube_add ()
currentObj = bpy.context.object
currentObj.dimensions = [0.01, 0.01, 0.01]
bpy.context.object.name = "pr2/wide_stereo_gazebo_r_stereo_camera_frame_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/wide_stereo_gazebo_r_stereo_camera_frame"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
bpy.ops.mesh.primitive_cube_add ()
currentObj = bpy.context.object
currentObj.dimensions = [0.001, 0.001, 0.001]
bpy.context.object.name = "pr2/narrow_stereo_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Blue"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/narrow_stereo_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
bpy.ops.mesh.primitive_cube_add ()
currentObj = bpy.context.object
currentObj.dimensions = [0.01, 0.01, 0.01]
bpy.context.object.name = "pr2/narrow_stereo_gazebo_l_stereo_camera_frame_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/narrow_stereo_gazebo_l_stereo_camera_frame"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
bpy.ops.mesh.primitive_cube_add ()
currentObj = bpy.context.object
currentObj.dimensions = [0.01, 0.01, 0.01]
bpy.context.object.name = "pr2/narrow_stereo_gazebo_r_stereo_camera_frame_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/narrow_stereo_gazebo_r_stereo_camera_frame"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/tilting_laser_v0/tilting_hokuyo.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/laser_tilt_mount_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Red"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/laser_tilt_mount_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
bpy.ops.mesh.primitive_cube_add ()
currentObj = bpy.context.object
currentObj.dimensions = [0.001, 0.001, 0.001]
bpy.context.object.name = "pr2/laser_tilt_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Red"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/laser_tilt_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/shoulder_v0/shoulder_pan.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/r_shoulder_pan_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Blue"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/r_shoulder_pan_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/shoulder_v0/shoulder_lift.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/r_shoulder_lift_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Grey"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/r_shoulder_lift_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/shoulder_v0/upper_arm_roll.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/r_upper_arm_roll_link_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/r_upper_arm_roll_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/upper_arm_v0/upper_arm.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/r_upper_arm_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Green"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/r_upper_arm_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/upper_arm_v0/forearm_roll.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/r_forearm_roll_link_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/r_forearm_roll_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/upper_arm_v0/elbow_flex.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/r_elbow_flex_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Grey"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/r_elbow_flex_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/forearm_v0/forearm.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/r_forearm_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Grey"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/r_forearm_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/forearm_v0/wrist_flex.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/r_wrist_flex_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Grey"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/r_wrist_flex_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/forearm_v0/wrist_roll.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/r_wrist_roll_link_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/r_wrist_roll_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/gripper_v0/gripper_palm.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/r_gripper_palm_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Red"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/r_gripper_palm_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
bpy.ops.mesh.primitive_cube_add ()
currentObj = bpy.context.object
currentObj.dimensions = [0.001, 0.001, 0.001]
bpy.context.object.name = "pr2/r_gripper_motor_accelerometer_link_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/r_gripper_motor_accelerometer_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/gripper_v0/l_finger.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/r_gripper_l_finger_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Grey"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/r_gripper_l_finger_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/gripper_v0/l_finger.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/r_gripper_r_finger_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Grey"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/r_gripper_r_finger_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [3.14159265359, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/gripper_v0/l_finger_tip.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/r_gripper_l_finger_tip_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Green"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/r_gripper_l_finger_tip_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/gripper_v0/l_finger_tip.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/r_gripper_r_finger_tip_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Green"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/r_gripper_r_finger_tip_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [3.14159265359, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/shoulder_v0/shoulder_pan.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/l_shoulder_pan_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Blue"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/l_shoulder_pan_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/shoulder_v0/shoulder_lift.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/l_shoulder_lift_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Grey"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/l_shoulder_lift_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/shoulder_v0/upper_arm_roll.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/l_upper_arm_roll_link_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/l_upper_arm_roll_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/upper_arm_v0/upper_arm.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/l_upper_arm_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Green"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/l_upper_arm_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/upper_arm_v0/forearm_roll.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/l_forearm_roll_link_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/l_forearm_roll_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/upper_arm_v0/elbow_flex.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/l_elbow_flex_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Grey"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/l_elbow_flex_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/forearm_v0/forearm.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/l_forearm_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Grey"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/l_forearm_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/forearm_v0/wrist_flex.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/l_wrist_flex_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Grey"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/l_wrist_flex_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/forearm_v0/wrist_roll.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/l_wrist_roll_link_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/l_wrist_roll_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/gripper_v0/gripper_palm.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/l_gripper_palm_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Red"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/l_gripper_palm_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
bpy.ops.mesh.primitive_cube_add ()
currentObj = bpy.context.object
currentObj.dimensions = [0.001, 0.001, 0.001]
bpy.context.object.name = "pr2/l_gripper_motor_accelerometer_link_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/l_gripper_motor_accelerometer_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/gripper_v0/l_finger.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/l_gripper_l_finger_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Grey"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/l_gripper_l_finger_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/gripper_v0/l_finger.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/l_gripper_r_finger_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Grey"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/l_gripper_r_finger_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [3.14159265359, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/gripper_v0/l_finger_tip.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/l_gripper_l_finger_tip_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Green"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/l_gripper_l_finger_tip_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/pr2_description/meshes/gripper_v0/l_finger_tip.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "pr2/l_gripper_r_finger_tip_link_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Green"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/l_gripper_r_finger_tip_link"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [3.14159265359, 0.0, 0.0]
bpy.ops.mesh.primitive_cube_add ()
currentObj = bpy.context.object
currentObj.dimensions = [0.01, 0.01, 0.01]
bpy.context.object.name = "pr2/l_forearm_cam_frame_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/l_forearm_cam_frame"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
bpy.ops.mesh.primitive_cube_add ()
currentObj = bpy.context.object
currentObj.dimensions = [0.01, 0.01, 0.01]
bpy.context.object.name = "pr2/r_forearm_cam_frame_visual0"
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "pr2/r_forearm_cam_frame"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
