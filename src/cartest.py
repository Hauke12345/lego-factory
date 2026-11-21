import sys; sys.path.insert(0,'/home/claude/scripts')
import bpy, math
from mathutils import Vector
bpy.ops.wm.read_factory_settings(use_empty=True)
import lego
parts,wheels=lego.build_car()
for ob,M in parts: ob.matrix_world=M
sc=bpy.context.scene; sc.render.engine='CYCLES'; sc.cycles.samples=32; sc.cycles.use_denoising=True
sc.render.resolution_x=640; sc.render.resolution_y=400
w=bpy.data.worlds.new('w'); sc.world=w; w.use_nodes=True
env=bpy.data.images.load('/home/claude/hdri/env.exr'); t=w.node_tree.nodes.new('ShaderNodeTexEnvironment'); t.image=env
w.node_tree.links.new(t.outputs[0],w.node_tree.nodes['Background'].inputs[0])
cam=bpy.data.objects.new('cam',bpy.data.cameras.new('cam')); sc.collection.objects.link(cam); sc.camera=cam
cam.location=(0.17,0.13,0.10); d=Vector((0,0,0.015))-cam.location; cam.rotation_euler=d.to_track_quat('-Z','Y').to_euler(); cam.data.lens=50
bpy.ops.mesh.primitive_plane_add(size=1)
sc.render.filepath='/home/claude/test/car.png'; bpy.ops.render.render(write_still=True)
