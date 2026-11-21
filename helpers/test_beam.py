import sys; sys.path.insert(0, '/home/claude/scripts')
import bpy, numpy as np, lblend, mathutils, cprev
P=[dict(name='b', part='32278', col=72, M=np.eye(4)), dict(name='p', part='2780', col=0, M=np.array([[1,0,0,40],[0,1,0,0],[0,0,1,140],[0,0,0,1.]])),
   dict(name='t', part='3701', col=1, M=np.array([[1,0,0,0],[0,1,0,-40],[0,0,1,-60],[0,0,0,1.]]))]
sc, cam, _ = cprev.setup(P, (900, 700))
cprev.aim(cam, ((0.03, -0.02, 0.03), (0.0, -0.045, 0.0), 50))
sc.render.filepath='/home/claude/factory/prev/beam_test.png'; bpy.ops.render.render(write_still=True)
