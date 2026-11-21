import sys; sys.path.insert(0, '/home/claude/scripts')
import bpy, numpy as np, cprev
def T(x=0,y=0,z=0):
    M=np.eye(4); M[:3,3]=[x,y,z]; return M
P=[dict(name='hub', part='26910', col=71, M=T()), dict(name='plug', part='26913c01', col=71, M=T(200,0,0))]
sc, cam, _ = cprev.setup(P, (1100, 800))
for nm, v in [('top', ((0.0, -0.01, 0.10), (0, -0.008, 0.03), 35)), ('side', ((0.09, -0.06, 0.06), (0.0, -0.01, 0.02), 30))]:
    cprev.aim(cam, v); sc.render.filepath=f'/home/claude/factory/prev/hub_{nm}.png'; bpy.ops.render.render(write_still=True)
