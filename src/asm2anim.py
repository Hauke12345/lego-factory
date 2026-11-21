import sys, os
sys.path.insert(0,'/home/claude/scripts')
import bpy, asm2, cprev, lblend
out=sys.argv[1]; N=int(sys.argv[2]); k0=int(sys.argv[3]); k1=int(sys.argv[4])
os.makedirs(out,exist_ok=True)
union={}
for t in (0.1,0.3,0.5,0.9):
    for p in asm2.build(asm2.cycle(t)): union.setdefault(p['name'],p)
sc,cam,objs=cprev.setup(list(union.values()),(1280,720))
cprev.aim(cam,((0.40,0.50,0.40),(-0.02,0.0,0.07),32))
sc.display.render_aa='FXAA'
for k in range(k0,k1):
    P={p['name']:p for p in asm2.build(asm2.cycle(k/N))}
    for n,ob in objs.items():
        if n in P: ob.hide_render=False; ob.matrix_world=lblend.M_bl(P[n]['M'])
        else: ob.hide_render=True
    sc.render.filepath=f'{out}/f{k:04d}.png'; bpy.ops.render.render(write_still=True); print('FRAME',k,flush=True)
