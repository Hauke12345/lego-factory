import sys, os, numpy as np
sys.path.insert(0,'/home/claude/scripts')
import bpy, line2, cprev, lblend
out=sys.argv[1]; N=int(sys.argv[2]); k0=int(sys.argv[3]); k1=int(sys.argv[4])
os.makedirs(out,exist_ok=True)
union={}
for t in np.linspace(0.001,0.999,90):
    for p in line2.build(line2.cycle(t)): union.setdefault(p['name'],p)
sc,cam,objs=cprev.setup(list(union.values()),(1280,720)); sc.display.render_aa='FXAA'
def focus(st):
    n=st['phase']
    if n in ('karrive','kpush','kretract'): return line2.PXK
    if n=='travel1': return st['px']
    if n in ('push','retract','sleds'): return 0.0
    if n=='travel': return st['px']*(1+150/1640)
    return line2.PXE-150-0.75*st['drive']
for k in range(k0,k1):
    st=line2.cycle(k/N); fx=focus(st)
    P={p['name']:p for p in line2.build(st)}
    for n,ob in objs.items():
        if n in P and abs(P[n]['M'][0,3]-fx)<1500: ob.hide_render=False; ob.matrix_world=lblend.M_bl(P[n]['M'])
        else: ob.hide_render=True
    f=fx*0.0004
    cprev.aim(cam,((f+0.34,0.56,0.44),(f-0.02,0.0,0.08),32))
    sc.render.filepath=f'{out}/f{k:04d}.png'; bpy.ops.render.render(write_still=True); print('FRAME',k,flush=True)
