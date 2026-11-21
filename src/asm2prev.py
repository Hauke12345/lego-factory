import sys, numpy as np
sys.path.insert(0,'/home/claude/scripts')
import bpy, asm2, cprev
out='/home/claude/factory/prev/'
for nm,t in (('rest',0.26),('seated',0.45)):
    P=asm2.build(asm2.cycle(t))
    sc,cam,_=cprev.setup(P,(1600,1000))
    for v,view in (('iso',((0.42,0.52,0.42),(-0.02,0.0,0.07),32)),('side',((-0.03,0.30,0.16),(-0.03,0.0,0.09),30)),('top',((0.0,0.001,0.95),(0.0,0.0,0.0),35))):
        cprev.aim(cam,view); sc.render.filepath=f'{out}asm2_{nm}_{v}.png'; bpy.ops.render.render(write_still=True)
