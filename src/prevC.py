"""Workbench preview of the chain station (side A) over one cycle. usage: python3 prevC.py OUTDIR N [K0 K1]"""
import sys, os, numpy as np
sys.path.insert(0, '/home/claude/scripts')
import bpy, mathutils
import line3, tline, cstation, lblend, cprev

out, N = sys.argv[1], int(sys.argv[2])
k0, k1 = (int(sys.argv[3]), int(sys.argv[4])) if len(sys.argv) > 4 else (0, N)
os.makedirs(out, exist_ok=True)
LDU = 0.0004
def bl(p): return np.array([p[0], -p[2], -p[1]])*LDU
TOT = sum(d for _, d in line3.PHASES)
U0 = (tline.T_LEAVE + tline.SEQ[-1][0]) % TOT                    # absolute time of u = 0 (build wagon leaves S4)
TS = [(U0 + TOT*(k + 0.5)/N) % TOT for k in range(N)]
G = tline.cart_G('SA', tline.s_S(tline.ZK[4] + tline.REFILL), 260.0)
def W(p): return (G @ np.array([*p, 1.0]))[:3]
REF = tline.REF

def caption(u, cs):
    if u < REF['back'][1]: return 'Supply train backs up: S5 passes, S4 stops at the chain station · the finished set waits on the flat table'
    if u < REF['up1'][1]: return 'Tilt hub turns the table upright: its stubs line up with S4\'s guides'
    if u < REF['push'][1]: return 'Boost-motor crank drives three rods: the drive set slides 38 off the stubs onto S4\'s guides'
    if u < REF['ret'][1]: return 'Rods back · S4 now carries the drive set (its pads push it home at the S4 station)'
    if u < REF['down1'][1]: return 'Table lies down again'
    if u < REF['fwd'][1]: return 'Train moves on one wagon: S5 with three loose sprockets stops at the station'
    if u < REF['up2'][1]: return 'Table upright again'
    if u < REF['s5ret'][1]: return 'Crank pusher slides S5\'s sprockets onto the stubs (they come to rest on the rod tips)'
    if u < REF['down2'][1]: return 'Train returns to the build stations · table lies down'
    if u < cstation.U_B: return 'Belt brings the queue of links back to the table edge'
    v = u - cstation.U_B
    j = int(v // cstation.TL)
    if j < 33: return f'Link {j + 1} of 34 snaps onto the tail · all three sprockets turn together (Technic chain under the table), guide posts keep the open chain on its path'
    if j < 34: return 'Link 34 closes the loop (closing mechanism still being designed: shown schematically)'
    return 'Chain closed · two more pitches so the sprocket axles line up for the next set'

def cam_for(u):
    # station centre (canonical) and a viewpoint on the TA side, high
    if REF['up1'][0] <= u < REF['s5ret'][1] + 0.5:          # upright work with the supply wagons: look along the table face
        tgt = W((-120, -250, 140)); pos = tgt + np.array([-650.0, -1000.0, -520.0]); lens = 36
    else:                                                     # flat-table chain building: from above the belt side
        tgt = W((-200, -180, -110)); pos = tgt + np.array([-420.0, -560.0, 380.0]); lens = 34
    return bl(pos), bl(tgt), lens

def frame_state(t):
    st = line3.cycle(t/TOT, 'builder'); st.update(s0=False, s2=False); return st

names = {}
for k in range(k0, k1):
    for p in line3.build(frame_state(TS[k])): names.setdefault(p['name'], dict(p))
sc, cam, objs = cprev.setup(list(names.values()), (960, 540))
sc.display.render_aa = 'FXAA'; cam.data.clip_start = 0.005
for k in range(k0, k1):
    t = TS[k]; st = frame_state(t); P = {p['name']: p for p in line3.build(st)}
    u = st['tl']['u_ref']; pos, tgt, lens = cam_for(u)
    for n, ob in objs.items():
        p = P.get(n); vis = p is not None
        if vis: vis = np.linalg.norm(bl(p['M'][:3, 3]) - tgt) < 1.2
        ob.hide_render = not vis
        if vis: ob.matrix_world = lblend.M_bl(p['M'])
    cam.location = mathutils.Vector(pos); cam.rotation_euler = (mathutils.Vector(tgt) - mathutils.Vector(pos)).to_track_quat('-Z', 'Y').to_euler()
    cam.data.lens = lens
    sc.render.filepath = f'{out}/f{k:04d}.png'; bpy.ops.render.render(write_still=True)
    with open(f'{out}/f{k:04d}.txt', 'w') as fh: fh.write(caption(u, cstation.state(u)))
    print('FRAME', k, flush=True)
