"""Shoulder station: the standing robot (already on its own tracks) gets its upper-left module fitted from the side.

After the stand-up flap the robot drives on along the exit rails and stops by its own motor encoders (the Move Hub's
built-in motors are tacho motors) with the left shoulder in front of the station. A train on lane UA brings the module on a
tall carrier: two brick walls with Technic bricks hold two fixed guide axles through the module's free kit-brick holes and two
plungers joined by a 9L yoke: the upper one pushes the kit brick (pad on the lowest shoulder pin), the lower one carries the
plug pusher (a cantilevered 1x2 tile behind plug C, under its cable). A crank pusher on a pedestal presses the yoke;
module, plug and cable go home together (56 LDU), then the plungers come back with the crank and the train leaves.

World: LDraw LDU, -y up. Robot faces -x, its left side (and the station) towards +z.
"""
import numpy as np
import asm2, rails, mods, lcheck, worker2
from asm2 import T, R, col3, add, P, LBG, DBG, BLK, WHT, RED, YEL, BLU, ORG

RAIL = -120.0
TR = 56.0
ZW = (210.0, 270.0)                      # inner / outer wall centres (z)
ZY = 370.0                               # yoke centre at full stroke (outer face 380 = pusher face at theta 0)
DY_PUSH = 144.0                          # crank pusher raised six bricks: its face meets the push arm on the yoke
XM_OFF = -60.0                           # pusher downstream of the plungers: the arriving wagon never passes it, its front column clears the plug plunger
ZM = 570.0
lcheck.RESTING.update({'umod': 'hangs on the guide axles', 'uplug': 'rides on the plug pusher', 'usled': 'rolls on the rails',
                       'uplung': 'slides in the carrier walls', 'upusherA': 'slides on the ledge'})

# ---------------------------------------------------------------- robot height on the rails (rests on its lowest link edge)
_C = np.array([[x, -8, z, 1] for x in (-26, 26) for z in (-1, 11)], float).T
def lowest(phase):
    Q = []; worker2.build(M0=np.eye(4), tag='t', body='t', P=Q, modules={'trackL'}, track_phase=phase)
    return max((q['M'] @ _C)[1].max() for q in Q if q['part'] == '3873')
def robot_M(rx, phase):
    """robot frame -> world for the robot standing on the exit rails at x = rx."""
    return T(rx, RAIL - lowest(phase), 0) @ R('y', 90)

# ---------------------------------------------------------------- carrier (built at lane-straight x = sx, the robot's x)
def wall_stack():
    """layers from the deck top (-92) upwards: (part, height); Technic bricks where an axle passes."""
    L = [('3010', 24)]*8 + [('3701', 24)] + [('3010', 24)]*5 + [('B', 8)]*2 + [('3701', 24)] + [('3010', 24)] + [('B', 8)]*4 + \
        [('3701', 24)] + [('B', 8)]*2 + [('3701', 24)]
    return L

def carrier(sx, e, body='usled', pb='uplung'):
    a = lambda n, p, c, M, b=body: add(f'uc_{n}', p, c, M, b)
    a('base', '3027', LBG, T(sx, -92, 260))
    y = -92.0; holes = []
    for k, (part, h) in enumerate(wall_stack()):
        top = y - h
        if part == 'B': a(f'br{k}', '3031', DBG, T(sx, top, 240))                                 # 4x4 bridge across both walls
        else:
            for z in ZW: a(f'w{k}_{z:.0f}', part, (YEL if part == '3701' else (DBG if k % 2 else LBG)), T(sx, top, z))
            if part == '3701': holes.append(top + 10)
        y = top
    hp, hm, hg1, hg2 = holes                                   # plug plunger, module plunger, guides
    for i, yg in enumerate((hg1, hg2)):
        a(f'guide{i}', '3708', LBG, T(sx, yg, 204) @ R('y', 90))                                   # tip at z 84
        for z in (225, 255): a(f'gb{i}_{z}', '32123a', YEL, T(sx, yg, z))
    # plungers + yoke (move with e)
    f = lambda n, p, c, M: a(n, p, c, M, pb)
    f('pl_mod', '50451', BLK, T(sx, hm, 240 + e) @ R('y', 90))                                     # 80 .. 400
    f('pl_mod_pad', '32123a', RED, T(sx, hm, 85 + e))
    f('pl_plug', '50451', BLK, T(sx, hp, 240 + e) @ R('y', 90))
    f('yoke', '40490', RED, T(sx, (hp + hm)/2, ZY + e) @ col3((-1, 0, 0), (0, 0, 1), (0, 1, 0)))
    for yy in (hp, hm):
        for z in (ZY - 15, ZY + 15): f(f'yb{yy:.0f}_{z:.0f}', '32123a', YEL, T(sx, yy, z + e))
    # push arm: 3x5 L-liftarm behind the yoke, short leg pinned into the yoke's middle holes, long leg out to the pusher face
    yc = (hp + hm)/2 - 20                                                                          # corner hole; short leg up
    f('parm', '32526', RED, T(sx, yc, ZY + 20 + e) @ col3((0, -1, 0), (0, 0, 1), (-1, 0, 0)))
    for yy in (yc, yc - 20): f(f'parm_pin{yy:.0f}', '2780', BLK, T(sx, yy, ZY + 10 + e) @ R('y', 90))
    # plug pusher: 1x2 axle-hole brick on the plunger tip, 1x4 plate reaching out beside the track beam, 1x2 tile behind the plug
    f('pp_brick', '32064b', DBG, T(sx, hp - 10, 90 + e))
    f('pp_plate', '3710', DBG, T(sx - 40, hp - 18, 90 + e))
    f('pp_tile', '3069b', RED, T(sx - 50, hp - 26, 80 + e) @ R('y', 90))
    return holes

def module(Mr, e, hook=mods.SH_UP, body='umod', plug_body='uplug'):
    G = Mr @ T(-e, 0, 0)
    def a(n, p, c, M, b=body): add(f'um_{n}', p, c, G @ M, plug_body if n == 'plugC' else b)
    def cab(n, part, ends): P.append(dict(name=f'um_{n}', part=part, col=0, M=G.copy(), body=body, ends=(f'um_{ends[0]}', f'um_{ends[1]}')))
    mods.shook_module(a, cab, hook=hook, body_hook=body)

# ---------------------------------------------------------------- raised crank pusher
def pusher(theta, XM, ZM=ZM, side='A'):
    n0 = len(P); asm2.pusher(side, theta, XM=XM, ZM=ZM, tag='U')
    new = P[n0:]
    base = [p for p in new if p['body'] == 'frame' and abs(p['M'][1, 3] + 24) < 0.5 and p['part'] in ('2465', '3010')]
    for p in new: p['M'] = T(0, -DY_PUSH, 0) @ p['M']
    for p in base:                                                    # pedestal: the lowest layer repeated six times underneath
        for k in range(int(DY_PUSH//24)): add(p['name'] + f'_ped{k}', p['part'], DBG if k % 2 else LBG, T(0, DY_PUSH - 24*k, 0) @ p['M'], 'frame')

def e_of(theta):
    return float(np.clip(asm2.face_z(theta, ZM) - (ZY + 30), 0, TR))

# ---------------------------------------------------------------- layout: exit rails extended, lane UA, baseplates
D_STOP = 1309.5                                  # drive distance from the flap (track phase with the lowest link edge flat)
RX = -1892.0 - D_STOP                            # robot x at the station (line2: x = -1892 - drive)
XM = round((RX + XM_OFF)/20)*20
def rails_ext():
    """exit rails continued from x = -2480 to -3440 (same 2-wide walls as line1's)."""
    for s in (1, -1):
        z = 130*s
        for k in range(4):
            xs = [(-2560 - 160*i, '3007') for i in range(6)] if k % 2 == 0 else [(-2520, '3001')] + [(-2640 - 160*i, '3007') for i in range(5)] + [(-3400, '3001')]
            for xc, part in xs: add(f'urail{s}_{k}_{xc}', part, DBG if k % 2 else LBG, T(xc, -24 - 24*k, z))
        for k in range(2):
            xs = [(-2560 - 160*i, '3034') for i in range(6)] if k == 0 else [(-2520, '3020')] + [(-2640 - 160*i, '3034') for i in range(5)] + [(-3400, '3020')]
            for xc, part in xs: add(f'urailp{s}_{k}_{xc}', part, LBG, T(xc, -104 - 8*k, z))
        for xc in range(-2520, -3441, -80): add(f'urailt{s}_{xc}', '87079', BLK, T(xc, RAIL, z))

TAKEN0 = set()
def layout(taken=None, base_taken=frozenset()):
    rails_ext()
    rails.track('UA'); rails.tiles('UA', rails.TAKEN if taken is None else taken)
    cells = set(rails.baseplates('UA')) | {(x, z) for x in (-2880, -3520) for z in (330, -310)} | {(-3520, 970)}
    for cx, cz in sorted(cells - set(base_taken)): add(f'baseU{cx}_{cz}', '3811', LBG, T(cx, 0, cz))

def s_station(): return rails.s_of_x('UA', RX)
S_WAIT = 300.0
def station(st):
    """st: ucart (path parameter of the module wagon), utheta, ue (plunger offset), mod (module on the wagon: True/False),
    robot_M (robot frame -> world) and hook angle."""
    e = st['ue']
    def build(sx):
        carrier(sx, e)
        if st['mod_on_wagon']: module(T(sx - RX, 0, 0) @ st['robot_M_station'], e)
    sc = st['ucart']
    rails.cart('UA', sc, build, RX, 'usled', 'uw')
    rails.loco_at('UA', sc, 'uloco', 'uloco', 'uw', 'usled')
    pusher(st['utheta'], XM)
