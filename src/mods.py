"""Module catalogue for the modular robot. Robot frame (worker2/tug convention): x right, y down, z backwards (chest at -z).
Slots and their interfaces (all fitted by a sideways push along robot x, the stations' push direction):
  lowerL / lowerR : track unit (beam pinned to the hub side hole at y -231, drive axle into the hub output) + optional extras
  upperL / upperR : shoulder kit (Technic brick on three shoulder pins) + optional device
A device's plug rides on the module that carries the device and enters its hub port (C left, D right) in the same push.
"""
import numpy as np
import worker2, lcheck
import cable as _cab
from asm2 import T, R, col3, LBG, DBG, BLK, WHT, RED, YEL

EYE_ROT = col3((1, 0, 0), (0, 0, 1), (0, -1, 0))          # colour sensor looking down (as on the tug)
EYE_POS = (20.0, -31.0, -102.0)                            # in front of the chest, below the hub's bottom edge

def eye_bracket(a, sx=1):
    """belly sensor carried by the right track unit: L-liftarm on the outside of the track beam (holes y -51, -31),
    reaching forward to an axle pin into a Technic brick, which carries a 2x4 plate; the sensor's back sits on the plate."""
    # L-liftarm 32140: long leg local z (axle end at 0 .. corner at 60), short leg local x to (20, 0, 60); holes along local y
    # placed with the corner at (90, -51, 0), short leg down to y -31, long leg forward to its axle end at z -60
    Lm = np.eye(4); Lm[:3, 0] = (0, 1, 0); Lm[:3, 1] = (-1, 0, 0); Lm[:3, 2] = (0, 0, 1)
    a('eb_L', '32140', DBG, T(90, -51, -60) @ Lm)
    for y in (-51, -31): a(f'eb_pin{y}', '2780', BLK, T(80, y, 0))
    a('eb_axp', '3749', BLK, T(80, -51, -60))
    a('eb_brick', '3700', DBG, T(70, -51, -70) @ col3((0, 1, 0), (0, 0, 1), (1, 0, 0)))
    a('eb_plate', '3020', DBG, T(40, -51, -78) @ col3((1, 0, 0), (0, 0, 1), (0, -1, 0)))
    a('eye', '26912', WHT, T(*EYE_POS) @ EYE_ROT)
    # cable clamp: a 1x8 brick column (+2 plates) standing on the plate, two 2x4 plates across the folded cable
    Mc = col3((0, 1, 0), (0, 0, 1), (1, 0, 0))
    a('cl_col', '3008', DBG, T(70, -91, -102) @ Mc)
    for k, z in enumerate((-110, -118, -126)): a(f'cl_colp{k}', '3460', DBG, T(70, -91, z) @ Mc)
    for k, y in enumerate((-151, -111)): a(f'cl_cov{k}', '3020', LBG, T(40, y, -134) @ col3((1, 0, 0), (0, 0, 1), (0, -1, 0)))

def eye_cable():
    """plug (port D) -> tight U-turn forwards -> across the chest -> down, lying flat on the chest in two S-folds
    (stacked outwards, held by the bracket's clamp plates) -> into the sensor's cable exit."""
    name = '~modcab_eye'
    if name in _cab._SYN: return name
    ex, ey, ez = EYE_POS[0], EYE_POS[1] - 36, EYE_POS[2] + 8               # sensor cable exit (top), entered heading down
    z1 = -81.5                                                              # clear of the chest studs (-70) even while twisting
    def build(Lf):
        r = _cab.Route((54, -191, -55), (1, 0, 0), (0, -1, 0))
        r.to((58, -191, -55)).B((0, 1, 0), 180, (-z1 - 55)/2)              # U-turn on edge, ends heading -x at z1
        r.S(16, 90).to((ex + 10, -191, z1)).B((0, 0, 1), -90, 10)          # twist, bend down at the sensor's x
        r.S(16, 90)                                                         # twist flat against the chest
        r.S(Lf)
        for _ in range(3): r.fold_pair((0, 0, -1), Lf)                    # three S-folds hanging down, stacked outwards
        zl = r.p[2]; dz = ez - zl; s = np.sign(dz)
        rs = max(12.0, abs(dz)/1.8); th = np.degrees(np.arccos(1 - abs(dz)/(2*rs)))   # S-bend (about the wide axis x) onto the exit's z
        ax = np.cross(r.T, (0, 0, s))
        r.B(ax, th, rs); r.B(-ax, th, rs)
        r.to((ex, ey, ez))
        return r
    r, Lf = _cab.solve_fold(build, 1.0, 85.0)
    _cab.register(name, r); return name

def eye_module(a, add_cable):
    """lower-right extras: belly sensor on its bracket, plug for port D, clamped cable."""
    eye_bracket(a)
    a('plugD', '23816a', LBG, T(40, -191, -55) @ col3((0, -1, 0), (0, 0, 1), (-1, 0, 0)))
    add_cable('cabD', eye_cable(), ('plugD', 'eye'))

# ---------------------------------------------------------------- back interface (part of the body prefab)
def back_bricks(a):
    """two vertical Technic bricks on the hub's lowest back rows: side-facing pin holes (z 40; y -111, -91, -71) for back modules."""
    for x in (-50, 50): a(f'bk{x}', '3701', DBG, T(x, -91, 30) @ col3((0, 1, 0), (0, 0, 1), (1, 0, 0)))

# ---------------------------------------------------------------- hook module (fitted with the left track unit, from the left)
from cell import ux
HOOK_UP, HOOK_DOWN = 30.0, -25.0
DZ = 56.0                                                   # behind the back bricks, leaving a 28-LDU cable slot
YC = -91.0
def hook_module(a, add_cable=None, hook=HOOK_UP, body_hook='hook'):
    """Boost motor behind the back bricks, output on the left; two 5L liftarms carry pins into the left back brick and the
    motor's side holes (all pins point +x, so the whole module goes on with one sideways push). Hook arm on the output."""
    MM = np.eye(4); MM[:3, :3] = np.column_stack([[0, -1, 0], [0, 0, 1], [-1, 0, 0]])
    a('hmotor', '26913', WHT, T(0, 0, 64 + DZ) @ R('y', 180) @ T(0, 0, -64) @ T(0, YC, 30) @ MM)
    for y in (-71, -111):
        a(f'hb_arm{y}', '32316', DBG, T(-70, y, 80) @ R('z', 90))           # 5L, holes along x at z 40 .. 120
        for z in (40, 120): a(f'hb_pin{y}_{z}', '2780', BLK, T(-60, y, z))
    zo = 64 + DZ
    a('h_axle', '4519', BLK, T(-74, YC, zo), body_hook)
    A = T(0, YC, zo) @ R('x', hook)
    a('h_arm', '32449', RED, A @ T(-95, 0, 0) @ ux([0, 0, 1]) @ T(0, 0, 30), body_hook)
    a('h_cross', '32073', BLK, A @ T(-45, 0, 60), body_hook)
    a('h_block', '6536', DBG, A @ T(0, 0, 60) @ col3((1, 0, 0), (0, 0, 1), (0, -1, 0)), body_hook)
    a('h_peg', '6558', BLK, A @ T(0, 20, 80) @ R('z', 90), body_hook)
    a('plugC', '23816a', LBG, T(-40, -191, -55) @ col3((0, 1, 0), (0, 0, 1), (1, 0, 0)))
    if add_cable: add_cable('cabC', hook_cable(), ('plugC', 'hmotor'))

def hook_cable_route(Lf, dbg=False, zd=-30.0):
    """plug C (left port) -> on-edge S-bend back to z zd -> twist, down beside the track beam with one S-fold
    (stacked outwards, under the hood on the left track beam) -> back along the hub side on edge ->
    through the slot between the back bricks and the motor -> round the motor's right side into its cable exit."""
    pr = (lambda tag: print(tag, r.p.round(1), r.T.round(2), r.W.round(2))) if dbg else (lambda tag: None)
    r = _cab.Route((-54, -191, -55), (-1, 0, 0), (0, 1, 0))
    dz = zd + 55; rs = 13.0; th = np.degrees(np.arccos(1 - dz/(2*rs)))
    r.B((0, 1, 0), th, rs).B((0, 1, 0), -th, rs); pr('sbend')           # on edge, shift backwards
    r.S(16, 90); pr('tw1')                                               # twist: wide axis along z
    r.B(np.cross(r.T, (0, 1, 0)), 90, 5); pr('down')                    # bend down at x -101
    r.S(Lf)
    if Lf > 0: r.fold_pair((1, 0, 0), Lf)                                # one S-fold, layers stacked inwards (+x)
    pr('folds')
    xo = -101.0; d = r.p[0] - xo; th2 = np.degrees(np.arccos(1 - d/16))  # on-edge S-bend back out to x -101
    ax = np.cross(r.T, (-1, 0, 0)); r.B(ax, th2, 8).B(-ax, th2, 8); pr('sb2')
    r.S(16, 90).to((xo, -101, zd)); pr('tw2')                            # twist: wide axis along x
    r.B(np.cross(r.T, (0, 0, 1)), 90, 10); pr('back')                    # bend backwards at y -91
    r.S(16, 90); pr('tw3')                                               # twist on edge (wide axis vertical), clear of the track beam
    r.to((r.p[0], -91, 58)).B((0, 1, 0), 90, 10); pr('right')            # into the slot between back bricks and motor
    r.to((60, -91, 68)).B((0, 1, 0), -90, 10); pr('rback')               # out of the slot, backwards beside the motor
    r.to((70, -91, 128 + DZ - 37 - 10)).B((0, 1, 0), -90, 10); pr('in')  # in towards the motor's cable exit (right face)
    r.to((56, -91, 128 + DZ - 37))
    return r

def hook_cable():
    name = '~modcab_hook'
    if name in _cab._SYN: return name
    r, Lf = _cab.solve_fold(hook_cable_route, 1.0, 50.0)
    _cab.register(name, r); return name

def hook_hood(a):
    """cable hood on the left track beam (outer side): 3x5 L-liftarm forward over the S-fold, 2x4 L hanging in front of it."""
    L1 = np.eye(4); L1[:3, 0] = (0, 1, 0); L1[:3, 1] = (-1, 0, 0); L1[:3, 2] = (0, 0, 1)
    a('hd_L1', '32526', DBG, T(-90, -211, -80) @ L1)
    for y in (-211, -171): a(f'hd_pin{y}', '2780', BLK, T(-80, y, 0))
    L3 = np.eye(4); L3[:3, 0] = (0, 0, 1); L3[:3, 1] = (-1, 0, 0); L3[:3, 2] = (0, -1, 0)
    a('hd_L3', '32140', DBG, T(-70, -151, -80) @ L3)
    for z in (-80, -60): a(f'hd_pinf{z}', '2780', BLK, T(-80, -211, z))

# ---------------------------------------------------------------- shoulder hook module (v3): fitted on the standing robot, from the left
# Base = the shoulder-kit interface (Technic brick 1x8 on the hub's three left shoulder pins), so it never touches the back and
# the robot can lie on its back on the flap pallet before. Two 7L beams run back from the brick's upper holes to a Boost motor
# high behind the hub; a 15L arm on its output carries the hook peg. Plug C, cable and module all move in the same sideways push.
import asm2 as _asm2
KIT_G = np.linalg.inv(_asm2.core_M0(0.0))                   # periph kit frame (lying, px = 0) -> robot frame
SH_Y, SH_Z = -391.0, 120.0                                  # hook motor output axis (robot frame)
SH_DX = -20.0                                               # motor shifted left so its side face meets the beams
SH_UP, SH_DOWN = 25.0, -52.0
SH_ARM = 280.0
def shook_peg(hook):
    """hook peg tip (robot frame) for a hook angle."""
    A = T(0, SH_Y, SH_Z) @ R('x', hook); return (A @ np.array([-14, 40, SH_ARM + 20, 1.0]))[:3]

def shook_module(a, add_cable=None, hook=SH_UP, body_hook='hook'):
    Y0 = _asm2.Y0
    a('sk_brick', '3702', DBG, KIT_G @ T(160, Y0 - 10, 70))
    for x in (100, 120, 140): a(f'sk_pin{x}', '2780', BLK, KIT_G @ T(x, Y0, 60) @ R('y', 90))
    for y in (-371, -411):
        a(f'sk_beam{y}', '32524', DBG, T(-90, y, 60) @ R('z', 90))                 # 7L, holes along x at z 0 .. 120
        for z in (0, 120): a(f'sk_bpin{y}_{z}', '2780', BLK, T(-80, y, z))
    MM = np.eye(4); MM[:3, :3] = np.column_stack([[0, -1, 0], [0, 0, 1], [-1, 0, 0]])
    a('sk_motor', '26913', WHT, T(SH_DX, 0, 0) @ T(0, 0, 64 + DZ) @ R('y', 180) @ T(0, 0, -64) @ T(0, SH_Y, 30) @ MM)
    a('sk_axle', '3705', BLK, T(-104 + SH_DX + 10, SH_Y, SH_Z), body_hook)            # 4L: 16 in the output, through the arm
    A = T(0, SH_Y, SH_Z) @ R('x', hook)
    AR = col3((0, -1, 0), (1, 0, 0), (0, 0, 1))                                      # beam length along +z, holes along x
    a('sk_arm', '32278', RED, A @ T(-114, 0, SH_ARM/2) @ AR, body_hook)
    a('sk_cross', '44294', BLK, A @ T(-54, 0, SH_ARM), body_hook)                    # 7L axle across to the robot's centre line
    a('sk_block', '6536', DBG, A @ T(0, 0, SH_ARM) @ col3((1, 0, 0), (0, 0, 1), (0, -1, 0)), body_hook)
    a('sk_peg', '6558', BLK, A @ T(0, 20, SH_ARM + 20) @ R('z', 90), body_hook)
    a('plugC', '23816a', LBG, T(-40, -191, -55) @ col3((0, 1, 0), (0, 0, 1), (1, 0, 0)))
    if add_cable: add_cable('cabC', shook_cable(), ('plugC', 'sk_motor'))

def shook_route(xo, dbg=False):
    """plug C -> out, up the hub's left side outside the shoulder beams -> back above the track beam -> up behind the hub ->
    right between the two beams, through the gap between hub top and motor -> back beside the motor into its cable exit.
    No surplus to fold: the first outward run is just long enough (xo solved for the real 625 LDU)."""
    pr = (lambda tag: print(tag, r.p.round(1), r.T.round(2), r.W.round(2), round(r.L))) if dbg else (lambda tag: None)
    yb = -280.0; zu = 60.0; xm = 40 + SH_DX + 30
    r = _cab.Route((-54, -191, -55), (-1, 0, 0), (0, 1, 0))
    r.to((xo + 26, -191, -55)).S(16, 90); pr('tw')                                   # out of the plug, twist: wide axis along z
    r.B(np.cross(r.T, (0, -1, 0)), 90, 10); pr('up')       # bend up
    r.S(16, 90); pr('tw2')                                                        # twist: wide axis along x
    r.to((r.p[0], yb + 10, r.p[2])).B(np.cross(r.T, (0, 0, 1)), 90, 10); pr('back')
    r.S(16, 90); pr('tw3')                                                        # on edge (wide vertical) over the track beam
    r.to((r.p[0], r.p[1], zu - 26)).S(16, 90); pr('tw4')                          # wide axis along x again
    r.B(np.cross(r.T, (0, -1, 0)), 90, 10); pr('up2')
    r.S(16, 90); pr('tw5')                                                        # wide axis along z
    r.to((r.p[0], SH_Y + 10, r.p[2])).B(np.cross(r.T, (1, 0, 0)), 90, 10); pr('right')   # between the beams, flat
    r.S(16, 90); pr('tw6')                                                        # on edge: wide vertical
    r.to((xm - 10, r.p[1], r.p[2])).B((0, 1, 0), -90, 10); pr('rback')
    ze = 128 + DZ - 37
    r.to((xm, r.p[1], ze - 10)).B((0, 1, 0), -90, 10); pr('in')
    r.to((56 + SH_DX, r.p[1], ze))
    return r

def shook_cable():
    name = '~modcab_shook'
    if name in _cab._SYN: return name
    a, b = -108.0, -140.0
    for _ in range(40):
        m = 0.5*(a + b)
        if shook_route(m).L < _cab.L_REAL: a = m
        else: b = m
    r = shook_route(0.5*(a + b)); SHOOK['xo'] = 0.5*(a + b)
    _cab.register(name, r); return name
SHOOK = {}
