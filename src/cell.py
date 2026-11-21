"""LEGO assembly cell (head station) built from real LDraw parts.
World = LDraw coords in LDU: x along the conveyor (flow +x), -y up, z across (conveyor at front, z<0).
Each placement: dict(name, part, col, M (4x4), body). Bodies move rigidly in the animation
('frame' is static). Snap/clash checks run per placement.
"""
import numpy as np

# colours (LDraw codes)
LBG, DBG, BLK, WHT, RED, YEL, BLU, ORG, TAN, LIM, TRC = 71, 72, 0, 15, 4, 14, 1, 25, 19, 27, 47
P = []

def T(x=0, y=0, z=0):
    M = np.eye(4); M[:3, 3] = [x, y, z]; return M
def R(axis, deg):
    a = np.radians(deg); c, s = np.cos(a), np.sin(a); M = np.eye(4)
    i, j = {'x': (1, 2), 'y': (2, 0), 'z': (0, 1)}[axis]
    M[i, i] = c; M[i, j] = -s; M[j, i] = s; M[j, j] = c
    return M
def add(name, part, col, M, body='frame'):
    P.append(dict(name=name, part=part, col=col, M=np.array(M, float), body=body))
    return P[-1]

# ---------------- layout constants ----------------
ZC = -190              # conveyor centre line
XS = 280               # sprocket axles at x = +-XS
YA = -46               # sprocket axle height
RP = 30                # chain pitch radius on 24T
NL = 82                # tread links
ZW = (ZC - 80, ZC + 80)  # guide/support walls (centres)
YP = -84               # pallet underside (top of tread)

_ST = {}
def build(st=None):
    global _ST
    st = dict(STATE, **(st or {})); _ST = st
    P.clear()
    add('base', '3811', LBG, T(0, 0, 0))
    # ---- conveyor walls: brick / plate / Technic brick (axle row) / brick / brick / tile ----
    for k, zw in enumerate(ZW):
        for xc in (-160, 160):
            s = f'{k}{"L" if xc < 0 else "R"}'
            add(f'w1_{s}', '2465', DBG, T(xc, -24, zw))
            for dx in (-80, 80): add(f'w2_{s}{dx}', '3460', DBG, T(xc + dx, -32, zw))
            add(f'w3_{s}', '3703', BLK, T(xc, -56, zw))
            add(f'w4_{s}', '2465', DBG, T(xc, -80, zw))
            add(f'w5_{s}', '2465', YEL, T(xc, -104, zw))
            sg = 1 if xc > 0 else -1
            if k == 1:                                   # inner wall: ledge column at x -40..40, portal post at x=50
                add(f'w6_{s}o', '4162', BLK, T(sg*240, -112, zw))
                if sg < 0: add(f'w6_{s}i', '6636', BLK, T(-100, -112, zw))
                else: add(f'w6_{s}i', '2431', BLK, T(120, -112, zw)); add(f'w6_{s}j', '3070b', BLK, T(70, -112, zw))
            elif k == 0:                                 # outer wall: portal posts at x=+-50, sensor at x 90..110
                add(f'w6_{s}o', '4162', BLK, T(sg*240, -112, zw))
                if sg < 0:
                    add(f'w6_{s}a', '2431', BLK, T(-120, -112, zw)); add(f'w6_{s}b', '3070b', BLK, T(-70, -112, zw)); add(f'w6_{s}c', '3069b', BLK, T(-20, -112, zw))
                else:
                    add(f'w6_{s}a', '3069b', BLK, T(20, -112, zw)); add(f'w6_{s}b', '3070b', BLK, T(70, -112, zw)); add(f'w6_{s}c', '3069b', BLK, T(140, -112, zw))
            else:
                for dx in (-80, 80): add(f'w6_{s}{dx}', '4162', BLK, T(xc + dx, -112, zw))
    # ---- sprocket axles (along z), bushes, 24T sprockets ----
    for side, x in (('idl', -XS), ('drv', XS)):
        if side == 'idl':
            add('ax_idl', '3708', BLK, T(x, YA, ZC) @ R('y', 90), body='idler')
            for z in (ZC - 100, ZC + 100): add(f'bush_idl{z}', '3713', LBG, T(x, YA, z), body='idler')
        else:
            add('ax_drv', '3708', BLK, T(x, YA, ZC + 10) @ R('y', 90), body='drive')      # z -310 .. -70
            add('bush_drv_o', '3713', LBG, T(x, YA, ZC - 100), body='drive')
            add("g24_drv", "3648b", LBG, T(x, YA, -80), body="drive")               # meshes with hub 8T
        add(f'spr_{side}', '3648b', LBG, T(x, YA, ZC), body='idler' if side == 'idl' else 'drive')
    # ---- tread belt ----
    for i, M in enumerate(tread_links(st['belt'])):
        add(f'tread{i:02d}', '3873', BLK, M, body=f'link{i:02d}')
    # ---- conveyor motor (Boost external motor, port C): 8T on its rotor -> 24T on drive axle (3:1) ----
    xm = XS - 40                                       # rotor axis x
    add('motC_plate', '3032', LBG, T(xm, -8, 0))       # 4x6 plate under the motor
    add('motC', '26913', WHT, T(xm, YA - 34, 0))       # rotor at (xm, YA, z=-60), facing -z
    add('ax_mC', '4519', BLK, T(xm, YA, -70) @ R('y', 90), body='motorC')     # z -40..-100
    add('g8_mC', '3647', LBG, T(xm, YA, -80), body='motorC')
    # ---- Move Hub on a pedestal, motor end at the back; outputs at the pusher's pin height ----
    hub_pedestal()
    ledge()
    pusher(st['theta'])
    bed()
    portals()
    press(st['beta'])
    magazine(st['heads'])
    sensor()
    # ---- pallet with locating pocket + robot (legs + body) ----
    pallet(st['pallet_x'], head=st['head_on'])
    return P

STATE = dict(theta=np.pi, beta=np.pi, belt=0.0, pallet_x=0.0, heads=4, head_on=False, head0_z=-110.0, head0_dy=0.0, drop=0.0)
# theta: pusher crank (pi = rest, 0 = pushed); beta: press crank (0 = up, pi = pressed at dead centre)

def ux(u):
    """rotation whose local y -> world x and local z -> unit vector u in the y-z plane (thin liftarms)."""
    u = np.asarray(u, float); u = u/np.linalg.norm(u)
    Y = np.array([1.0, 0, 0]); Z = np.array([0, u[1], u[2]]); X = np.cross(Y, Z)
    M = np.eye(4); M[:3, 0], M[:3, 1], M[:3, 2] = X, Y, Z; return M

def zdir(u):
    """rotation about x taking local +z to u (in the y-z plane); thick beams keep holes along x."""
    u = np.asarray(u, float); u = u/np.linalg.norm(u)
    return R('x', np.degrees(np.arctan2(-u[1], u[2])))

# Move Hub: axis along z, outputs (local z=+140) at the back; local x=+60 -> world x=+60
YO = -166                       # hub output height (= pusher pin height)
ZK = 310                        # hub output z (crank centre)
ZH = ZK - 140                   # hub centre z
YH = YO - 66                    # hub top (local y=0)

def hub_pedestal():
    ytop = YH + 96                                     # hub bottom = pedestal top (-136)
    for xc in (-50, 50):
        for k in range(5):
            add(f'ped{k}_{xc}', '2465', LBG if k % 2 else DBG, T(xc, -24 - 24*k, 160) @ R('y', 90))
        for dz in (-80, 80): add(f'pedp_{xc}{dz}', '3460', LBG, T(xc, -128, 160 + dz) @ R('y', 90))
        for k in range(8): add(f'jump_{xc}{k}', '3794b', LBG, T(xc, -136, 20 + 40*k) @ R('y', 90))   # half-stud shift in z
    add('hub', '26910', WHT, T(0, YH, ZH))

# ---------------- ledge, rails, pusher, crank, link ----------------
YL = -152              # ledge top = body stud tips; heads slide at this level
RC, LK = 40, 380       # crank radius, link length (pin rest -30, forward -110)

def ledge():
    # column on the inner conveyor wall (replaces its top tiles at x -40..40) and a back column
    add('lcol_b', '3010', DBG, T(0, -128, ZW[1]))
    add('lcol_p', '3710', DBG, T(0, -136, ZW[1]))
    for k in range(5): add(f'bcol{k}', '3010', DBG, T(0, -24 - 24*k, -30))
    add('bcol_p', '3710', DBG, T(0, -128, -30))
    add('bcol_p2', '3710', DBG, T(0, -136, -30))
    add('ledge', '3035', LBG, T(0, -144, -80) @ R('y', 90))          # 4x8 plate, z -160..0
    for k, y in enumerate((-152, -160)):                             # rails: 2 plates high (guide the head's lower 8 LDU)
        add(f'rail_R{k}', '3460', YEL, T(30, y, -80) @ R('y', 90))       # right rail full length
        add(f'rail_L{k}', '3023', YEL, T(-30, y, -140) @ R('y', 90))     # left rail only at the front (pusher axle passes behind)
    add('tile_L', '6636', LBG, T(-30, -152, -60) @ R('y', 90))
    for z in (-120, -40): add(f'ledge_t{z}', '87079', LBG, T(0, -152, z) @ R('y', 90))

def pusher(theta):
    zp, ang, crank_pin = slider(theta)
    zc = zp - 20                                   # block centre (axle in its back hole)
    for x in (-10, 10): add(f'pb{x}', '3701', BLU, T(x, YL - 24, zc) @ R('y', 90), 'pusher')
    add('pb_tile', '87079', WHT, T(0, YL - 32, zc) @ R('y', 90), 'pusher')
    add('pb_axle', '3707', BLK, T(-60, YO, zp), 'pusher')               # x -140..20
    add('pb_bush', '32123a', LBG, T(-125, YO, zp) @ R('y', 90), 'pusher')
    # crank on the hub output (x = -60 side): axle 2 + thin 4L liftarm (axle hole on the output, round hole at r=40)
    add('cr_axle', '3704', BLK, T(-58, YO, ZK), 'crank')
    u = np.array([0, crank_pin[0] - YO, crank_pin[1] - ZK])
    if RC == 60: add('cr_arm', '32348', RED, T(-70, YO, ZK) @ ux(u), 'crank')        # long-stroke variant: thick bent liftarm, pin in its 4th hole
    else: add('cr_arm', '32449', RED, T(-75, YO, ZK) @ ux(u) @ T(0, 0, 30), 'crank')    # first hole on the axle
    # link: beam 7 (crank end, inner plane x=-90) + beam 15 (pusher end, outer plane x=-110), pinned at 2 holes
    p0 = np.array([YO, zp]); p1 = np.array(crank_pin); d = (p1 - p0)/LK
    def at(s): return p0 + d*s
    D = ux([0, d[0], d[1]])                         # beams: holes along local y -> world x
    y, z = at(140); add('lk_b15', '32278', DBG, T(-110, y, z) @ D, 'conrod')
    y, z = at(320); add('lk_b7', '32524', DBG, T(-90, y, z) @ D, 'conrod')
    for s_ in (260, 280):
        y, z = at(s_); add(f'lk_pin{s_}', '2780', BLK, T(-100, y, z), 'conrod')
    y, z = at(380); add('cr_pin', '32002', BLK, T(-80, y, z), 'conrod')       # 3/4 pin: 20 in the beam, 10 in the crank

# ---------------- slide bed under the tread at the station ----------------
def bed():
    add('bed_b', '3894', DBG, T(0, -56, ZC), 'frame')                  # Technic brick 1x6 along x, holes along z at y=-46
    add('bed_p', '3666', DBG, T(0, -64, ZC))
    add('bed_t', '6636', LBG, T(0, -72, ZC))                            # top -72: tread links slide over it
    for x in (-40, 40): add(f'bed_ax{x}', '3737', BLK, T(x, YA, ZC) @ R('y', 90))    # axle 10 across both walls

# ---------------- portals carrying the press shaft ----------------
YS, ZS = -198, -150     # press shaft
def portals():
    for sx in (-1, 1):
        x = 50*sx
        posts = [(x, -270)] + ([(x, -110)] if sx > 0 else [])
        for (px, pz) in posts:
            for k in range(3): add(f'post{sx}_{pz}_{k}', '3005', DBG, T(px, -128 - 24*k, pz))
            add(f'post{sx}_{pz}_p', '3024', DBG, T(px, -184, pz))
            add(f'post{sx}_{pz}_t', '6541', BLK, T(px, -208, pz) @ R('y', 90))          # hole along x at y=-198
            add(f'post{sx}_{pz}_pin', '2780', BLK, T(60*sx, YS, pz))
        add(f'portal{sx}', '40490', YEL, T(70*sx, YS, -190) @ ux([0, 0, 1]))            # beam 9, z -270..-110
    for z in (-250, -230): add(f'xbar{z}', '3737', BLK, T(0, YS, z))                     # crossbar axles

def press(beta):
    phi = press_phi(beta)
    u = np.array([0, np.sin(phi), -np.cos(phi)])              # front lever direction
    ub = np.array([0, -np.cos(phi), -np.sin(phi)])            # back arm direction
    add('pr_shaft', '3737', BLK, T(0, YS, ZS), 'press')
    for x in (-5, 5): add(f'pr_lever{x}', '6632', LBG, T(x, YS, ZS) @ ux(u), 'press')
    add('pr_back', '32065', LBG, T(90, YS, ZS) @ ux(ub) @ T(0, 0, 60), 'press')       # 7L thin, axle end on the shaft
    tip = np.array([YS, ZS]) + PR_AB*ub[1:]
    pin = np.array([YO + PR_R*np.sin(beta), ZK - PR_R*np.cos(beta)])
    # B crank: axle 3 into the hub output (x=+60), 4L thin liftarm, r = 40
    add('prc_axle', '3705', BLK, T(74, YO, ZK), 'pcrank')                              # axle 4: x 34..114 (ends inside the crank)
    add('prc_arm', '32449', RED, T(110, YO, ZK) @ ux([0, *(pin - [YO, ZK])]) @ T(0, 0, 30), 'pcrank')
    d = (pin - tip)/PR_L; D = ux([0, d[0], d[1]])
    y, z = tip + d*140; add('pc_b15', '32278', DBG, T(105, y, z) @ D, 'pconrod')       # tip end, plane x 96..114
    y, z = tip + d*360; add('pc_b11', '32525', DBG, T(125, y, z) @ D, 'pconrod')       # crank end, plane x 116..134
    for s_ in (260, 280):
        y, z = tip + d*s_; add(f'pc_pin{s_}', '2780', BLK, T(115, y, z), 'pconrod')
    add('pc_pin_tip', '32002', BLK, T(95, *tip) @ R('y', 180), 'pconrod')     # 10 in the back arm (x 85..95), 20 in beam 15
    add('pc_pin_crank', '32002', BLK, T(115, *pin) @ R('y', 180), 'pconrod')  # 10 in the crank (105..115), 20 in beam 11

PR_AB, PR_R, PR_L = 100, 40, 460      # back arm, B crank radius, press conrod (beam 15 + beam 11)
def _beta_press():
    tipdir = np.array([YS - PR_AB*np.cos(np.radians(13.29)) - YO, ZS - PR_AB*np.sin(np.radians(13.29)) - ZK])
    u = tipdir/np.linalg.norm(tipdir); return np.arctan2(u[0], -u[1])      # pin direction (sin b, -cos b) = u

def press_phi(beta):
    from scipy.optimize import brentq
    C = np.array([YO, ZK]); pin = C + PR_R*np.array([np.sin(beta), -np.cos(beta)])
    tip = lambda p: np.array([YS - PR_AB*np.cos(p), ZS - PR_AB*np.sin(p)])
    f = lambda p: np.linalg.norm(tip(p) - pin) - PR_L
    return brentq(f, np.radians(-45), np.radians(13.35))

# ---------------- magazine ----------------
def magazine(n):
    """walls at x=+-30 spanning z -160..-60 (odd stud grid like the right rail). Layer 2 is a Technic 1x1 at z=-150
    whose hole carries the press shaft right next to the lever. Layer 4 holds a front/back stop axle across."""
    for sx in (-1, 1):
        x = 30*sx; R90 = R('y', 90)
        def br(name, part, yt, zc, rot=R90): add(f'mag{sx}_{name}', part, DBG, T(x, yt, zc) @ rot)
        if sx > 0:                                               # layer 1 only on the right (left hangs above the pusher axle)
            br('1a', '3004', -184, -140); br('1b', '3004', -184, -100); br('1c', '3005', -184, -70)
        br('2a', '6541', -208, -150, R('y', 90) @ R('y', -90) @ R('y', 90)); br('2b', '3010', -208, -100)
        br('3a', '3010', -232, -120); br('3b', '3005', -232, -70)
        br('4a', '3700', -256, -140); br('4b', '3005', -256, -110); br('4c', '3700', -256, -80)
        br('5a', '3005', -280, -150); br('5b', '3010', -280, -100)
        br('6a', '3010', -304, -120); br('6b', '3005', -304, -70)
    for z in (-140, -80): add(f'mag_stop{z}', '3705', BLK, T(0, -246, z))      # axle 4 across the channel
    for z in (-150, -70): add(f'mag_top{z}', '3710', YEL, T(0, -312, z))
    st = _ST
    for k in range(n):
        if k == 0 and st['head0_z'] is None: continue
        z = st['head0_z'] if k == 0 else -110
        y = YL - 24 - 32*k + (st['head0_dy'] if k == 0 else st['drop'])
        head_at(f'head{k}', st['pallet_x'] if (k == 0 and st.get('head0_on_pallet')) else 0, y, z, f'head{k}')

# ---------------- colour & distance sensor (port D) on the outer wall, looking across at the robots ----------------
def sensor():
    add('sensor', '26912', WHT, T(100, -104 - 48, -300) @ R('y', 180))     # eye (local -z face) looks +z at the robots

def slider(theta):
    """in-line slider-crank: returns pusher pin z, link angle, crank pin (y, z)."""
    cy, cz = YO + RC*np.sin(theta), ZK - RC*np.cos(theta)
    zp = cz - np.sqrt(LK**2 - (RC*np.sin(theta))**2)
    return zp, np.arcsin(RC*np.sin(theta)/LK), (cy, cz)

def tread_links(phase):
    """82 links on the stadium path (pins at radius RP around both sprockets). phase in LDU of belt travel."""
    L = 2*XS; per = 2*L + 2*np.pi*RP; pitch = per/NL
    out = []
    for i in range(NL):
        s = (i*pitch + phase) % per
        if s < L:                                    # top run, moving +x
            p = np.array([-XS + s, YA - RP, 0]); ang = 0.0
        elif s < L + np.pi*RP:                       # around drive sprocket
            a = (s - L)/RP; p = np.array([XS + RP*np.sin(a), YA - RP*np.cos(a), 0]); ang = np.degrees(a)
        elif s < 2*L + np.pi*RP:                     # bottom run, moving -x
            p = np.array([XS - (s - L - np.pi*RP), YA + RP, 0]); ang = 180.0
        else:
            a = (s - 2*L - np.pi*RP)/RP; p = np.array([-XS - RP*np.sin(a), YA + RP*np.cos(a), 0]); ang = 180 + np.degrees(a)
        # link frame: local z = travel tangent, local -y = outward, local x = across (world z)
        B = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]], float)   # local z -> world x, local x -> world -z
        M = T(p[0], p[1], ZC) @ R('z', ang) @ B
        out.append(M)
    return out

def pallet(x, body='pallet', head=False):
    y0 = YP - 8                                         # plate top surface
    add('pal_plate', '3036', LBG, T(x, y0, ZC), body)
    add('pal_floor', '87079', DBG, T(x, y0 - 8, ZC), body)
    for dz in (-40, 40):
        add(f'pal_p28{dz}', '3034', LBG, T(x, y0 - 8, ZC + dz), body)
        for dx in (-40, 40): add(f'pal_t24{dz}{dx}', '87079', BLU, T(x + dx, y0 - 16, ZC + dz), body)
    for dx in (-60, 60):
        add(f'pal_p22{dx}', '3022', LBG, T(x + dx, y0 - 8, ZC), body)
        add(f'pal_t22{dx}', '3068b', BLU, T(x + dx, y0 - 16, ZC), body)
    # robot sub-assembly already on the pallet: legs + body
    add('rob_legs', '3001', DBG, T(x, y0 - 8 - 24, ZC), 'robot')
    add('rob_body', '3001', RED, T(x, y0 - 8 - 48, ZC), 'robot')
    if head: head_at('rob_head', x, YL - 24 + 0, ZC, 'robot')

def head_at(name, x, ybrick, z, body):
    """robot head: 2x2 brick + 2x2 tile (smooth top so heads stack loosely in the magazine). ybrick = brick top."""
    add(name + '_b', '3003', WHT, T(x, ybrick, z), body)
    add(name + '_t', '3068b', LBG, T(x, ybrick - 8, z), body)

# ---------------- the station cycle ----------------
BELT_V = 40.0 / 30       # LDU per frame at 30 fps (~16 mm/s)
PHASES = [('arrive', 1.4), ('push', 1.0), ('retract', 1.0), ('drop', 0.25), ('press', 1.2), ('leave', 1.4)]

def cycle(t):
    """state at normalised cycle time t in [0, 1)."""
    tot = sum(d for _, d in PHASES); u = (t % 1.0)*tot
    for name, d in PHASES:
        if u <= d: f = u/d; break
        u -= d
    ease = lambda x: 0.5 - 0.5*np.cos(np.pi*np.clip(x, 0, 1))
    bp = _beta_press()
    st = dict(theta=np.pi, beta=np.pi, pallet_x=0.0, heads=4, head0_z=-110.0, head0_dy=0.0, drop=0.0, head0_on_pallet=False)
    if name == 'arrive': st['pallet_x'] = -200*(1 - ease(f))
    elif name == 'push':
        st['theta'] = np.pi*(1 - ease(f)); zp = slider(st['theta'])[0]; st['head0_z'] = min(-110.0, zp - 20 - 40 - 20)
    elif name == 'retract':
        st['theta'] = -np.pi*ease(f); st['head0_z'] = -190.0
    elif name == 'drop':
        st['theta'] = -np.pi; st['head0_z'] = -190.0; st['drop'] = 32*min(1, (f/0.6)**2)
        st['heads'] = 4
    elif name == 'press':
        b = np.pi + (bp - np.pi)*np.sin(np.pi*f)      # up -> dead centre -> up
        st['beta'] = b; st['head0_z'] = -190.0; st['drop'] = 32
        phi = press_phi(b); st['head0_dy'] = max(0.0, min(4.0, (YS + 40*np.sin(phi) + 9) - (-184))) if f < 0.5 else 4.0
    elif name == 'leave':
        st['pallet_x'] = 200*ease(f); st['head0_z'] = -190.0; st['head0_dy'] = 4.0; st['drop'] = 32; st['head0_on_pallet'] = True
    st['phase'] = name
    st['belt'] = 200*ease(f) if name == 'arrive' else (200 + 200*ease(f) if name == 'leave' else 200)
    return st

if __name__ == '__main__':
    build()
    print(len(P), 'placements')
