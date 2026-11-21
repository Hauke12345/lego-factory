"""Worker v2: a standing, modular robot from real LDraw parts (LDraw coords, -y up, robot faces -z, ground y=GROUND).

Modules (each one a prefab the factory feeds and fits):
  core      Move Hub standing upright: stud face = chest (faces -z), motor end down
  track L/R triangular track frames: beam 13 pinned to the hub's middle side hole, the hub's own motor output
            drives the top sprocket, two idlers on a beam 9 at the bottom
  head      4x4 plate pressed onto the top two chest stud rows, 2x4 brick face with 2x2 round eyes
  gun arm   Boost motor as backpack (studs into the hub's back), arm on its output, stud blaster in the hand;
            its cable plugs into the hub's side port
  tool arm  7L arm pinned into the two upper side holes on the left
"""
import numpy as np
from cell import T, R, ux, LBG, DBG, BLK, WHT, RED, YEL, BLU, ORG
TLB, DKBL = 43, 272

# upright hub: local x -> world -x, local y -> world +z, local z -> world +y
MH = np.eye(4); MH[:3, :3] = np.column_stack([[-1, 0, 0], [0, 0, 1], [0, 1, 0]])
HC = np.array([0, -211, -66])           # hub origin: outputs at (+-60, -71, 0)
Y_OUT = -71
Y_ID, Z_ID = -11, 80                    # idler axles
RP = 30
GROUND = Y_ID + RP + 8                  # track outer surface under the idlers = 27

def hub_pt(xl, yl, zl): return HC + MH[:3, :3] @ np.array([xl, yl, zl])

def track_path(nlinks=34):
    """pin positions/angles of a closed track around drive (top) and two idlers, all pitch radius RP."""
    C = [np.array([Y_OUT, 0.0]), np.array([Y_ID, Z_ID]), np.array([Y_ID, -Z_ID])]   # (y, z), clockwise seen from +x
    segs = []
    for k in range(3):
        a, b = C[k], C[(k + 1) % 3]; d = (b - a)/np.linalg.norm(b - a)
        n = np.array([-d[1], d[0]])                       # outward normal (to the left of travel in (y,z))
        if np.dot(n, a - sum(C)/3) < 0: n = -n
        segs.append((a + RP*n, b + RP*n, n))
    pieces = []                                           # (kind, data, length)
    for k in range(3):
        p0, p1, n = segs[k]; pieces.append(('L', (p0, p1), np.linalg.norm(p1 - p0)))
        c = C[(k + 1) % 3]; n0 = n; n1 = segs[(k + 1) % 3][2]
        a0, a1 = np.arctan2(n0[1], n0[0]), np.arctan2(n1[1], n1[0])
        da = (a1 - a0 + np.pi) % (2*np.pi) - np.pi
        pieces.append(('A', (c, a0, da), abs(da)*RP))
    per = sum(p[2] for p in pieces); pitch = per/nlinks
    def at(s):
        s %= per
        for kind, dat, ln in pieces:
            if s <= ln:
                if kind == 'L':
                    p0, p1 = dat; t = (p1 - p0)/ln; return p0 + t*s, t
                c, a0, da = dat; a = a0 + np.sign(da)*s/RP
                p = c + RP*np.array([np.cos(a), np.sin(a)]); t = np.sign(da)*np.array([-np.sin(a), np.cos(a)])
                return p, t
            s -= ln
    return at, pitch, per

def build(st=None, M0=np.eye(4), tag='w', body='worker', P=None, gun=True, track_phase=0.0, arm=np.radians(70), modules=None):
    st = st or {}
    mods = modules or {'core', 'trackL', 'trackR', 'head', 'gunarm', 'toolarm'}
    P = [] if P is None else P
    def a(name, part, col, M, b=body):
        P.append(dict(name=f'{tag}_{name}', part=part, col=col, M=M0 @ M, body=b))
    Hm = T(*HC) @ MH
    if 'core' in mods: a('hub', '26910', WHT, Hm)
    # ---------------- tracks ----------------
    at, pitch, per = track_path()
    for sx in (-1, 1):
        if f'track{"L" if sx < 0 else "R"}' not in mods: continue
        s = 'L' if sx < 0 else 'R'
        yh = hub_pt(0, 0, -20)[1]                                        # middle side hole height (-231)
        a(f'tk{s}_vbeam', '41239', DBG, T(70*sx, yh + 120 - 20, 0) @ ux([0, 1, 0]))   # holes -251..-11, pinned at -231
        a(f'tk{s}_pin', '2780', BLK, T(60*sx, yh, 0))
        a(f'tk{s}_hbeam', '40490', DBG, T(90*sx, Y_ID, 0) @ ux([0, 0, 1]))
        a(f'tk{s}_pin2', '2780', BLK, T(80*sx, Y_ID, 0))
        a(f'tk{s}_drv_ax', '3706', BLK, T(100*sx, Y_OUT, 0))              # x 40..160 into the hub output
        for z in (-Z_ID, Z_ID): a(f'tk{s}_id_ax{z}', '3705', BLK, T(120*sx, Y_ID, z))
        for (y, z, nm) in ((Y_OUT, 0, 'drv'), (Y_ID, -Z_ID, 'idf'), (Y_ID, Z_ID, 'idb')):
            a(f'tk{s}_spr_{nm}', '3648b', LBG, T(136*sx, y, z) @ R('y', 90))
        # belt travel: moving forward (-z) by d, the ground run must stand still, so the loop parameter (which runs -z along the bottom) goes back by d
        ph = track_phase[0 if sx < 0 else 1] if isinstance(track_phase, (tuple, list)) else track_phase
        for i in range(34):
            p, t = at(-ph + i*pitch)
            out = -np.array([-t[1], t[0]]) if sx > 0 else -np.array([-t[1], t[0]])
            # link frame: local z = travel tangent, local -y = outward, local x = across (world x)
            Z = np.array([0, t[0], t[1]]); Yv = np.array([0, out[0], out[1]]); X = np.cross(Yv, Z)   # 'out' points into the loop: local -y (grip plate) must face away from it
            M = np.eye(4); M[:3, 0], M[:3, 1], M[:3, 2] = X, Yv, Z; M[:3, 3] = [136*sx, p[0], p[1]]
            a(f'tk{s}_link{i:02d}', '3873', BLK, M)
    # ---------------- head ----------------
    if 'head' in mods:
        a('hd_plate', '3031', DBG, T(0, -371, -74) @ R('x', 90))
        a('hd_face', '3001', WHT, T(0, -391, -98) @ R('x', 90))
        for ex in (-20, 20): a(f'hd_eye{ex}', '14769', TLB, T(ex, -391, -106) @ R('x', 90))
        a('hd_mouth', '2431', BLK, T(0, -361, -82) @ R('x', 90))
    # ---------------- backpack motor + gun arm (right side) ----------------
    if 'gunarm' in mods:
        MM = np.eye(4); MM[:3, :3] = np.column_stack([[0, -1, 0], [0, 0, 1], [-1, 0, 0]])
        a('motor', '26913', WHT, T(0, -331, 30) @ MM)
        a('ga_axle', '4519', BLK, T(70, -331, 64))                        # x 40..100 in the motor output
        # arm frame: shoulder axis at (y=-331, z=64); canonical pose points forward (-z); arm angle: 0 forward, 90 down
        A = T(0, -331, 64) @ R('x', np.degrees(arm))
        a('ga_arm', '32065', RED, A @ T(85, 0, 0) @ ux([0, 0, -1]) @ T(0, 0, 60))     # 7L thin: axle end on the motor axle
        a('ga_hand', '32000', DBG, A @ T(101, -10, -110) @ R('y', 90))
        for zz in (-100, -120): a(f'ga_pin{zz}', '32002', BLK, A @ T(90, 0, zz) @ R('y', 180))
        if gun:
            a('ga_clip', '4085c', BLK, A @ T(101, -18, -120))
            a('gun', '15391c01', BLK, A @ T(101, -8, -140))
    # ---------------- tool arm (left) ----------------
    if 'toolarm' in mods:
        y1 = hub_pt(0, 0, -120)[1]
        a('ta_arm', '32065', RED, T(-90, y1 + 60, 0) @ ux([0, 1, 0]))      # holes y1..y1+120, outside the track beam
        for zl in (-120, -100): a(f'ta_pin{zl}', '6558', BLK, T(-70, hub_pt(0, 0, zl)[1], 0))
        a('ta_hook', '32526', LBG, T(-110, y1 + 100, 0) @ ux([0, 1, 0]))
        for dy in (100, 120): a(f'ta_hpin{dy}', '2780', BLK, T(-100, y1 + dy, 0))
    return P

if __name__ == '__main__':
    import lcheck
    P = build()
    print(len(P), 'parts; ground', GROUND)
    lcheck.run(P)
