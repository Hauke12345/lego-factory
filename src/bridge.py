"""Lego truss bridge over the box -> book gap, built from real part types with real connections.

World (Blender) coords in metres: x across the road, vehicle travels towards -y, z up, table at z=0.
Structure
  approach (on the box) : plate layer A1, layer A2 = row of click-hinge plates (2 fingers) + plate,
                          tile layer A3, curved-slope lip
  deck                  : layer D1 = click-hinge plates (1 finger) at both ends + two 6x14 plates,
                          tiles 2x6, Technic bricks 1x16 as curbs, Technic brick 1x14 top chords,
                          thin liftarms (posts 1x5, diagonals 1x6, 3-4-5 triangles) pinned into the holes
  landing (on the book) : mirror of the approach
  pier (on the table)   : plate 4x8 + 7 bricks 2x8 (= 22 plates) + hinge bricks 1x2 whose top plates
                          clip under the deck (deck stud row 8)
Pure-python geometry is importable without bpy (MuJoCo sim, HUD)."""
import math

S, P, B, GAP = 0.008, 0.0032, 0.0096, 0.0001
SC, T0 = 0.044, -6.55
Z_BOX = SC*(-4.348-T0); Z_BOOK = SC*(-5.262-T0)
Y_E = SC*0.539          # box edge
Y_K = SC*-2.637         # book edge
YA = Y_E - 0.5*S; ZA = Z_BOX + 1.5*P       # upper hinge axis
LH = 19*S                                   # axis-to-axis deck length
DZ = ZA - (Z_BOOK + 1.5*P)
SIN = DZ/LH; TH = math.asin(SIN); COS = math.cos(TH); TAN = math.tan(TH)
YL = YA - LH*COS; ZL = Z_BOOK + 1.5*P       # lower hinge axis
Y_LE = YL - 0.5*S                           # landing plate edge (on the book)
ROAD = Z_BOX + 3*P                          # approach road (tile) top
ROAD_L = Z_BOOK + 3*P
U_PIER = 8*S
N_PIER = 7
CURB_V = 5.5*S
POST_U = [3.5*S, 6.5*S, 9.5*S, 12.5*S, 15.5*S]
W_HOLE = P/2 + 0.0058                       # curb hole height above the deck axis plane
POST_H = 4*S                                # 1x5 liftarm, 32 mm between end holes
HINGE_COLS = (1, 2, 4, 5, 6, 7, 9, 10)
def _filler_runs():
    runs, cur = [], []
    for c in range(12):
        if c in HINGE_COLS:
            if cur: runs.append(cur); cur = []
        else: cur.append(c)
    if cur: runs.append(cur)
    return [((r[0] + r[-1])/2 - 5.5, len(r)) for r in runs]     # (centre in studs, width)
FILLERS = _filler_runs()

def deck_point(u, v=0.0, w=0.0):
    """deck-local (u along span downhill from the upper axis, v across = world x, w normal) -> world xyz"""
    return (v, YA - u*COS - w*SIN, ZA - u*SIN + w*COS)

HK = 0.0025                                 # hinge knuckle axis offset below the top plate
Q_PIER = deck_point(U_PIER, 0, -1.5*P - HK) # pier hinge axis
Y_PIER = Q_PIER[1] + 0.5*S                  # pier centre (hinge sits on its -y stud row)

def road_z(y):
    """drivable top along the centre line (ignores lips)."""
    y0 = deck_point(0, 0, 1.5*P)[1]; y1 = deck_point(LH, 0, 1.5*P)[1]
    if y >= y0: return ROAD
    if y <= y1: return ROAD_L
    return ROAD - (y0 - y)*TAN


def build(lego):
    """Returns [(obj, world_matrix, group)] in assembly order."""
    import bpy, bmesh
    from mathutils import Matrix, Vector
    import truck
    _cyl, _fin = lego._add_cyl, lego._finish
    D = Matrix.Translation((0, YA, ZA)) @ Matrix.Rotation(-math.pi/2, 4, 'Z') @ Matrix.Rotation(TH, 4, 'Y')
    LG, DG, YE, BK = lego.LGRAY(), lego.DGRAY(), lego.YELLOW(), lego.BLACK()
    parts = []
    def add(ob, M, g): parts.append((ob, M, g)); return ob
    def W(x, y, z, rz=0.0): return Matrix.Translation((x, y, z)) @ Matrix.Rotation(rz, 4, 'Z')
    def Dk(u, v, w, rz=0.0, M2=None):
        M = D @ Matrix.Translation((u, v, w)) @ Matrix.Rotation(rz, 4, 'Z')
        return M @ M2 if M2 is not None else M
    plate = lambda n, a, b, m: lego.brick(n, a, b, P, m)
    tile = lambda n, a, b, m: lego.brick(n, a, b, P, m, studs=False)

    def hinge_plate(name, fingers, m):
        """Click hinge plate 1x2: 2 studs along local X, finger(s) beyond the +X end, axis at x=1.5S, z=P/2."""
        bm = bmesh.new()
        bmesh.ops.create_cube(bm, size=1, matrix=Matrix.Translation((0, 0, P/2)) @ Matrix.Diagonal((2*S-2*GAP, S-2*GAP, P, 1)))
        lego._studs(bm, 2, 1, P); lego.anti_grid(2, 1)
        lego._meta['axes'].append(('click%d' % fingers, (1.5*S, 0, P/2), (0, 1, 0), 0.004))
        ys = (0.0,) if fingers == 1 else (-0.0026, 0.0026)
        wd = 0.0028 if fingers == 1 else 0.0022
        for y in ys:
            bmesh.ops.create_cube(bm, size=1, matrix=Matrix.Translation((1.25*S, y, P/2)) @ Matrix.Diagonal((0.5*S, wd, P*0.8, 1)))
            _cyl(bm, Vector((1.5*S, y, P/2)), P/2, wd, 14, 'Y')
        return _fin(bm, name, m, 0.0002)

    def curved_slope(name, nx, ny, h, m):
        """Slope curved nx x ny, height h, high end at -X falling to +X."""
        bm = bmesh.new(); x0, x1 = -nx*S/2 + GAP, nx*S/2 - GAP; y0, y1 = -ny*S/2 + GAP, ny*S/2 - GAP
        prof = [(x0, 0), (x1, 0), (x1, 0.0006)]
        for k in range(1, 12):
            t = 1 - k/12; prof.append((x0 + (x1-x0)*t, 0.0006 + (h - 0.0006)*(0.5 - 0.5*math.cos(math.pi*(1 - t)))))
        prof.append((x0, h))
        v0 = [bm.verts.new((x, y0, z)) for x, z in prof]; v1 = [bm.verts.new((x, y1, z)) for x, z in prof]
        bm.faces.new(v0[::-1]); bm.faces.new(v1)
        for i in range(len(prof)):
            j = (i+1) % len(prof); bm.faces.new((v0[i], v0[j], v1[j], v1[i]))
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
        lego.anti_grid(nx, ny)
        return _fin(bm, name, m, 0.0002)

    def liftarm(name, n, m):
        """Thin liftarm 1xn (4 mm thick), along local X, centred, holes along X, thickness along Y."""
        bm = bmesh.new(); ln = (n - 1)*S; r = 0.0036
        bmesh.ops.create_cube(bm, size=1, matrix=Matrix.Diagonal((ln, 0.004, 2*r, 1)))
        for e in (-ln/2, ln/2): _cyl(bm, Vector((e, 0, 0)), r, 0.004, 20, 'Y')
        lego._meta['holes'] += [(-ln/2 + k*S, 0, 0) for k in range(n)]
        ob = _fin(bm, name, m, 0.0003)
        bm = bmesh.new()
        for k in range(n):
            for sy in (-1, 1): _cyl(bm, Vector((-ln/2 + k*S, sy*0.00205, 0)), 0.0024, 0.0002, 14, 'Y')
        h = _fin(bm, name + '_holes', BK, 0); h.parent = ob
        return ob

    def half_bush(name, m):
        bm = bmesh.new(); _cyl(bm, Vector((0, 0, 0)), 0.0034, 0.0038, 20, 'Y')
        lego._meta['holes'].append((0, 0, 0))
        return _fin(bm, name, m, 0.0003)

    def pin_head(name, m):
        bm = bmesh.new(); _cyl(bm, Vector((0, 0, 0)), 0.0031, 0.0009, 16, 'Y'); _cyl(bm, Vector((0, 0.0004, 0)), 0.0024, 0.0012, 16, 'Y')
        return _fin(bm, name, m, 0.0002)

    def hinge_base(name, m):
        """Hinge brick 1x2 base: 2 along X, knuckle (axis along X) on top, axis at z=B."""
        bm = bmesh.new()
        hb = B - 2*HK
        bmesh.ops.create_cube(bm, size=1, matrix=Matrix.Translation((0, 0, hb/2)) @ Matrix.Diagonal((2*S-2*GAP, S-2*GAP, hb, 1)))
        for x in (-0.0045, 0.0045):
            _cyl(bm, Vector((x, 0, B - HK)), 0.0024, 0.005, 16, 'X')
            bmesh.ops.create_cube(bm, size=1, matrix=Matrix.Translation((x, 0, hb + 0.0006)) @ Matrix.Diagonal((0.005, 0.003, 0.0016, 1)))
        lego.anti_grid(2, 1); lego._meta['axes'].append(('hingeB', (0, 0, B - HK), (1, 0, 0), 0.007))
        return _fin(bm, name, m, 0.0003)

    def hinge_top(name, m):
        """Hinge plate 1x2 top: plate (2 along X) with a knuckle below, axis at z=-0.0? (plate bottom at z=0)."""
        bm = bmesh.new()
        bmesh.ops.create_cube(bm, size=1, matrix=Matrix.Translation((0, 0, HK + P/2)) @ Matrix.Diagonal((2*S-2*GAP, S-2*GAP, P, 1)))
        _cyl(bm, Vector((0, 0, 0)), 0.0024, 0.0035, 16, 'X')
        bmesh.ops.create_cube(bm, size=1, matrix=Matrix.Translation((0, 0, HK/2 + 0.0004)) @ Matrix.Diagonal((0.0035, 0.003, HK, 1)))
        lego._meta['studs'] += [(-S/2, 0, HK + P), (S/2, 0, HK + P)]; lego._meta['axes'].append(('hingeT', (0, 0, 0), (1, 0, 0), 0.002))
        return _fin(bm, name, m, 0.0003)

    # ---------------- pier ----------------
    add(plate('pierFoot', 8, 4, LG), W(0, Y_PIER, 0), 'pier')
    for k in range(N_PIER):
        add(lego.brick(f'pier{k}', 8, 2, B, LG), W(0, Y_PIER, P + k*B), 'pier')
    zt = P + N_PIER*B
    for k, x in enumerate((-3*S, 3*S)):
        add(hinge_base(f'pierHinge{k}', LG), W(x, Y_PIER - 0.5*S, zt), 'pier')
    # ---------------- approach (box) & landing (book) ----------------
    def abutment(tag, y_edge, z0, sgn):
        """sgn=+1: structure extends towards +y from the edge (approach); -1 towards -y (landing)."""
        rz_h = -math.pi/2 if sgn > 0 else math.pi/2          # hinge finger points back over the gap
        rz_s = math.pi/2 if sgn > 0 else -math.pi/2          # lip falls away from the bridge
        Y = lambda d: y_edge + sgn*d
        add(plate(f'{tag}A1a', 12, 6, DG), W(0, Y(3*S), z0), tag)
        add(plate(f'{tag}A1b', 12, 4, DG), W(0, Y(8*S), z0), tag)
        for c in HINGE_COLS:
            add(hinge_plate(f'{tag}H{c}', 2, LG), W((c - 5.5)*S, Y(S), z0 + P, rz_h), tag)
        for k, (x, n) in enumerate((c*S, n) for c, n in FILLERS):
            add(plate(f'{tag}F{k}', n, 2, DG), W(x, Y(S), z0 + P), tag)
        add(plate(f'{tag}A2', 12, 6, DG), W(0, Y(5*S), z0 + P), tag)
        for k, x in enumerate((-3*S, 3*S)):
            add(tile(f'{tag}T{k}', 6, 2, DG), W(x, Y(S), z0 + 2*P), tag + 'Top')
            add(tile(f'{tag}T{k+2}', 6, 6, DG), W(x, Y(5*S), z0 + 2*P), tag + 'Top')
        for k in range(6):
            add(curved_slope(f'{tag}L{k}', 2, 2, 2*P, DG), W((-5 + 2*k)*S, Y(9*S), z0 + P, rz_s), tag + 'Top')
    abutment('app', Y_E, Z_BOX, +1)
    abutment('land', Y_LE, Z_BOOK, -1)
    # ---------------- deck ----------------
    for k, x in enumerate((-3*S, 3*S)):
        add(hinge_top(f'pierTop{k}', LG), Dk(U_PIER, x, -1.5*P - HK, math.pi/2), 'deck')
    for end, (u, rz) in enumerate(((1.5*S, math.pi), (17.5*S, 0.0))):
        for c in HINGE_COLS:
            add(hinge_plate(f'dH{end}{c}', 1, LG), Dk(u, (c - 5.5)*S, -P/2, rz), 'deck')
        for k, (v, n) in enumerate((c*S, n) for c, n in FILLERS):
            add(plate(f'dF{end}{k}', 2, n, LG), Dk(u, v, -P/2), 'deck')
    for k, v in enumerate((-3*S, 3*S)):
        add(plate(f'deck{k}', 14, 6, LG), Dk(9.5*S, v, -P/2), 'deck')
    for i, u in enumerate((3.5*S, 9.5*S, 15.5*S)):
        for j, v in enumerate((-4*S, -2*S, 0, 2*S, 4*S)):
            add(tile(f'road{i}{j}', 6, 2, DG), Dk(u, v, P/2), 'road')
    for k, v in enumerate((-CURB_V, CURB_V)):
        add(truck.technic_brick(f'curb{k}', 16), Dk(9.5*S, v, P/2), 'curb')
    # truss: posts (inner layer) and diagonals (outer layer), pins with heads on the outside
    lo, hi = W_HOLE, W_HOLE + POST_H
    for side, sv in enumerate((-1, 1)):
        vp = sv*(CURB_V + S/2 + 0.002); vd0 = sv*(CURB_V + S/2 + 0.006); vd1 = sv*(CURB_V + S/2 + 0.010)
        for k, u in enumerate(POST_U):
            po = add(liftarm(f'post{k}{side}', 5, YE), Dk(u, vp, (lo + hi)/2, 0, Matrix.Rotation(-math.pi/2, 4, 'Y')), 'post')
            for w in (lo, hi):
                # outermost layer at this hole: post (4 mm), even-bay diagonal (8 mm), odd-bay diagonal (12 mm)
                outer = 0.004
                for kb in range(4):
                    ends = ((POST_U[kb], lo), (POST_U[kb+1], hi)) if kb % 2 == 0 else ((POST_U[kb], hi), (POST_U[kb+1], lo))
                    if any(abs(eu - u) < 1e-6 and abs(ew - w) < 1e-6 for eu, ew in ends):
                        outer = max(outer, 0.008 if kb % 2 == 0 else 0.012)
                vh = sv*(CURB_V + S/2 + outer + 0.0005)
                ph = pin_head(f'pin{k}{side}{int(w == hi)}', lego.mat('pinblue', lego.srgb('0A3463')))
                rot = Matrix.Rotation(math.pi if sv < 0 else 0, 4, 'Z')
                ph.parent = po
                ph.matrix_basis = (Matrix.Rotation(-math.pi/2, 4, 'Y')).inverted() @ Matrix.Translation((0, vh - vp, w - (lo + hi)/2)) @ rot
        for k in range(4):
            u0, u1 = POST_U[k], POST_U[k+1]
            a, b = ((u0, lo), (u1, hi)) if k % 2 == 0 else ((u0, hi), (u1, lo))
            ang = math.atan2(b[1] - a[1], b[0] - a[0])
            add(liftarm(f'diag{k}{side}', 6, YE), Dk((a[0] + b[0])/2, vd0 if k % 2 == 0 else vd1, (a[1] + b[1])/2, 0, Matrix.Rotation(-ang, 4, 'Y')), 'diag')
        # the last diagonal sits in the outer layer: a half bush fills the empty inner layer on its pin
        add(half_bush(f'bush{side}', LG), Dk(POST_U[4], vd0, lo), 'diag')
    for k, v in enumerate((-CURB_V, CURB_V)):
        add(truck.technic_brick(f'chord{k}', 14), Dk(9.5*S, v, P/2 + POST_H), 'chord')
    return parts
