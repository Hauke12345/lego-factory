"""Lego pickup truck built only from real part types with real connections.
Car-local coords: X forward, Y left, Z up, ground at z=0.
Running gear: two Technic bricks 1x14 under the chassis, Technic axles 8L through their holes,
wheels pushed onto the axle ends (outside the 6-wide body), so nothing floats."""
import bpy, bmesh, math
from mathutils import Vector, Matrix
import lego
from lego import S, P, B, GAP, _add_cyl, _finish, brick, slope, round1x1, minifig

WR = 0.0145                 # tyre radius (30.4 mm tyre)
WW = 0.009                  # tyre width
HOLE_Z = 0.0058             # Technic hole centre above the brick bottom
Z_TB = WR - HOLE_Z          # bottom of the Technic bricks
Z_CH = Z_TB + B             # bottom of the chassis plate
Z1 = Z_CH + P               # top of chassis plate (layer 1 base)
AX_F, AX_R = 5*S, -4*S      # axle x positions (on Technic hole centres)
WY = 3*S + 0.0015 + WW/2    # wheel centre |y|
BLUE = lego.BLUE
TORANGE = lambda: lego.mat('torange', lego.srgb('F08F1C'), trans=0.6, emit=0.6)
GREEN = lambda: lego.mat('green', lego.srgb('237841'))

def xi(i): return (i - 7.5)*S          # x of stud column i (0 = rear)

def technic_brick(name, n):
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1, matrix=Matrix.Translation((0, 0, B/2)) @ Matrix.Diagonal((n*S-2*GAP, S-2*GAP, B, 1)))
    lego._studs(bm, n, 1, B); lego.anti_grid(n, 1)
    lego._meta['holes'] += [((k - n/2)*S, 0, HOLE_Z) for k in range(1, n)]
    ob = _finish(bm, name, lego.LGRAY())
    bm = bmesh.new()                       # visible pin holes: dark discs on both faces
    for k in range(1, n):
        for sy in (-1, 1):
            _add_cyl(bm, Vector(((k - n/2)*S, sy*(S/2 - GAP + 0.00005), HOLE_Z)), 0.0024, 0.0003, 16, 'Y')
    h = _finish(bm, name + '_holes', lego.BLACK(), 0); h.parent = ob
    return ob

def axle(name, n=8):
    bm = bmesh.new(); Ltot = n*S
    for rot in (0, math.pi/2):
        bmesh.ops.create_cube(bm, size=1, matrix=Matrix.Rotation(rot, 4, 'Y') @ Matrix.Diagonal((0.0018, Ltot, 0.0048, 1)))
    lego._meta['axes'].append(('axle', (0, 0, 0), (0, 1, 0), Ltot/2))
    return _finish(bm, name, lego.BLACK(), 0.0002)

def wheel(name):
    bm = bmesh.new(); _add_cyl(bm, Vector((0, 0, 0)), WR, WW, 36, 'Y')
    lego._meta['axes'].append(('hub', (0, 0, 0), (0, 1, 0), WW/2))
    ob = _finish(bm, name, lego.BLACK(), 0.002)
    bm = bmesh.new(); _add_cyl(bm, Vector((0, 0, 0)), 0.0088, WW + 0.0006, 24, 'Y')
    rim = _finish(bm, name + '_rim', lego.LGRAY(), 0.0004); rim.parent = ob
    bm = bmesh.new(); _add_cyl(bm, Vector((0, 0, 0)), 0.0032, WW + 0.0012, 12, 'Y')
    hub = _finish(bm, name + '_hub', lego.DGRAY(), 0.0002); hub.parent = ob
    return ob

def windscreen(name, ny=6):
    """Windscreen 2x6x2: thin trans-clear glass leaning back from the front edge, with a base strip."""
    bm = bmesh.new()
    x0, x1, h = S - 0.0016, -S + 0.0016, 2*B
    th = math.atan2(h, x0 - x1); ln = math.hypot(h, x0 - x1) - 0.0034
    M = Matrix.Translation(((x0 + x1)/2, 0, h/2)) @ Matrix.Rotation(-th, 4, 'Y') @ Matrix.Diagonal((ln, ny*S - 2*GAP, 0.0013, 1))
    bmesh.ops.create_cube(bm, size=1, matrix=Matrix.Rotation(0, 4, 'Z') @ M)
    lego.anti_grid(2, ny)
    ob = _finish(bm, name, lego.TCLEAR(), 0.0003)
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1, matrix=Matrix.Translation((S*0.6, 0, P/2)) @ Matrix.Diagonal((0.8*S, ny*S - 2*GAP, P, 1)))
    base = _finish(bm, name + '_base', lego.TCLEAR(), 0.0002); base.parent = ob
    return ob

def window(name, nx=3, h=2*B, mat=None):
    """Window 1xNx2 frame (side-on) with trans-clear glass."""
    mat = mat or BLUE()
    bm = bmesh.new(); t = 0.0016; Lx = nx*S - 2*GAP; Wy = S - 2*GAP
    for M in (Matrix.Translation((0, 0, t/2)) @ Matrix.Diagonal((Lx, Wy, t, 1)),
              Matrix.Translation((0, 0, h - t/2)) @ Matrix.Diagonal((Lx, Wy, t, 1)),
              Matrix.Translation((Lx/2 - t/2, 0, h/2)) @ Matrix.Diagonal((t, Wy, h, 1)),
              Matrix.Translation((-Lx/2 + t/2, 0, h/2)) @ Matrix.Diagonal((t, Wy, h, 1))):
        bmesh.ops.create_cube(bm, size=1, matrix=M)
    lego._studs(bm, nx, 1, h); lego.anti_grid(nx, 1)
    ob = _finish(bm, name, mat, 0.0003)
    bm = bmesh.new(); bmesh.ops.create_cube(bm, size=1, matrix=Matrix.Translation((0, 0, h/2)) @ Matrix.Diagonal((Lx - 2*t, 0.0012, h - 2*t, 1)))
    g = _finish(bm, name + '_glass', lego.TCLEAR(), 0); g.parent = ob
    return ob

def tile(name, nx, ny, mat): return brick(name, nx, ny, P, mat, studs=False)

def build_truck():
    """Returns (parts, wheels, bom). parts = [(obj, local_matrix)] in assembly order."""
    parts = []
    def add(ob, x, y, z, rz=0.0):
        parts.append((ob, Matrix.Translation((x, y, z)) @ Matrix.Rotation(rz, 4, 'Z'))); return ob
    # running gear
    for k, y in enumerate((2.5*S, -2.5*S)):
        add(technic_brick(f'tbrick{k}', 14), 0, y, Z_TB)
    for k, x in enumerate((AX_F, AX_R)):
        add(axle(f'axle{k}', 8), x, 0, WR)
    wheels = []
    for k, (x, y) in enumerate([(AX_F, WY), (AX_F, -WY), (AX_R, WY), (AX_R, -WY)]):
        wheels.append(add(wheel(f'wheel{k}'), x, y, WR))
    # chassis
    add(brick('chassis', 16, 6, P, lego.DGRAY()), 0, 0, Z_CH)
    z2, z3, z4 = Z1 + B, Z1 + 2*B, Z1 + 3*B
    # front: bumper, engine, hood
    add(brick('bumper', 1, 6, B, lego.LGRAY()), xi(15), 0, Z1)
    add(brick('engine', 2, 6, B, BLUE()), (xi(13) + xi(14))/2, 0, Z1)
    add(slope('hood', 2, 6, BLUE(), flat=S*0.3), (xi(13) + xi(14))/2, 0, z2)
    add(tile('grille', 1, 2, lego.LGRAY()), xi(15), 0, z2)
    for k, y in enumerate((2.5*S, -2.5*S)):
        add(round1x1(f'head{k}', lego.TYELLOW()), xi(15), y, z2)
    # cab
    add(brick('dash', 2, 6, B, BLUE()), (xi(11) + xi(12))/2, 0, Z1)
    add(windscreen('windscreen'), (xi(11) + xi(12))/2, 0, z2)
    add(minifig('driver'), xi(9), 1*S, Z1)
    for k, y in enumerate((2.5*S, -2.5*S)):
        add(brick(f'cabside{k}', 3, 1, B, BLUE()), xi(9), y, Z1)
        add(window(f'window{k}', 3, 2*B), xi(9), y, z2)
    for k, z in enumerate((Z1, z2, z3)):
        add(brick(f'cabback{k}', 1, 6, B, BLUE()), xi(7), 0, z)
    add(brick('roof', 6, 6, P, lego.WHITE()), (xi(7) + xi(12))/2, 0, z4)
    for k, y in enumerate((1.5*S, -1.5*S)):
        add(round1x1(f'beacon{k}', TORANGE(), P), xi(9), y, z4 + P)
    # cargo bed
    for z in (Z1, z2):
        for k, y in enumerate((2.5*S, -2.5*S)):
            add(brick(f'bed{k}_{int(z == z2)}', 6, 1, B, BLUE()), (xi(1) + xi(6))/2, y, z)
        add(brick(f'tailgate{int(z == z2)}', 1, 6, B, BLUE()), xi(0), 0, z)
    for k, y in enumerate((2.5*S, -2.5*S)):
        add(round1x1(f'tail{k}', lego.TRED()), xi(0), y, z3)
    # cargo: a few loose Lego bricks
    add(brick('cargo0', 2, 4, B, lego.RED()), (xi(3) + xi(4))/2, 0, Z1)
    add(brick('cargo1', 2, 2, B, lego.YELLOW()), (xi(1) + xi(2))/2, -1*S, Z1)
    add(brick('cargo2', 2, 2, B, GREEN()), (xi(3) + xi(4))/2, 0, z2)
    bom = [('Technic brick 1×14', 2, 'light grey'), ('Technic axle 8', 2, 'black'), ('Wheel + tyre 30.4', 4, 'black/grey'),
           ('Plate 6×16', 1, 'dark grey'), ('Brick 1×6', 1, 'light grey'), ('Brick 2×6', 2, 'blue'),
           ('Slope 2×6', 1, 'blue'), ('Tile 1×2 grille', 1, 'light grey'), ('Round plate 1×1', 2, 'trans-yellow'),
           ('Windscreen 2×6×2', 1, 'trans-clear'), ('Minifigure', 1, 'blue'),
           ('Brick 1×3', 2, 'blue'), ('Window 1×3×2', 2, 'blue'), ('Brick 1×6', 5, 'blue'), ('Plate 6×6', 1, 'white'),
           ('Round plate 1×1', 2, 'trans-orange'), ('Brick 1×6 (bed)', 4, 'blue'), ('Round plate 1×1', 2, 'trans-red'),
           ('Brick 2×4 / 2×2', 3, 'red/yellow/green')]
    return parts, wheels, bom
