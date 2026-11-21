"""Track-unit line: builds the robot's track units (beams, pins, axles, sprockets, chain) on open build wagons and hands each
finished unit over to the waiting track wagon (asm2 carrier) by sliding it from the build wagon's stub guides onto the
track wagon's guides.

Canonical frame (as asm2, lane A side): x along the lanes, -y up, z across. A unit "at sx" sits where it would be on a core
at px = sx + 40 (robot frame -> canonical: asm2.core_M0(sx + 40)); on the track wagon (out position) it sits 56 further +z.
Build wagon: lane centre z = ZB (inner side, -z). Supply wagons: lane centre z = 260 (outer side, like the track wagon).
Every part reaches the unit by a sideways push along z; parts are brought by supply trains.
Side B is the mirror image (z -> -z), as in asm2.
"""
import numpy as np
import asm2, worker2, lcheck
from asm2 import T, R, col3, add, P, Y0, LBG, DBG, BLK, WHT, RED, YEL, BLU, ORG

ZB = -80.0                       # build-wagon lane centre (canonical z)
TRH = 56.0                       # handover stroke (build stubs -> track guides)
TRS = 56.0                       # supply stroke
BEAMX = col3((0, 1, 0), (0, 0, 1), (1, 0, 0))          # liftarm along x, holes along z
BEAMY = col3((-1, 0, 0), (0, 0, 1), (0, 1, 0))         # liftarm along y (vertical), holes along z
ROTZ = R('y', 90)                                      # axle (or pin) along z
lcheck.RESTING.update({'bwag': 'rolls on the rails', 'swag': 'rolls on the rails', 'unit': 'hangs on the build stubs',
                       'hplung': 'slides in the build-wagon walls', 'splung': 'slides in the supply-wagon walls',
                       'part': 'held by the supply tooling'})

STAGES = ['pin', 'vbeam', 'hbeam', 'axles', 'driveset']
_GROUPS = {'pin': ('_pin',), 'vbeam': ('_vbeam',), 'hbeam': ('_hbeam', '_pin2'), 'axles': ('_drv_ax', '_id_ax'),
           'driveset': ('_spr_', '_link')}
def unit_parts(Mu, stages, tag='u', body='unit', chain=None):
    """the unit's parts for the given stages, robot frame mapped by Mu. chain: list of link matrices (robot frame) if threading."""
    Q = []; worker2.build(M0=Mu, tag=tag, body=body, P=Q, modules={'trackL'})
    out = []
    for q in Q:
        n = q['name'][len(tag) + 4:] if False else q['name'].split('tkL', 1)[1]
        st = next((s for s, keys in _GROUPS.items() if any(n.startswith(k) for k in keys)), None)
        if n == '_pin2': st = 'hbeam'
        if st is None or st not in stages: continue
        out.append(q)
    return out

def unit_M(sx, dz=0.0): return T(0, 0, dz) @ asm2.core_M0(sx + 40)

# ---------------------------------------------------------------- column helper (stud-grid heights from the deck top -92)
def column(name, x, z, holes, top=None, body='bwag', base=-92.0, w1='3004', tb='3700', pl='3023', col=DBG):
    """1x2 column (hole centred between its studs) at (x, z) from y=base up; Technic 1x1 bricks with their hole at each height in holes (hole = top + 10)."""
    y = base; k = 0
    for h in sorted(holes, reverse=True):
        t = h - 10.0; gap = y - (t + 24)                    # space below the Technic brick
        assert gap >= -1e-6 and abs(gap % 8) < 1e-6, (name, h, gap)
        nb = int(gap // 24)
        while gap >= 24 and (gap - 24) % 8 == 0 and gap - 24 >= 0 and nb > 0:
            add(f'{name}_b{k}', w1, col, T(x, y - 24, z), body); y -= 24; gap -= 24; k += 1; nb -= 1
        while gap > 0: add(f'{name}_p{k}', pl, col, T(x, y - 8, z), body); y -= 8; gap -= 8; k += 1
        add(f'{name}_t{k}', tb, YEL, T(x, y - 24, z), body); y -= 24; k += 1
    if top is not None:
        while y - 24 >= top - 1e-6: add(f'{name}_b{k}', w1, col, T(x, y - 24, z), body); y -= 24; k += 1
        while y - 8 >= top - 1e-6: add(f'{name}_p{k}', pl, col, T(x, y - 8, z), body); y -= 8; k += 1
    return y

# ---------------------------------------------------------------- build wagon
def build_wagon(sx, eh=0.0, body='bwag', pb='hplung'):
    """eh: handover plunger travel (0 = back, TRH = unit pushed onto the track wagon's guides)."""
    a = lambda n, p, c, M, b=body: add(f'bw_{n}', p, c, M, b)
    for z in (-80, 40): a(f'deck{z}', '3027', LBG, T(sx, -92, z))
    for x in (-140, 140): a(f'tie{x}', '3832', DBG, T(sx + x, -100, -40) @ R('y', 90))      # 2x10 across both deck plates
    for zw in (-130, -90):
        for x in (-140, 140):
            y = -100.0
            for k in range(4): a(f'col{zw}_{x}_{k}', '3004', DBG if k % 2 else ORG, T(sx + x, y - 24*(k + 1), zw));
            y -= 96
            if x == -140: a(f'colt{zw}_{x}', '3700', YEL, T(sx + x, y - 24, zw))         # hole at -210 (hbeam stop rod)
            else: a(f'colt{zw}_{x}', '3004', DBG, T(sx + x, y - 24, zw))
            y -= 24
            for k in range(2): a(f'colp{zw}_{x}_{k}', '3023', DBG, T(sx + x, y - 8*(k + 1), zw))
        a(f'bar{zw}', '3703', ORG, T(sx, -260, zw))                                       # holes at y Y0, x sx-140 .. sx+140
        for k in range(2): a(f'barp{zw}_{k}', '3023', DBG, T(sx - 140, -268 - 8*k, zw))
        a(f'bart{zw}', '3700', YEL, T(sx - 140, -300, zw))                                # hole at -290 (hbeam stop rod)
    for i, dx in enumerate((0, 80)):                                                       # stub guides, tips at z 82
        a(f'stub{i}', '3708', LBG, T(sx + dx, Y0, -38) @ ROTZ)
        for z in (-115, -105, 35): a(f'sb{i}_{z}', '32123a', YEL, T(sx + dx, Y0, z))
    a('holder', '32316', DBG, T(sx + 40, Y0, 50) @ BEAMX)                                  # holds the hub pin's inner half (hole sx+60)
    for y in (-210, -290):                                                                 # hbeam stops (hbeam can't swing about pin2)
        a(f'stop{y}', '50451', BLK, T(sx - 140, y, -60) @ ROTZ)
        a(f'stopb{y}', '32123a', RED, T(sx - 140, y, 95))
    # handover plungers: pads on the vbeam's inner face (sx-40, sx-140), axle tip on the hub pin's inner end (sx+60)
    f = lambda n, p, c, M: a(n, p, c, M, pb)
    for dx in (-40, -140):
        f(f'hp{dx}', '50451', BLK, T(sx + dx, Y0, -100 + eh) @ ROTZ)                     # -260 .. 60
        f(f'hpad{dx}', '32123a', RED, T(sx + dx, Y0, 55 + eh))
    f('hp60', '50451', BLK, T(sx + 60, Y0, -120 + eh) @ ROTZ)                              # -280 .. 40
    f('hyoke', '32278', RED, T(sx - 40, Y0, -226 + eh) @ BEAMX)                            # 15L: holes sx-180 .. sx+100
    for dx in (-40, -140, 60):
        for z in (-211, -241): f(f'hyb{dx}_{z}', '32123a', YEL, T(sx + dx, Y0, z + eh))

# ---------------------------------------------------------------- supply wagons (lane centre z 260, push towards -z)
def supply_base(sx, name, body='swag'):
    add(f'{name}_deck', '3027', LBG, T(sx, -92, 260), body)

def wall_holes(sx, name, pts, body='swag', zs=(250, 310)):
    """two walls (z 250, 310) of 1x16 bricks centred at sx-20; Technic 1x16 layers (holes sx-160 .. sx+120) at the plunger heights."""
    hs = sorted({y for _, y in pts}, reverse=True); xc = sx - 20
    for z in zs:
        y = -92.0; k = 0
        for h in hs:
            t = h - 10.0; gap = y - (t + 24)
            assert gap >= -1e-6 and abs(gap % 8) < 1e-6, (name, h, gap)
            while gap >= 24 and (gap - 24) % 8 == 0:
                add(f'{name}_w{z}_{k}', '2465', DBG if k % 2 else LBG, T(xc, y - 24, z), body); y -= 24; gap -= 24; k += 1
            while gap > 0:
                for dx in (-80, 80): add(f'{name}_w{z}_{k}_{dx}', '3460', DBG, T(xc + dx, y - 8, z), body)
                y -= 8; gap -= 8; k += 1
            add(f'{name}_w{z}_{k}', '3703', YEL, T(xc, y - 24, z), body); y -= 24; k += 1

def supply_pin(sx, e, body='swag', pb='splung'):
    """hub pin: outer half in a thin liftarm on the plungers; pushed into the build wagon's holder."""
    n = 'sp0'; supply_base(sx, n, body)
    wall_holes(sx, n, [(20, Y0), (100, Y0)], body)
    f = lambda nn, p, c, M: add(f'{n}_{nn}', p, c, M, pb)
    f('hold', '32065', DBG, T(sx + 60, Y0, 75 + e) @ BEAMX)                               # thin 7L: holes sx .. sx+120
    for dx in (20, 100):
        f(f'pl{dx}', '50451', BLK, T(sx + dx, Y0, 220 + e) @ ROTZ)                         # 60 .. 380
        for z in (65, 85): f(f'b{dx}_{z}', '32123a', YEL, T(sx + dx, Y0, z + e))
        for z in (345, 375): f(f'yb{dx}_{z}', '32123a', YEL, T(sx + dx, Y0, z + e))
    f('yoke', '40490', RED, T(sx + 60, Y0, 360 + e) @ BEAMX)

def supply_vbeam(sx, e, body='swag', pb='splung'):
    n = 'sp1'; supply_base(sx, n, body)
    wall_holes(sx, n, [(40, Y0), (-60, Y0)], body)
    f = lambda nn, p, c, M: add(f'{n}_{nn}', p, c, M, pb)
    for dx in (40, -60):
        f(f'pl{dx}', '50451', BLK, T(sx + dx, Y0, 222 + e) @ ROTZ)                         # tip 62
        f(f'pad{dx}', '32123a', RED, T(sx + dx, Y0, 85 + e))
        for z in (355, 385): f(f'yb{dx}_{z}', '32123a', YEL, T(sx + dx, Y0, z + e))
    f('yoke', '32524', RED, T(sx - 20, Y0, 370 + e) @ BEAMX)                               # 7L: sx-80 .. sx+40

def supply_hbeam(sx, e, body='swag', pb='splung'):
    n = 'sp2'; supply_base(sx, n, body)
    wall_holes(sx, n, [(-160, -290), (-160, -330)], body)
    f = lambda nn, p, c, M: add(f'{n}_{nn}', p, c, M, pb)
    for y in (-290, -330):
        f(f'pl{y}', '50451', BLK, T(sx - 160, y, 242 + e) @ ROTZ)                          # tip 82
        f(f'pad{y}', '32123a', RED, T(sx - 160, y, 105 + e))
        for z in (355, 385): f(f'yb{y}_{z}', '32123a', YEL, T(sx - 160, y, z + e))
    f('yoke', '32316', RED, T(sx - 160, -290, 370 + e) @ BEAMY)                           # 5L: -330 .. -250 (lower end in the pusher face band)

def supply_axles(sx, e, body='swag', pb='splung'):
    """each axle's outer end sits in a round-hole sleeve, end to end with a coaxial plunger tip in the same hole. The idler
    sleeve (9L) and the drive sleeve (2L) are pinned to a 5L connector in front; two tie plungers clamp the idler sleeve
    (bushes both sides) so the sleeves go out and come back with the yoke. Round holes: the axles stay put."""
    n = 'sp3'; supply_base(sx, n, body)
    coax = [(-160, -330), (-160, -170), (-100, Y0)]; ties = [(-160, -290), (-160, -210)]
    wall_holes(sx, n, coax + ties, body)
    f = lambda nn, p, c, M: add(f'{n}_{nn}', p, c, M, pb)
    for x, y in coax:
        f(f'pc{x}_{y}', '3708', BLK, T(sx + x, y, 280 + e) @ ROTZ)                         # 160 .. 400 (tip inside the sleeve hole)
        f(f'pcb{x}_{y}', '32123a', YEL, T(sx + x, y, 175 + e))
    for x, y in ties:
        f(f'pt{x}_{y}', '3708', BLK, T(sx + x, y, 260 + e) @ ROTZ)                         # 140 .. 380
        for z in (145, 175): f(f'ptb{x}_{y}_{z}', '32123a', YEL, T(sx + x, y, z + e))
    f('sleeveI', '40490', DBG, T(sx - 160, -250, 160 + e) @ BEAMY)                         # 9L vertical: holes -330 .. -170
    f('sleeveD', '43857', DBG, T(sx - 90, Y0, 160 + e) @ BEAMX)                            # 2L: holes sx-100, sx-80
    f('sconn', '32316', DBG, T(sx - 120, Y0, 140 + e) @ BEAMX)                             # 5L in front: sx-160 .. sx-80
    for x in (-160, -80): f(f'scpin{x}', '2780', BLK, T(sx + x, Y0, 150 + e) @ ROTZ)
    f('yokeI', '40490', RED, T(sx - 160, -250, 350 + e) @ BEAMY)
    f('yokeD', '32316', RED, T(sx - 120, Y0, 380 + e) @ BEAMX)                             # 5L behind: sx-160 .. sx-80
    f('ypin', '6558', BLK, T(sx - 160, Y0, 360 + e) @ ROTZ)
    for x, y in coax[:2] + ties:
        for z in (335, 365): f(f'yb{x}_{y}_{z}', '32123a', YEL, T(sx + x, y, z + e))
    for z in (365, 395): f(f'ybD_{z}', '32123a', YEL, T(sx - 100, Y0, z + e))

def supply_sprockets(sx, e, body='swag', pb='splung', name='sp4'):
    """drive set: the three sprockets with the closed chain already round them (prefab from the chain station), sitting on
    fixed coaxial guides (tips 2 LDU off the axle ends). Round-hole 2L pads inside the chain loop, pinned to a frame
    behind it, push the set off the guides onto the axles; one plunger moves the frame (bushes both sides)."""
    n = name; supply_base(sx, n, body)
    G = [(-160, -330), (-160, -170), (-100, Y0)]
    wall_holes(sx, n, G + [(-140, Y0)], body, zs=(290, 310))
    for x, y in G:
        add(f'{n}_g{x}_{y}', '3737', LBG, T(sx + x, y, 262) @ ROTZ, body)                    # 10L: tip 162 .. 362
        for z in (275, 325): add(f'{n}_gb{x}_{y}_{z}', '32123a', YEL, T(sx + x, y, z), body)
    f = lambda nn, p, c, M: add(f'{n}_{nn}', p, c, M, pb)
    f('padU', '43857', DBG, T(sx - 160, -320, 156 + e) @ BEAMY)                            # holes -330 (guide), -310 (pin)
    f('padL', '43857', DBG, T(sx - 160, -180, 156 + e) @ BEAMY)                            # holes -190 (pin), -170 (guide)
    f('padD', '43857', DBG, T(sx - 110, Y0, 156 + e) @ BEAMX)                              # holes sx-120 (pin), sx-100 (guide)
    f('frV', '40490', RED, T(sx - 160, -250, 176 + e) @ BEAMY)                             # 9L: -330 .. -170
    f('frH', '32316', RED, T(sx - 120, Y0, 196 + e) @ BEAMX)                               # 5L: sx-160 .. sx-80
    for y in (-310, -190): f(f'pp{y}', '2780', BLK, T(sx - 160, y, 166 + e) @ ROTZ)
    f('ppVH', '2780', BLK, T(sx - 160, Y0, 186 + e) @ ROTZ)
    f('ppD', '6558', BLK, T(sx - 120, Y0, 176 + e) @ ROTZ)
    f('pl', '23948', BLK, T(sx - 140, Y0, 286 + e) @ ROTZ)                                 # 11L: 176 .. 396
    for z in (181, 211, 365, 395): f(f'plb{z}', '32123a', YEL, T(sx - 140, Y0, z + e))

SUPPLY = {'pin': supply_pin, 'vbeam': supply_vbeam, 'hbeam': supply_hbeam, 'axles': supply_axles, 'driveset': supply_sprockets}
def station_state(sx, stage, e, unit_body='unit'):
    """build wagon at sx with all stages before `stage` done, the stage's parts e out on the supply wagon."""
    k = STAGES.index(stage)
    for q in unit_parts(unit_M(sx), STAGES[:k], body=unit_body): P.append(q)
    for q in unit_parts(unit_M(sx, e), [stage], tag='p', body='part' if e > 0 else unit_body):
        if e > 0:
            q['body'] = 'part_' + ('dset' if stage == 'driveset' else q['name']); lcheck.RESTING[q['body']] = 'held by the supply tooling'
        P.append(q)
    build_wagon(sx)
    SUPPLY[stage](sx, e)

def handover_state(sx, eh, unit_body='unit'):
    """finished unit slid eh along +z from the build stubs towards the track wagon (asm2 carrier, plungers out)."""
    for q in unit_parts(unit_M(sx, eh), STAGES, body=unit_body): P.append(q)
    build_wagon(sx, eh)
    asm2.carrier('A', sx, asm2.TR)

def station_test(sx, stage, e, theta, XM, ZM, rc=60):
    station_state(sx, stage, e)
    asm2.pusher('A', theta, XM=XM, ZM=ZM, tag='T', rc=rc)
