"""Worker family v3: one common frame (worker2 core: upright Move Hub + tracks + head) and role kits that
pin onto the two shoulder sockets (the hub's three side holes) and the chest/back stud fields.

  LOADER  right: joint motor (small angular motor) + lift arm with a two-tine fork; left: hook arm;
          belly: Boost colour/distance sensor looking down (line following)
  GUARD   right: shoulder rail + Boost motor + six-shooter (the motor turns the trigger);
          left: shoulder rail + Boost colour/distance sensor looking forward (targeting eye)
LDraw coords, -y up, robot faces -z, ground at y = worker2.GROUND.
"""
import numpy as np
import worker2
from cell import T, R, ux, LBG, DBG, BLK, WHT, RED, YEL, BLU, ORG
AZURE, DKGRN, TRO = 322, 288, 57

def col3(a, b, c): M = np.eye(4); M[:3, 0], M[:3, 1], M[:3, 2] = a, b, c; return M

def rail(a, s, long_pin):
    """shoulder rail: bent beam 3x5 on the hub's three side holes, 5-hole leg pointing forward at shoulder height."""
    a(f'rail{s}', '32526', DBG, T(70*s, -351, 0) @ col3((0, 1, 0), (1, 0, 0), (0, 0, -1)))
    if long_pin: a(f'rail{s}_p0', '6558', BLK, T(70*s, -351, 0))          # hub + rail + brick
    else: a(f'rail{s}_p0', '2780', BLK, T(60*s, -351, 0))
    for y in (-331, -311): a(f'rail{s}_p{y}', '2780', BLK, T(60*s, y, 0))

def rail_brick(a, s):
    a(f'rb{s}', '3701', DBG, T(90*s, -361, -20) @ R('y', 90))            # Technic 1x4, studs up, holes z = 0,-20,-40
    for z in (-20, -40): a(f'rb{s}_p{z}', '2780', BLK, T(80*s, -351, z))

def build(role='loader', M0=np.eye(4), tag='w', P=None, lift=0.0, track_phase=0.0):
    P = [] if P is None else P
    mods = {'core', 'trackL', 'trackR', 'head'} | ({'toolarm'} if role == 'loader' else set())
    worker2.build(M0=M0, tag=tag, P=P, modules=mods, track_phase=track_phase)
    def a(name, part, col, M, b='worker'):
        P.append(dict(name=f'{tag}_{name}', part=part, col=col, M=M0 @ M, body=b))
    if role == 'loader':
        # ---- right shoulder: joint motor, rotor axis along +x ----
        a('jm', '68488', WHT, T(90, -331, 0) @ col3((0, 1, 0), (-1, 0, 0), (0, 0, 1)))
        for y in (-351, -311): a(f'jm_p{y}', '2780', BLK, T(60, y, 0))
        A = T(0, -331, 0) @ R('x', -lift) @ T(0, 331, 0)                  # lift: arm swings forward/up about the rotor
        a('la_arm', '40490', YEL, A @ T(150, -271, 0) @ ux([0, 1, 0]))
        for y in (-351, -311): a(f'la_rp{y}', '2780', BLK, A @ T(140, y, 0))
        Lm = col3((0, -1, 0), (-1, 0, 0), (0, 0, -1))
        for x in (130, 170): a(f'la_tine{x}', '32526', YEL, A @ T(x, -191, 0) @ Lm)
        for y in (-211, -231): a(f'la_tp{y}', '6558', BLK, A @ T(150, y, 0))
        # ---- belly sensor on the lower chest studs, eye looking down ----
        a('eye', '26912', WHT, T(0, -111, -114) @ col3((1, 0, 0), (0, 0, 1), (0, -1, 0)))
    elif role == 'guard':
        # ---- right pod: Boost motor + six-shooter ----
        rail(a, 1, True); rail_brick(a, 1)
        a('gp_plate', '3795', DBG, T(100, -369, -20) @ R('y', 90))
        a('gp_motor', '26913', WHT, T(100, -441, -20))
        a('gp_barrel', '18588', DBG, T(100, -407, -100))
        a('gp_trig', '18587', RED, T(100, -407, -100))
        for y in (-427, -387): a(f'gp_bp{y}', '2780', BLK, T(100, y, -80) @ R('y', 90))
        a('gp_axle', '4519', BLK, T(100, -407, -90) @ R('y', 90))
        # ---- left pod: targeting eye (Boost colour & distance sensor) looking forward ----
        rail(a, -1, True); rail_brick(a, -1)
        a('ge_b', '3004', DBG, T(-90, -385, 0) @ R('y', 90))
        a('ge_eye', '26912', WHT, T(-100, -409, -20))
    return P

if __name__ == '__main__':
    import sys, lcheck
    for role in ('loader', 'guard'):
        print('=====', role)
        lcheck.run(build(role))
