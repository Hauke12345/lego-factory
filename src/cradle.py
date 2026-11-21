"""Standing assembly: the robot hangs in a cradle on its pallet, every side free for side-pushed modules.
Robot frame as worker2/tug (x right, y down, z back); robot floor (track bottoms) at y +35.
Hanging: two 10L axles through two vertical Technic bricks on the hub's upper back (part of the body prefab) and through
the cradle's two 13L uprights. Uprights -> 3x5 L corners -> 15L arms running back -> Technic bricks on two brick columns
-> 2x16 cross plate -> 2x2 tower on the pallet's 6x16 base, which rides the main belt.
All joints on the stud/hole grid (Technic holes take the 2-LDU offset between the hub's back studs and the floor)."""
import numpy as np
import asm2
from asm2 import T, R, col3, LBG, DBG, BLK, YEL, BLU

O_Y = -151.0                          # robot origin height in the world (floor at -116 = belt pallet top -92 minus one brick)
def robot_world(px):
    """robot frame -> world: standing at x = px, facing downstream (-x), back towards +x."""
    return T(px, O_Y, 0) @ R('y', 90)

def upper_back_bricks(a):
    for x in (-30, 30): a(f'ub{x}', '3701', DBG, T(x, -311, 30) @ col3((0, 1, 0), (0, 0, 1), (1, 0, 0)))   # holes y -331, -311, -291 at z 40

BEAMV = col3((0, 0, 1), (1, 0, 0), (0, 1, 0))           # liftarm: length along y, holes along x
BEAMZ = col3((0, 1, 0), (1, 0, 0), (0, 0, -1))          # liftarm: length along z, holes along x
BASE_TOP = -92.0 - O_Y                                   # pallet base top in robot frame (+59)
def cradle(a, axles_in=1.0):
    """axles_in: 1 = hanger axles through the robot, 0 = pulled out sideways (+x) clear of the back bricks."""
    for s in (-1, 1):
        a(f'up{s}', '41239', LBG, T(90*s, -411, 40) @ BEAMV)                     # holes y -531 .. -291
        L = np.eye(4); L[:3, 0] = (0, 1, 0); L[:3, 1] = (s*-1.0, 0, 0); L[:3, 2] = (0, 0, -1)
        L[:3, 1] = np.cross(L[:3, 2], L[:3, 0])
        a(f'cor{s}', '32526', DBG, T(110*s, -531, 120) @ L)                     # corner at (z 40, y -531): short leg down, long leg back
        for y in (-531, -491): a(f'cp_u{s}_{y}', '2780', BLK, T(100*s, y, 40))
        a(f'arm{s}', '32278', LBG, T(130*s, -531, 180) @ BEAMZ)                 # holes z 40 .. 320
        for z in (80, 120): a(f'cp_a{s}_{z}', '2780', BLK, T(120*s, -531, z))
        a(f'colt{s}', '3701', DBG, T(150*s, -541, 320) @ R('y', 90))             # Technic brick on the column, holes z 300/320/340 at y -531
        for z in (300, 320): a(f'cp_c{s}_{z}', '2780', BLK, T(140*s, -531, z))
        y = -517.0
        for k in range(19): a(f'col{s}_{k}', '3004', DBG, T(150*s, y + 24*k, 320) @ R('y', 90))
        for k in range(2): a(f'colp{s}_{k}', '3023', DBG, T(150*s, y + 24*19 + 8*k, 320) @ R('y', 90))
    yc = -517.0 + 24*19 + 16                                                     # cross plate top (-45)
    a('cross', '4282', DBG, T(0, yc, 320))
    for k in range(4): a(f'tower{k}', '3003', DBG, T(0, yc + 8 + 24*k, 320))
    a('base', '3027', LBG, T(0, BASE_TOP, 220) @ R('y', 90))
    # hanger axles (10L) through uprights, upper back bricks
    for y in (-331, -291): a(f'hang{y}', '3737', BLK, T((1 - axles_in)*200, y, 40))
