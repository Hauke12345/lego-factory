"""Worker robot ("gunner" variant) from real LDraw parts, in its own frame (LDraw coords, -y up):
Move Hub flat, drive wheels on the hub's motor outputs (z=+140 end), skid under the other end,
Boost motor as torso (its cable goes to hub port C), stud blaster on the right arm.
The robot faces -z. Ground at y = GROUND."""
import numpy as np
import cell
from cell import T, R, ux, LBG, DBG, BLK, WHT, RED, YEL, BLU, ORG

WR = 71                         # tyre radius (55976)
GROUND = 66 + WR                # hub output height + tyre radius = 137
TLB = 43                        # trans light blue (eyes)

def build(st=None, M0=np.eye(4), tag='w', body='worker', gun=True, P=None):
    P = cell.P if P is None else P
    def a(name, part, col, M, b=body):
        P.append(dict(name=f'{tag}_{name}', part=part, col=col, M=M0 @ M, body=b))
    a('hub', '26910', WHT, T())
    # wheel modules: axle 4 pushed 20 into the output, rim + off-road tyre outside the hub
    for sx in (-1, 1):
        a(f'wax{sx}', '3705', BLK, T(80*sx, 66, 140), f'{body}')
        Rw = R('y', 90 if sx > 0 else -90)
        a(f'rim{sx}', '56145', LBG, T(94*sx, 66, 140) @ Rw)
        a(f'tyre{sx}', '55976', BLK, T(94*sx, 66, 140) @ Rw)
    # skid under the front end: brick 2x2 + plate 2x2 + round-bottom plate (hub anti-studs at y=96)
    a('sk_b', '3003', DBG, T(0, 96, -140)); a('sk_p', '3022', DBG, T(0, 120, -140)); a('sk_p2', '3022', LBG, T(0, 128, -140))   # bottom 136, 1 LDU above ground
    # torso: Boost motor on the hub's front stud field, spacer plates under its raised outer rows
    for sx in (-1, 1): a(f'tp{sx}', '3666', DBG, T(30*sx, -8, -80) @ R('y', 90))
    a('motor', '26913', WHT, T(0, -72, -80))
    # neck + head: 2x2 brick on the motor's top studs, 2x4 plate, studs-on-side brick with round-tile eyes
    a('neck', '3003', DBG, T(0, -96, -80))
    a('hplate', '3020', DBG, T(0, -104, -80))
    a('hface', '30414', WHT, T(0, -128, -90))
    a('hback', '3010', WHT, T(0, -128, -70))
    a('htop', '3020', RED, T(0, -136, -80))
    for ex in (-10, 10): a(f'eye{ex}', '98138', TLB, T(ex, -118, -108) @ R('x', 90))
    # shoulders: Technic 1x2 bricks on the motor's raised outer studs, arms = beam 5 pointing forward
    for sx in (-1, 1):
        a(f'sh{sx}', '3700', RED, T(30*sx, -88, -60) @ R('y', 90))
        a(f'shpin{sx}', '2780', BLK, T(40*sx, -78, -60))
        a(f'arm{sx}', '32316', LBG, T(50*sx, -78, -100) @ ux([0, 0, -1]))
    if gun:
        # stud blaster clipped on the hub's free front stud row, fired by a cam on the motor output
        a('gplate', '3710', DBG, T(0, -8, -150))
        a('gclip', '4085c', BLK, T(30, -16, -150))                      # vertical clip, clip axis at (30, -12, -170)
        a('gun', '15391c01', BLK, T(30, -6, -170))                        # handle bar centred in the clip
        a('gax', '4519', BLK, T(0, -38, -150) @ R('y', 90))
        a('gcam', '43857', RED, T(0, -38, -172) @ R('x', 90) @ T(0, 0, 10))
    return P

if __name__ == '__main__':
    import lcheck
    P = build(P=[])
    lcheck.run(P)
