"""Lower-module extras on the lying track station (asm2 carrier, robot on its back): the tug's right track unit comes with the
belly-sensor bracket already pinned on, and plug D rides in a shoe on the plunger group (periph's plug shoe, carried by a
Technic brick 1x4 held on the pin plunger by two bushes and sliding on the outer guide, so it can't turn), so unit, sensor, cable and plug all go home in the same push."""
import numpy as np
import asm2, mods, lcheck, cable as _cab
lcheck.RESTING.update({"plugDB": "rides in the plug shoe"})
from asm2 import T, R, add, Y0, LBG, DBG, BLK, YEL, RED

def eye_on_unit(Mu, body):
    """Mu: robot-frame -> world transform of the (moving) right unit."""
    a = lambda n, p, c, M: add(f'eyeB_{n}', p, c, Mu @ M, ('plugDB' if body != 'robot' else body) if n == 'plugD' else body)
    def cab(n, part, ends): asm2.P.append(dict(name=f'eyeB_{n}', part=part, col=0, M=Mu.copy(), body=body, ends=(f'eyeB_{ends[0]}', f'eyeB_{ends[1]}')))
    mods.eye_module(a, cab)

def plug_shoe(sx, e, side='B'):
    """periph's plug shoe on the asm2 pin plunger (sx: carrier x, e: plunger offset)."""
    s = 1 if side == 'A' else -1; b = f'plung{side}'; xp = sx + 20
    a = lambda n, p, c, M: add(f'shoe{side}_{n}', p, c, M, b)
    a('base', '3701', YEL, T(sx + 60, Y0 - 10, (110 + e)*s))                            # holes on the pin plunger (60) and the guide (80)
    a('bush', '32123a', YEL, T(sx + 60, Y0, (125 + e)*s))
    for k in range(3): a(f'p{k}', '3710', DBG, T(xp, Y0 - 18 - 8*k, (110 + e)*s))
    a('floor', '3032', LBG, T(xp, Y0 - 42, (120 + e)*s) @ R('y', 90))
    a('stop', '3069b', RED, T(xp, Y0 - 50, (70 + e)*s))
    a('wall', '3010', YEL, T(xp - 30, Y0 - 66, (120 + e)*s) @ R('y', 90))
    a('rim', '3710', YEL, T(xp + 30, Y0 - 50, (120 + e)*s) @ R('y', 90))
