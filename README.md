# Self-replicating LEGO world — factory + robots (snapshot 2026-11-21)

Everything is generated from the Python in `src/` (Blender 5.x `bpy` as a Python module, numpy, scipy, Pillow, ffmpeg).
Real LDraw parts; every station is clash/fit-checked with `lcheck.py`.

## Setup
1. LDraw library (not included, public): download `complete.zip` from ldraw.org, unpack to `/home/claude/ldraw/lib/complete/ldraw`.
2. LDCad shadow library (included as `ldcad-shadow/`): copy to `/home/claude/ldraw/shadow`.
3. Put `src/` at `/home/claude/scripts` (paths are hard-coded in `ldr.py` ROOT / SHADOW / CACHE and in the preview scripts).

## Map of the code
- Parts & checking: `ldr.py` (LDraw + shadow snaps), `lcheck.py` (clashes, pin/axle/stud fits, gear meshes incl. 90° double bevels), `lsweep.py`
- Robots: `worker2.py` (robot: core, left/right track units, head), `mods.py` (shoulder hook, modules), `periph.py`, `lowmods.py`, `tug.py`
- Final assembly line: `line3.py` (whole line, builder + tug configs, `cycle()` / `build()`), `asm2.py` (track-unit cell + crank pushers), `cell.py`, `rails.py` (train track, carts, locos), `shoulder.py` (stand-up/shoulder station)
- Track-unit line: `tline.py` (build wagon S0–S4, supply train S0–S5, timeline `state()`), `tunit.py` (wagons, supply tooling)
- Chain station: `chainst.py` (tilting table, chain drive, transfer rods/crank, guide posts), `cstation.py` (hinge, tilt hub, link belt + queue, timeline)
- Previews: `prev3.py`, `prevT.py`, `prevC.py` (+ `*_post.py` captions/encoding); film tools `film*.py`, `render_*.py`

Run a preview, e.g.: `python3 src/prevC.py OUTDIR 246 && python3 src/prevC_post.py OUTDIR out.mp4 6`
Check a time point: `python3 -c "import line3, lcheck; lcheck.run(line3.build(line3.cycle(14/41, 'builder')))"`

## Open items
Parts warehouse restocking the supply wagons, link belt and S5; cable from tilt hub to the chain-station motors.
