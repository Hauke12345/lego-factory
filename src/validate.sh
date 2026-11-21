#!/bin/bash
# Build validation pipeline for the Lego models. Run after every design change; nothing gets rendered unless it passes.
#   1. geometry:  part-part intersections (real meshes, 0.12 mm shrink), stud/pin/axle/hinge connection graph,
#                 every model one connected piece, pin stacks without air gaps
#   2. FEM:       linear frame model, self-weight + truck at 34 positions -> stress & connection utilisation,
#                 support uplift, parts that carry no load
#   3. dynamics:  MuJoCo rigid-body run of the truck crossing (optional, slow: VALIDATE_DYN=1)
set -e
cd /home/claude
python3 scripts/check_build.py render_b/build_check.json | grep -v "^EGL" | tee render_b/validate_geom.txt
grep -q "0 unexpected" render_b/validate_geom.txt || { echo "FAIL: intersections"; exit 1; }
grep -q "0 with an air gap" render_b/validate_geom.txt || { echo "FAIL: pin stack gaps"; exit 1; }
grep -q "UNCONNECTED" render_b/validate_geom.txt && { echo "FAIL: unconnected parts"; exit 1; }
[ "$(grep -c 'in 1 connected group' render_b/validate_geom.txt)" = "2" ] || { echo "FAIL: model falls apart into several groups"; exit 1; }
python3 scripts/fem.py render_b/build_check.json | tee render_b/validate_fem.txt
python3 scripts/fem_panel.py > /dev/null
python3 - <<'PY'
import json; r = json.load(open('/home/claude/render_b/fem.json'))
u = max(max(c['util'].values()) for c in r['cases'])
print(f'FEM peak utilisation {u:.2f}'); assert u < 1.0, 'FAIL: overloaded part or connection'
PY
if [ "$VALIDATE_DYN" = "1" ]; then python3 scripts/load_test.py | tee render_b/validate_dyn.txt; fi
echo "VALIDATION PASSED"
