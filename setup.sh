#!/usr/bin/env bash
# Rebuild the working environment of the LEGO factory project (Linux, Python 3.11+).
# The code expects the fixed layout /home/claude/{scripts,ldraw,factory}; this script creates it.
set -e
REPO="$(cd "$(dirname "$0")" && pwd)"
BASE=/home/claude
mkdir -p $BASE/ldraw/lib/complete $BASE/factory
pip install --break-system-packages bpy==5.2.3 numpy scipy pillow matplotlib opencv-python-headless imageio || \
pip install bpy==5.2.3 numpy scipy pillow matplotlib opencv-python-headless imageio
command -v ffmpeg >/dev/null || (apt-get update && apt-get install -y ffmpeg)
# LDraw parts library (public, ~700 MB unpacked)
if [ ! -f $BASE/ldraw/lib/complete/ldraw/LDConfig.ldr ]; then
  curl -L -o /tmp/complete.zip https://library.ldraw.org/library/updates/complete.zip
  unzip -q /tmp/complete.zip -d $BASE/ldraw/lib/complete
fi
# LDCad shadow library (connection/snap data, shipped in this repo)
rm -rf $BASE/ldraw/shadow && cp -r "$REPO/ldcad-shadow" $BASE/ldraw/shadow
# code
if [ "$REPO/src" != "$BASE/scripts" ]; then rm -rf $BASE/scripts && ln -s "$REPO/src" $BASE/scripts; fi
python3 -c "import sys; sys.path.insert(0,'$BASE/scripts'); import ldr; print(ldr.part('3001')['title'])"
echo "setup done"
