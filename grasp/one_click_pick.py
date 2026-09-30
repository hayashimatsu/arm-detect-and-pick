"""ONE-CLICK PICK — shortcut for: run demo_start.py, then detect_pick(TARGET).
Isaac Sim Script Editor: open this file, set TARGET, Ctrl+Enter. After that, detect() / pick() / detect_pick() /
status() can also be called directly (see demo_start.py and README).
"""
TARGET = "cube"          # "cube" | "cuboid" | "cylinder" | "sphere"
ORDER_BY = "camera"      # "camera" (nearest to the camera first) | "arm_base"
MAX_PICKS = 12

import os, omni.usd
# project root = parent of the folder holding the open scene (scenes/*.usd)
_p = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(
    omni.usd.get_context().get_stage().GetRootLayer().realPath))), "demo_start.py")
exec(compile(open(_p, encoding="utf-8").read(), _p, "exec"), globals())
print(detect_pick(TARGET, MAX_PICKS, order_by=ORDER_BY))
