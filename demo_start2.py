"""demo_start2.py -- Isaac Sim Script Editor: open this file, Ctrl+Enter ONCE (scene scenes/arm310d_d455_ik_demo.usd open).
It only gets the arm ready and prints "demo_start2: READY". Two functions:

    layout("sphere")        (testing) 10 new random blocks, every size of that type included; basket emptied;
                            the original red cube is removed.   layout() = any 10 random blocks.
    detect_pick("sphere")   pick every block of that type into the basket:
                              - the wrist D455 runs as VIDEO: class / confidence / world position / distance are tracked live
                                (no more two photos), the arm moves along one continuous path (no stops),
                              - grasp = one continuous close, place = put down on the basket floor (never dropped).
                            Report: outputs/lab/live/run_<time>/index.html (annotated wrist video, times, what moved).

See lab/README-lab.md.
"""
import os
import omni.usd

_root = os.path.dirname(os.path.dirname(os.path.abspath(omni.usd.get_context().get_stage().GetRootLayer().realPath)))
_api = os.path.join(_root, "lab", "api2.py")
if not os.path.isfile(_api):
    raise RuntimeError(f"Open this project's scene (scenes/arm310d_d455_ik_demo.usd) first: {_api} not found")
exec(compile(open(os.path.join(_root, "lab", "shim_numpy_cross.py"), encoding="utf-8").read(), "shim_numpy_cross.py", "exec"), globals())
__file__ = _api
exec(compile(open(_api, encoding="utf-8").read(), _api, "exec"), globals())
