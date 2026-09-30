"""demo_start.py — Isaac Sim Script Editor: open this file, Ctrl+Enter ONCE. Then call any of these from the Script Editor:

    detect()                         take pictures (view A, B if needed) -> objects, classes, world centers; arm back home
    pick("sphere")                   put ONE object of that class into the basket, using the last detect() result
    pick("sphere", id=3)             ... that detected id instead of the nearest one
    detect_pick("sphere")            repeat detect() -> pick() until no sphere is left; opens the report
    layout("sphere")                 (testing) new random blocks on the table incl. every sphere; basket emptied
    refresh()                        reopen the scene and restart (when Isaac Sim got slow); test blocks are removed
    status()                         what is running now / the last result

target = "cube" | "cuboid" | "cylinder" | "sphere".  order_by = "camera" (default, nearest to the camera first) | "arm_base".
Every call returns at once and runs in the background (Isaac Sim must keep rendering); poll status().
Only one action runs at a time. pick() needs a fresh detect(): after any pick/layout the old detection is stale.
Reports: detect -> outputs/oneclick/<time>/index.html, pick / detect_pick -> outputs/pick/<time>/index.html.
"""
import asyncio, builtins, datetime, math, os, traceback

def _stage_root():
    """Project root = parent of the folder holding the open scene (scenes/*.usd). No path is hard-coded."""
    import omni.usd
    layer = omni.usd.get_context().get_stage().GetRootLayer()
    return os.path.dirname(os.path.dirname(os.path.abspath(layer.realPath)))


ROOT = _stage_root()      # the scene of this folder must be open (see README)
_REG = "_nhc_d455_yolo2_demo"


def _reg():
    if not hasattr(builtins, _REG):
        setattr(builtins, _REG, dict(start_task=None, task=None, action=None, state="idle", result=None, error=None,
                                     detection=None, started=None))
    return getattr(builtins, _REG)


def _load(rel):
    p = os.path.join(ROOT, rel); g = dict(NO_AUTORUN=True, __file__=p)
    exec(compile(open(p, encoding="utf-8").read(), p, "exec"), g)       # always the latest code on disk
    return g


# ------------------------------------------------------------------ start (runs once when this file is executed)
async def _start():
    oc = _load("perception/one_click.py")
    pk = _load("grasp/pick.py")
    await pk["ensure_sim"]()
    ns = oc["_load_arm_stack"]()                 # checks the open stage is this folder's scene; loads IK + D455 capture
    ns["ik_follow_start"]()
    import omni.kit.app
    for _ in range(600):
        await omni.kit.app.get_app().next_update_async()
        if ns["ik_follow_status"]().get("running"):
            break
    else:
        raise RuntimeError(f"IK did not start: {ns['ik_follow_status']()}")
    await oc["_ensure_server"]()                 # YOLO server (own Python env), so the first detect() is fast
    return "ready"


def _ik_target_visible(on):
    """Show / hide the /World/IKTarget marker (session layer only, never saved). Hidden while an action runs."""
    try:
        import omni.usd
        from pxr import Usd, UsdGeom
        st = omni.usd.get_context().get_stage(); prim = st.GetPrimAtPath("/World/IKTarget")
        if prim.IsValid():
            with Usd.EditContext(st, st.GetSessionLayer()):
                UsdGeom.Imageable(prim).GetVisibilityAttr().Set(UsdGeom.Tokens.inherited if on else UsdGeom.Tokens.invisible)
    except Exception:
        pass


def _schedule(action, coro_fn):
    r = _reg()
    if r["task"] is not None and not r["task"].done():
        return dict(state="busy", running=r["action"], hint="wait, or check status()")

    async def body():
        _ik_target_visible(False)
        try:
            if r["start_task"] is not None and not r["start_task"].done():
                await r["start_task"]
            if r["start_task"] is not None and r["start_task"].exception():
                raise RuntimeError(f"demo_start failed: {r['start_task'].exception()}")
            res = await coro_fn()
            r.update(state="done", result=res, error=None)
            print(f"[{action}] done:", res)
        except Exception as exc:
            r.update(state="error", error=f"{type(exc).__name__}: {exc}", traceback=traceback.format_exc())
            print(f"[{action}] ERROR:", r["error"])
        except asyncio.CancelledError:
            r.update(state="cancelled", error="cancelled"); raise
        finally:
            _ik_target_visible(True)
    r.update(action=action, state="running", result=None, error=None, traceback=None,
             started=datetime.datetime.now().isoformat(timespec="seconds"))
    r["task"] = asyncio.ensure_future(body())
    return dict(state="scheduled", action=action, hint="poll status()")


def _stale(reason):
    d = _reg()["detection"]
    if d is not None and not d.get("stale"):
        d.update(stale=True, stale_reason=reason)


def _check(target, order_by):
    if target not in ("cube", "cuboid", "cylinder", "sphere"):
        raise ValueError(f'target must be "cube", "cuboid", "cylinder" or "sphere", got {target!r}')
    if order_by not in ("camera", "arm_base"):
        raise ValueError(f'order_by must be "camera" or "arm_base", got {order_by!r}')


# ------------------------------------------------------------------ public API
def detect():
    """Pictures -> objects with class, confidence, world center; arm back home. Result kept for pick()."""
    async def run():
        pk = _load("grasp/pick.py")
        await pk["ensure_sim"]()
        doc = await pk["Picker"]().perceive()
        cam = doc["views"][0]["camera"]["position_m"]
        objs = [dict(id=o["id"], class_name=o["class_name"], conf=round(o["conf"], 3),
                     center_mm=[round(v * 1000, 1) for v in o["center_xyz_m"]] if o.get("center_xyz_m") else None,
                     dist_from_camera_m=round(math.dist(o["center_xyz_m"], cam), 3) if o.get("center_xyz_m") else None)
                for o in doc["objects"]]
        _reg()["detection"] = dict(doc=doc, run_id=doc["run_id"], stale=False, objects=objs)
        return dict(run_id=doc["run_id"], n_objects=len(objs), objects=sorted(objs, key=lambda o: o["dist_from_camera_m"] or 9),
                    report=os.path.join(ROOT, "outputs", "oneclick", doc["run_id"], "index.html"))
    return _schedule("detect", run)


def pick(target, id=None, order_by="camera"):
    """ONE object of `target` into the basket, from the last detect() (nearest first, or the given id)."""
    _check(target, order_by)
    d = _reg()["detection"]
    if d is None:
        return dict(state="refused", why="no detection yet -> call detect() first")
    if d.get("stale"):
        return dict(state="refused", why=f"the last detection is stale ({d['stale_reason']}) -> call detect() first")
    cands = [o for o in d["objects"] if o["class_name"] == target and (id is None or o["id"] == id)]
    if not cands:
        return dict(state="refused", why=f"the last detect() ({d['run_id']}) has no {target}" + (f" with id {id}" if id is not None else ""),
                    detected=[(o["id"], o["class_name"]) for o in d["objects"]])

    async def run():
        pk = _load("grasp/pick.py")
        _stale("a pick() moved objects")
        doc = await pk["Picker"]().run(target, 1, order_by, open_report=False, doc=d["doc"], only_id=id)
        return _summary(doc)
    return _schedule(f"pick({target!r})", run)


def detect_pick(target, max_picks=12, order_by="camera", open_report=True):
    """detect() -> pick() repeated until no `target` is left (or max_picks attempts). Opens the report at the end."""
    _check(target, order_by)

    async def run():
        pk = _load("grasp/pick.py")
        _stale("detect_pick() moved objects")
        doc = await pk["Picker"]().run(target, max_picks, order_by, open_report=open_report)
        return _summary(doc)
    return _schedule(f"detect_pick({target!r})", run)


def layout(target=None, n=10, seed=None):
    """(testing) Stop -> new random test blocks (<=60 mm, physics; every block of `target` included) -> play. Basket emptied."""
    async def run():
        placed = await _load("grasp/series_test.py")["relayout"](target, seed, n)
        _stale("layout() replaced the blocks")
        return dict(placed=[p["name"] for p in placed])
    return _schedule(f"layout({target!r})", run)


def refresh():
    """Reopen this folder's scene (never saved) and start again like demo_start.py.
    Clears what builds up over a long session and slows the simulation down: camera render products left by
    earlier code, Replicator state, physics/IK callbacks. Test blocks are removed -> layout() again if needed."""
    async def run():
        import omni.usd, omni.timeline, omni.kit.app
        ctx = omni.usd.get_context(); path = ctx.get_stage().GetRootLayer().realPath
        ik = {"__file__": os.path.join(ROOT, "scripts", "ik_controller.py")}
        exec(compile(open(ik["__file__"], encoding="utf-8").read(), ik["__file__"], "exec"), ik)
        ik["ik_follow_stop"]()
        omni.timeline.get_timeline_interface().stop()
        for name in ("_nhc_d455_capture_left", "_nhc12_d455_capture_registry"):   # cached render products
            if hasattr(builtins, name):
                delattr(builtins, name)
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()
        ok, err = await ctx.open_stage_async(path)
        if not ok:
            raise RuntimeError(f"could not reopen {path}: {err}")
        for _ in range(60):
            await omni.kit.app.get_app().next_update_async()
        _stale("refresh() reopened the scene")
        await _start()
        return dict(reopened=path, note="test blocks removed; call layout() for a test scene")
    return _schedule("refresh()", run)


def _summary(doc):
    d = doc["diagnosis"]
    picks = [c for c in doc["cycles"] if not c.get("done")]
    end = next((c for c in doc["cycles"] if c.get("done")), None)
    return dict(in_basket=d["in_basket"], expected=d["expected_truth"],
                picks=[dict(id=c["target_id"], cls=c["target_class"], outcome=c["outcome"], dist_from_camera_m=c.get("dist_from_camera_m"))
                       for c in picks],
                stopped_because=end["reason"] if end else ("one pick() done" if len(picks) == 1 else "max_picks reached"),
                issues=[i["what"] for i in d["issues"]], report=os.path.join(doc["run_dir"], "index.html"), total_s=doc["total_s"])


def status():
    """What is running now (with the current step) and the last result."""
    import json
    r = _reg()
    out = {k: r.get(k) for k in ("action", "state", "started", "error", "result")}
    if r["start_task"] is not None:
        out["start"] = "running" if not r["start_task"].done() else (f"ERROR {r['start_task'].exception()}" if r["start_task"].exception() else "ready")
    if r["state"] == "running":
        for f in ("pick", "oneclick"):
            try:
                out[f"{f}_step"] = json.load(open(os.path.join(ROOT, "outputs", f, "latest_status.json"))).get("step")
            except Exception:
                pass
    d = r["detection"]
    out["last_detection"] = None if d is None else dict(run_id=d["run_id"], stale=d["stale"], objects=[(o["id"], o["class_name"]) for o in d["objects"]])
    return out


# ------------------------------------------------------------------ executed on Ctrl+Enter
def _boot():
    r = _reg()
    if r["task"] is not None and not r["task"].done():
        print("demo_start: an action is still running:", r["action"], "-> not restarting"); return
    if r["start_task"] is None or r["start_task"].done():
        r["start_task"] = asyncio.ensure_future(_start())
    print("demo_start: starting (IK + camera + YOLO server). Functions: detect(), pick(target), detect_pick(target), layout(target), status()")


if not globals().get("NO_AUTORUN"):
    _boot()
