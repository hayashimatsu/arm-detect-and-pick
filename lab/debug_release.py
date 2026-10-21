"""lab/debug_release.py -- feedback-2 #1: single block, nothing else changed, per-frame record of the release and the leave.
Writes outputs/lab/live/release_debug_<tag>_<seed>.json (rows from lab/fast_pick.py ReleaseDebug) and a summary.
    exec(open(".../lab/debug_release.py").read()); start_dbg("cube", [33, 36, 41], "r1")     (needs builtins._lab_ds2 = demo_start2 globals)"""
import asyncio, builtins, json, os
import omni.kit.app, omni.timeline


async def _idle(reg):
    for _ in range(60 * 60 * 5):
        await omni.kit.app.get_app().next_update_async()
        if reg["task"] is None or reg["task"].done():
            return


async def _dbg(target, seeds, tag, mode=True):
    g = builtins._lab_ds2; reg = builtins._nhc_d455_yolo2_demo; root = g["ROOT"]; rows = []
    for idx, sd in enumerate(seeds):
        row = dict(seed=sd, trial=idx)
        try:
            omni.timeline.get_timeline_interface().set_auto_update(True)
            g["layout"](target, seed=sd); await _idle(reg)
            for _ in range(30): await omni.kit.app.get_app().next_update_async()
            g["_remove_original_object"]()
            ns = g["_load_lab"]()
            ns["P"]["slow_leave"] = False
            res = await ns["LivePicker"]().run_live(target, 2 if mode == "next" else 1, True, mode)
            cyc = res.get("cycles", [])
            row.update(error=res.get("error"), run_dir=res.get("run_dir"), n_cycles=len(cyc))
            dbgd = next((c.get("release_debug") or c.get("release_debug_prev") for c in cyc if c.get("release_debug") or c.get("release_debug_prev")), None)
            if dbgd:
                json.dump(dbgd, open(os.path.join(root, "outputs", "lab", "live", f"release_debug_{tag}_{sd}_{idx}.json"), "w"))
                row["frames"] = len(dbgd["rows"]); row["api_error"] = dbgd["api_error"]; row["cycles_ok"] = [c.get("ok") for c in cyc]; z0 = dbgd["rows"][0]["block"][2]; row["max_block_rise_mm"] = round(max(1000 * (r["block"][2] - z0) for r in dbgd["rows"]), 1); row["gripper_contacts"] = sorted({c for r in dbgd["rows"] for c in r["contacts"] if "HeavyDuty" not in c})
        except Exception:
            import traceback; row["error"] = traceback.format_exc()
        rows.append(row)
        json.dump(dict(done=False, rows=rows), open(os.path.join(root, "outputs", "lab", "live", f"release_debug_{tag}.json"), "w"), indent=1, default=str)
    json.dump(dict(done=True, rows=rows), open(os.path.join(root, "outputs", "lab", "live", f"release_debug_{tag}.json"), "w"), indent=1, default=str)


def start_dbg(target, seeds, tag="r1", mode=True):
    builtins._lab_dbg_task = asyncio.ensure_future(_dbg(target, seeds, tag, mode))
