"""lab/refine_eval.py -- feedback-4 #2: visual final correction on/off. First pick only (max_picks=1), same layout (class, seed) for every mode.
Per run: hand-to-block distance when the fingers close (stage truth, report only), first-try success, time from leg start to grasp and to release.
   exec(open(...).read()); start_refine(["sphere","cube"], [51,52,53,54,55], [False,"change","estimate"], None, "x1")      shift=None | (0.025, 0.0)
Needs builtins._lab_ds2 (demo_start2 globals).  Writes outputs/lab/live/refine_<tag>.json"""
import asyncio, builtins, json, os
import omni.kit.app, omni.timeline


async def _idle(reg):
    for _ in range(60 * 60 * 5):
        await omni.kit.app.get_app().next_update_async()
        if reg["task"] is None or reg["task"].done():
            return


async def _re(classes, seeds, modes, shift, tag):
    g = builtins._lab_ds2; reg = builtins._nhc_d455_yolo2_demo; root = g["ROOT"]; rows = []
    for cls in classes:
        for sd in seeds:
            for mode in modes:
                row = dict(cls=cls, seed=sd, mode=mode, shift=shift)
                try:
                    omni.timeline.get_timeline_interface().set_auto_update(True)
                    g["layout"](cls, seed=sd); await _idle(reg)
                    for _ in range(30): await omni.kit.app.get_app().next_update_async()
                    g["_remove_original_object"]()
                    ns = g["_load_lab"](); ns["P"]["visual_refine"] = mode
                    res = await ns["LivePicker"]().run_live(cls, 1, False, False, dict(shift=shift) if shift else None)
                    ev = {e["name"]: e["sim_s"] for e in res.get("events", [])}; c = (res.get("cycles") or [None])[0]
                    row.update(error=res.get("error"), run_dir=res.get("run_dir"))
                    if c:
                        rl = c.get("refine_log", [])
                        row.update(grasp_err_mm=c.get("grasp_err_mm"), ok=bool(c.get("ok")), first_try=bool(c.get("ok") and (c.get("tug") or {}).get("tests") == 1),
                                   leg_to_grasp_s=None if "k1:at_grasp" not in ev else round(ev["k1:at_grasp"] - ev["k1:leg_start"], 2),
                                   leg_to_release_s=None if "k1:released" not in ev else round(ev["k1:released"] - ev["k1:leg_start"], 2), arrive_to_release_s=c.get("from_arrive_to_release_s"),
                                   refine_samples=len(rl), retargets=c.get("retargets"), cls_picked=c.get("cls"))
                except Exception:
                    import traceback; row["error"] = traceback.format_exc()
                rows.append(row)
                json.dump(dict(done=False, rows=rows), open(os.path.join(root, "outputs", "lab", "live", f"refine_{tag}.json"), "w"), indent=1, default=str)
    json.dump(dict(done=True, rows=rows), open(os.path.join(root, "outputs", "lab", "live", f"refine_{tag}.json"), "w"), indent=1, default=str)


def start_refine(classes, seeds, modes, shift=None, tag="x1"):
    builtins._lab_re_task = asyncio.ensure_future(_re(classes, seeds, modes, shift, tag))
