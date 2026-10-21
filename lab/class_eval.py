"""lab/class_eval.py -- how well does the video scan see every block (recall) and name its class (cube vs cuboid ...)?
Per seed: layout(seed=...), arm to view A, scan ~1.5 s with the video tracker, then compare tracks and per-frame observations with the stage truth (report only).
   exec(open(...).read()); start_ce([1,2,3], "c1")          needs builtins._lab_ds2.   Writes outputs/lab/live/class_eval_<tag>.json"""
import asyncio, builtins, json, math, os
import numpy as np
import omni.kit.app, omni.timeline


async def _idle(reg):
    for _ in range(60 * 60 * 5):
        await omni.kit.app.get_app().next_update_async()
        if reg["task"] is None or reg["task"].done():
            return


async def _ce(seeds, tag, target=None, secs=1.5):
    g = builtins._lab_ds2; reg = builtins._nhc_d455_yolo2_demo; root = g["ROOT"]; rows = []
    path = os.path.join(root, "outputs", "lab", "live", f"class_eval_{tag}.json")
    for sd in seeds:
        row = dict(seed=sd)
        try:
            omni.timeline.get_timeline_interface().set_auto_update(True)
            g["layout"](target, seed=sd); await _idle(reg)
            for _ in range(30): await omni.kit.app.get_app().next_update_async()
            g["_remove_original_object"](); g["_base_ns"]["_ik_target_visible"](False)
            ns = g["_load_lab"](); pk = ns["LivePicker"](); await ns["_PK"]["ensure_sim"]()
            rg = builtins._arm310d_ik_follow_controller_registry; rg["instance"]._on_update.__globals__["MAX_JOINT_DELTA_PER_STEP"] = ns["P"]["ik_cap"]
            truth = pk.truth(); live = ns["Live"](pk.oc); live.rec_dir = os.path.join(root, "outputs", "lab", "live", f"class_eval_{tag}_{sd}"); os.makedirs(live.rec_dir, exist_ok=True); await live.start()
            va = pk.oc["_view_poses"]()[0]; cur, _ = pk.oc["_pose"]("/World/IKTarget")
            await pk.fly([[cur[0], cur[1], max(cur[2], va["position_m"][2])], va["position_m"]], va["quaternion_wxyz"], "to_view", [ns["P"]["fillet"]])
            await pk.arrive(va["position_m"], 0.004, 90, va["quaternion_wxyz"])
            await pk.scan(live, "cube", secs, secs + 1.0)
            tr = live.tracker.tr; objs = []
            names = list(truth)
            def nearest(c):
                n = min(names, key=lambda k: math.dist(truth[k][:2], c[:2])); return n, math.dist(truth[n][:2], c[:2])
            by = {n: dict(name=n, true_cls=n.split("_")[0], xy=[round(v, 3) for v in truth[n][:2]], frames=[], track=None) for n in names if n != "object"}
            for o in live.obs_log:
                n, dd = nearest(o[3])
                if n in by and dd < 0.03:
                    by[n]["frames"].append(dict(yolo=o[8], cls=o[1], conf=o[2], top=o[9], h=o[10], fg=o[11], good=(not o[6]) and o[7] <= live.max_speed))
            for t in tr:
                n, dd = nearest(t["center"])
                if n in by and dd < 0.03:
                    by[n]["track"] = dict(cls=t["class_name"], conf=round(float(t["conf"]), 2), n=t["n"], votes={k: round(v, 2) for k, v in t["votes"].items()})
            row["objects"] = list(by.values()); row["blobs"] = (live.blobs_latest[1] if live.blobs_latest else None); row["raw"] = getattr(live, "last_raw", None); row["errors"] = live.errors[:3]
            await live.stop(); g["_base_ns"]["_ik_target_visible"](True)
        except Exception:
            import traceback; row["error"] = traceback.format_exc()
        rows.append(row)
        json.dump(dict(done=False, rows=rows), open(path, "w"), indent=1, default=str)
    json.dump(dict(done=True, rows=rows), open(path, "w"), indent=1, default=str)


def start_ce(seeds, tag="c1", target=None, secs=1.5):
    builtins._lab_ce_task = asyncio.ensure_future(_ce(seeds, tag, target, secs))
