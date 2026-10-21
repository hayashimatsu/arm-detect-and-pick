"""lab/basket_probe.py -- what does the wrist D455 depth see of the basket from above it (empty hand), for several yaws and heights?
   exec(open(...).read()); start_bp("b1")      needs builtins._lab_ds2.  Writes outputs/lab/live/basket_probe_<tag>.json (+ png)"""
import asyncio, builtins, json, os
import numpy as np
import omni.kit.app


async def _bp(tag):
    g = builtins._lab_ds2; root = g["ROOT"]; out = dict(rows=[])
    try:
        ns = g["_load_lab"](); pk = ns["LivePicker"]()
        await ns["_PK"]["ensure_sim"]()
        reg = builtins._arm310d_ik_follow_controller_registry; gg = reg["instance"]._on_update.__globals__; gg["MAX_JOINT_DELTA_PER_STEP"] = ns["P"]["ik_cap"]
        live = ns["Live"](pk.oc); await live.start()
        bmin, bmax = pk.basket_box(); bc = [(bmin[0] + bmax[0]) / 2, (bmin[1] + bmax[1]) / 2]
        out["basket"] = dict(bmin=[round(v, 3) for v in bmin], bmax=[round(v, 3) for v in bmax])
        cur, _ = pk.oc["_pose"]("/World/IKTarget")
        for yaw in (0.0, 45.0, 90.0, -45.0):
            qd = list(pk.pm.quat_down_with_yaw(yaw))
            for z in (0.76, 0.90):
                cur, _ = pk.oc["_pose"]("/World/IKTarget")
                await pk.fly([[cur[0], cur[1], max(cur[2], 0.9)], [bc[0], bc[1], z]], qd, f"probe_{yaw}_{z}", [0.07])
                await pk.arrive([bc[0], bc[1], z], 0.004, 150, qd)
                await pk.frames(20)
                depth = np.asarray(live.depth_ann.get_data(), dtype=np.float32)
                d = live.cp.get_data(); v = np.asarray(d["cameraViewTransform"], float).reshape(4, 4); inv = np.linalg.inv(v)
                cam = dict(position_m=inv[3, :3].tolist(), right=inv[0, :3].tolist(), up=inv[1, :3].tolist(), forward=(-inv[2, :3]).tolist())
                P_ = live.geo.world_points(depth, cam, live.K, 0.5)
                ok = np.isfinite(depth) & (depth > 0) & (depth < 5)
                m = ok & (P_[..., 0] > bmin[0]) & (P_[..., 0] < bmax[0]) & (P_[..., 1] > bmin[1]) & (P_[..., 1] < bmax[1])
                pts = P_[m]
                row = dict(yaw=yaw, z=z, cam=[round(x, 3) for x in cam["position_m"]], n_in_basket=int(m.sum()), frac=round(float(m.sum()) / max(1, ok.sum()), 3))
                if len(pts):
                    row.update(x_range=[round(float(pts[:, 0].min()), 3), round(float(pts[:, 0].max()), 3)], y_range=[round(float(pts[:, 1].min()), 3), round(float(pts[:, 1].max()), 3)],
                               z_pct=[round(float(np.percentile(pts[:, 2], p)), 4) for p in (2, 10, 50, 90, 98)])
                out["rows"].append(row)
                json.dump(out, open(os.path.join(root, "outputs", "lab", "live", f"basket_probe_{tag}.json"), "w"), indent=1, default=str)
        await live.stop()
        home = out.get("home")
    except Exception:
        import traceback; out["error"] = traceback.format_exc()
    out["done"] = True
    json.dump(out, open(os.path.join(root, "outputs", "lab", "live", f"basket_probe_{tag}.json"), "w"), indent=1, default=str)


def start_bp(tag="b1"):
    builtins._lab_bp_task = asyncio.ensure_future(_bp(tag))
