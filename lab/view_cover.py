"""lab/view_cover.py -- which table points does the wrist camera see (with a border margin) from view A, and from A shifted by (dx, dy)?
Pure projection with the camera at A (orientation fixed, camera moves with the hand).  start_vc("v1")  -> outputs/lab/live/view_cover_<tag>.json"""
import asyncio, builtins, json, os
import numpy as np


async def _vc(tag):
    g = builtins._lab_ds2; root = g["ROOT"]; out = {}
    try:
        ns = g["_load_lab"](); pk = ns["LivePicker"](); await ns["_PK"]["ensure_sim"]()
        rg = builtins._arm310d_ik_follow_controller_registry; rg["instance"]._on_update.__globals__["MAX_JOINT_DELTA_PER_STEP"] = ns["P"]["ik_cap"]
        live = ns["Live"](pk.oc); await live.start()
        va = pk.oc["_view_poses"]()[0]; cur, _ = pk.oc["_pose"]("/World/IKTarget")
        await pk.fly([[cur[0], cur[1], max(cur[2], va["position_m"][2])], va["position_m"]], va["quaternion_wxyz"], "to_view", [ns["P"]["fillet"]])
        await pk.arrive(va["position_m"], 0.004, 90, va["quaternion_wxyz"]); await pk.frames(30)
        d = live.cp.get_data(); v = np.asarray(d["cameraViewTransform"], float).reshape(4, 4); inv = np.linalg.inv(v)
        cam = dict(position_m=inv[3, :3].tolist(), right=inv[0, :3].tolist(), up=inv[1, :3].tolist(), forward=(-inv[2, :3]).tolist())
        depth = np.asarray(live.depth_ann.get_data(), dtype=np.float32); H, W = depth.shape
        K = live.K
        out.update(cam=cam, K={k: float(v) for k, v in K.items() if isinstance(v, (int, float))}, size=[W, H], hand=list(map(float, va["position_m"])))
        cfg = pk.oc["CFG"]; wx, wy = cfg["workspace_x_m"], cfg["workspace_y_m"]
        xs = np.linspace(wx[0], wx[1], 18); ys = np.linspace(wy[0], wy[1], 15)
        right, up, fwd, pos = [np.asarray(cam[k]) for k in ("right", "up", "forward", "position_m")]
        def seen(shift, z, margin):
            res = np.zeros((len(xs), len(ys)), bool)
            for i, x in enumerate(xs):
                for j, y in enumerate(ys):
                    p = np.array([x, y, z]) - (pos + np.array([shift[0], shift[1], 0.0]))
                    zc = p @ fwd
                    if zc <= 0.05: continue
                    u = K["fx"] * (p @ right) / zc + K["cx"]; w = K["cy"] - K["fy"] * (p @ up) / zc
                    res[i, j] = margin <= u < W - margin and margin <= w < H - margin
            return res
        out["table_z"] = cfg["table_z_m"]
        for z in (0.62, 0.66):
            base = seen((0, 0), z, 45)
            out[f"A_cover_z{z}"] = float(base.mean())
            best = []
            for dx in np.arange(-0.2, 0.31, 0.05):
                for dy in np.arange(-0.2, 0.41, 0.05):
                    s2 = seen((dx, dy), z, 45); u = (base | s2).mean()
                    best.append((round(float(u), 3), round(float(dx), 2), round(float(dy), 2), round(float(s2.mean()), 3)))
            best.sort(reverse=True); out[f"best_shift_z{z}"] = best[:12]
            out[f"A_map_z{z}"] = ["".join("#" if base[i, j] else "." for j in range(len(ys))) for i in range(len(xs))]
        await live.stop()
    except Exception:
        import traceback; out["error"] = traceback.format_exc()
    out["done"] = True
    json.dump(out, open(os.path.join(root, "outputs", "lab", "live", f"view_cover_{tag}.json"), "w"), indent=1, default=str)


def start_vc(tag="v1"):
    builtins._lab_vc_task = asyncio.ensure_future(_vc(tag))
