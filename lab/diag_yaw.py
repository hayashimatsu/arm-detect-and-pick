"""lab/diag_yaw.py -- are the tracked footprint size / yaw (what the grasp planner uses for cube and cuboid) good?
Parks the arm at view A like the scan does, tracks for `secs`, compares each box track with the stage truth (report only)."""
import asyncio, builtins, json, math, os
import numpy as np


async def dy(tag, secs=1.5, still=True):
    import omni.kit.app
    from isaacsim.core.experimental.prims import RigidPrim
    app = omni.kit.app.get_app(); out = {}
    try:
        await _PK["ensure_sim"]()
        oc = builtins._lab_oc; pose = oc["_pose"]
        live = Live(oc); await live.start()
        va = oc["_view_poses"]()[0]; p0, q0 = pose("/World/IKTarget")
        for i in range(1, 121):
            u = i / 120; s = u * u * (3 - 2 * u)
            pose("/World/IKTarget", (np.asarray(p0) + s * (np.asarray(va["position_m"]) - np.asarray(p0))).tolist(), oc["_slerp"](q0, va["quaternion_wxyz"], s))
            await app.next_update_async()
        for _ in range(int(secs * 60)): await app.next_update_async()
        tr = {}
        import omni.usd
        st = omni.usd.get_context().get_stage()
        from pxr import UsdGeom
        for c in [c for c in st.GetPrimAtPath("/World/TestBlocks").GetChildren() if c.IsA(UsdGeom.Gprim)]:
            p, q = oc["_cached_prim"](RigidPrim, str(c.GetPath())).get_world_poses()
            q = q.numpy()[0]; w, x, y, z = q
            tr[c.GetName()] = (p.numpy()[0][:2].tolist(), math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))))
        rows = []
        for t in live.tracks(None, 2):
            if t["class_name"] not in ("cube", "cuboid"):
                continue
            nm = min(tr, key=lambda n: math.dist(tr[n][0], t["center"][:2]))
            ty = tr[nm][1]; per = 90 if nm.startswith("cube") else 180
            def e(yaw):                                                    # yaw error mod the shape's symmetry, degrees
                return round(((yaw - ty + per / 2) % per) - per / 2, 1)
            hist = [o for o in getattr(live, "obs_log", [])]
            o = t["obs"]
            rows.append(dict(track=t["id"], truth=nm, cls=t["class_name"], n=t["n"], top_yaw_err_last=e(o.get("top_face_yaw_deg", 0.0)),
                             size_last=o.get("top_face_size_mm"), size_truth=nm, conf=round(t["conf"], 2)))
        out = dict(rows=rows)
        await live.stop()
    except Exception:
        import traceback; out["error"] = traceback.format_exc()
    json.dump(out, open(os.path.join(ROOT, "outputs", "lab", "live", f"diagyaw_{tag}.json"), "w"), indent=1, default=str)


def start_dy(tag="y1", secs=1.5):
    builtins._lab_dy_task = asyncio.ensure_future(dy(tag, secs))
