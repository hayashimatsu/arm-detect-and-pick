"""lab/phys_test.py -- feedback-4 #1: how far does a launched ball / lying cylinder roll, and what stops it?
Launch (rolling start: omega = v / R) of one block from a fixed start in a fixed direction; the other blocks are parked away.
Variants: table/basket friction material, rigid body angular / linear damping.  Writes outputs/lab/live/phys_<tag>.json
   exec(open(...).read()); start_phys("sphere_60", "p1", [("baseline", None, None)])      needs builtins._lab_ds2 (demo_start2 globals)
The settings used here are authored in the SESSION layer only (never saved into scenes/*.usd)."""
import asyncio, builtins, json, math, os
import numpy as np
import omni.kit.app, omni.timeline, omni.usd

LAUNCHES = [((-0.55, -0.10), (0.0, 1.0)), ((-0.45, -0.10), (0.0, 1.0)), ((-0.55, 0.10), (0.0, -1.0)), ((-0.35, 0.00), (-1.0, 0.0)), ((-0.50, 0.00), (0.0, 1.0))]   # (start xy, direction)
TABLE_X, TABLE_Y, TABLE_Z = (-1.237, 1.237), (-0.381, 0.381), 0.595


def set_materials(mu):
    """Physics material (static = dynamic = mu, restitution 0) on the table collider and on the basket (session layer). mu None = leave as is."""
    if mu is None:
        return "unchanged"
    from pxr import Usd, UsdShade, UsdPhysics, Sdf
    st = omni.usd.get_context().get_stage(); done = []
    with Usd.EditContext(st, st.GetSessionLayer()):
        path = "/World/LabPhysicsMaterial"
        m = UsdShade.Material.Define(st, path); a = UsdPhysics.MaterialAPI.Apply(m.GetPrim())
        a.CreateStaticFrictionAttr(float(mu)); a.CreateDynamicFrictionAttr(float(mu)); a.CreateRestitutionAttr(0.0)
        mat = UsdShade.Material(st.GetPrimAtPath(path))
        for p in st.Traverse():
            n = str(p.GetPath())
            if "SM_HeavyDutyPackingTable_C02_01" in n and p.HasAPI(UsdPhysics.CollisionAPI):
                UsdShade.MaterialBindingAPI.Apply(p).Bind(mat, UsdShade.Tokens.strongerThanDescendants, "physics"); done.append(n[-40:])
        cont = st.GetPrimAtPath("/World/container_h20")
        if cont.IsValid():
            UsdShade.MaterialBindingAPI.Apply(cont).Bind(mat, UsdShade.Tokens.strongerThanDescendants, "physics"); done.append("/World/container_h20")
    return done


def set_damping(names, ang, lin):
    from pxr import Usd, PhysxSchema
    st = omni.usd.get_context().get_stage()
    with Usd.EditContext(st, st.GetSessionLayer()):
        for n in names:
            p = st.GetPrimAtPath(f"/World/TestBlocks/{n}")
            r = PhysxSchema.PhysxRigidBodyAPI.Apply(p)
            if ang is not None: r.CreateAngularDampingAttr(float(ang))
            if lin is not None: r.CreateLinearDampingAttr(float(lin))


async def launch(fp, name, start, direction, speed, radius, max_s=12.0):
    from isaacsim.core.experimental.prims import RigidPrim
    app = omni.kit.app.get_app(); rp = RigidPrim(f"/World/TestBlocks/{name}")
    pos, quat = rp.get_world_poses(); z = float(pos.numpy()[0][2])
    ux, uy = direction; vx, vy = speed * ux, speed * uy
    rp.set_world_poses(np.array([[start[0], start[1], z]]), np.array([[1.0, 0.0, 0.0, 0.0]]))
    rp.set_velocities(np.array([[vx, vy, 0.0]]), np.array([[vy / radius, -vx / radius, 0.0]]))
    track = []; still = 0; t = 0
    for i in range(int(max_s * 60)):
        await app.next_update_async()
        if i % 3: continue
        p = rp.get_world_poses()[0].numpy()[0]; track.append((round(i / 60.0, 2), float(p[0]), float(p[1]), float(p[2])))
        if p[2] < TABLE_Z - 0.05 or not (TABLE_X[0] < p[0] < TABLE_X[1] and TABLE_Y[0] < p[1] < TABLE_Y[1]):
            return dict(outcome="fell off the table", dist_cm=round(100 * math.dist(start, (p[0], p[1])), 1), t_s=round(i / 60.0, 2))
        if len(track) > 2 and math.dist(track[-1][1:3], track[-2][1:3]) < 0.0004:
            still += 1
            if still >= 12:
                return dict(outcome="stopped", dist_cm=round(100 * math.dist(start, (p[0], p[1])), 1), t_s=round(i / 60.0, 2))
        else:
            still = 0
    p = track[-1]
    return dict(outcome="still rolling at %.0f s" % max_s, dist_cm=round(100 * math.dist(start, (p[1], p[2])), 1), t_s=max_s)


def _rot(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


async def launch_cyl(name, start, speed, radius, max_s=12.0):
    """Lying cylinder launched ACROSS its axis (the only direction it can roll) with a rolling start."""
    from pxr import UsdGeom
    from isaacsim.core.experimental.prims import RigidPrim
    app = omni.kit.app.get_app(); rp = RigidPrim(f"/World/TestBlocks/{name}")
    st = omni.usd.get_context().get_stage(); ax = UsdGeom.Cylinder(st.GetPrimAtPath(f"/World/TestBlocks/{name}")).GetAxisAttr().Get() or "Z"
    pos, quat = rp.get_world_poses(); q = quat.numpy()[0]; z = float(pos.numpy()[0][2])
    a = _rot(q) @ np.array({"X": [1, 0, 0], "Y": [0, 1, 0], "Z": [0, 0, 1]}[ax]); a = a[:2] / max(np.linalg.norm(a[:2]), 1e-9)
    d = np.array([-a[1], a[0]]); ux, uy = d
    rp.set_world_poses(np.array([[start[0], start[1], z]]), quat.numpy().copy())
    rp.set_velocities(np.array([[speed * ux, speed * uy, 0.0]]), np.array([[speed * uy / radius, -speed * ux / radius, 0.0]]))
    track = []; still = 0
    for i in range(int(max_s * 60)):
        await app.next_update_async()
        if i % 3: continue
        p = rp.get_world_poses()[0].numpy()[0]; track.append((p[0], p[1]))
        if p[2] < TABLE_Z - 0.05 or not (TABLE_X[0] < p[0] < TABLE_X[1] and TABLE_Y[0] < p[1] < TABLE_Y[1]):
            return dict(outcome="fell off the table", dist_cm=round(100 * math.dist(start, (p[0], p[1])), 1), t_s=round(i / 60.0, 2), axis_xy=[round(float(v), 2) for v in a])
        if len(track) > 2 and math.dist(track[-1], track[-2]) < 0.0004:
            still += 1
            if still >= 12:
                return dict(outcome="stopped", dist_cm=round(100 * math.dist(start, (p[0], p[1])), 1), t_s=round(i / 60.0, 2), axis_xy=[round(float(v), 2) for v in a])
        else:
            still = 0
    return dict(outcome="still rolling", dist_cm=round(100 * math.dist(start, track[-1]), 1), t_s=max_s, axis_xy=[round(float(v), 2) for v in a])


async def _phys(name, tag, variants, speed, n_launch, seed=31):
    g = builtins._lab_ds2; reg = builtins._nhc_d455_yolo2_demo; root = g["ROOT"]; rows = []
    omni.timeline.get_timeline_interface().set_auto_update(True)
    g["layout"]("sphere" if name.startswith("sphere") else ("cylinder" if name.startswith("cylinder") else None), seed=seed)
    for _ in range(60 * 60 * 3):
        await omni.kit.app.get_app().next_update_async()
        if reg["task"] is None or reg["task"].done(): break
    for _ in range(30): await omni.kit.app.get_app().next_update_async()
    ns = g["_load_lab"](); fp = ns["FastPicker"]()
    names = list(fp.truth().keys()); R = {"sphere_60": 0.03, "sphere_50": 0.025, "sphere_40": 0.02, "cylinder_d40_l60_lying": 0.02}.get(name, 0.03)
    from isaacsim.core.experimental.prims import RigidPrim
    for n in names:                                                    # park every other block far away (out of the way, off the table)
        if n != name:
            rp = fp.oc["_cached_prim"](RigidPrim, f"/World/TestBlocks/{n}"); p, q = rp.get_world_poses(); pp = p.numpy().copy(); pp[0, 0] = 3.0 + 0.2 * names.index(n); pp[0, 1] = 3.0
            rp.set_world_poses(pp, q.numpy().copy()); rp.set_velocities(np.zeros((1, 3)), np.zeros((1, 3)))
    for vname, mu, damp in variants:
        tl = omni.timeline.get_timeline_interface(); tl.stop()                                  # change physics settings with the timeline stopped, then play
        for _ in range(10): await omni.kit.app.get_app().next_update_async()
        mats = set_materials(mu)
        if damp is not None:
            set_damping([name], damp[0], damp[1])
        tl.play()
        for _ in range(90): await omni.kit.app.get_app().next_update_async()
        for n in names:                                                                         # park the other blocks again (stop/play put them back)
            if n != name:
                rp2 = RigidPrim(f"/World/TestBlocks/{n}"); p, q = rp2.get_world_poses(); pp = p.numpy().copy(); pp[0, 0] = 3.0 + 0.2 * names.index(n); pp[0, 1] = 3.0
                rp2.set_world_poses(pp, q.numpy().copy()); rp2.set_velocities(np.zeros((1, 3)), np.zeros((1, 3)))
        for _ in range(20): await omni.kit.app.get_app().next_update_async()
        for i, (st_, dr) in enumerate(LAUNCHES[:n_launch]):
            res = await (launch_cyl(name, st_, speed, R) if name.startswith("cylinder") else launch(fp, name, st_, dr, speed, R))
            rows.append(dict(variant=vname, table_mu=mu, damping=damp, launch=i, start=st_, direction=dr, **res))
            json.dump(dict(done=False, block=name, speed=speed, rows=rows), open(os.path.join(root, "outputs", "lab", "live", f"phys_{tag}.json"), "w"), indent=1, default=str)
    json.dump(dict(done=True, block=name, speed=speed, rows=rows), open(os.path.join(root, "outputs", "lab", "live", f"phys_{tag}.json"), "w"), indent=1, default=str)


def start_phys(name, tag, variants, speed=0.3, n_launch=5):
    builtins._lab_phys_task = asyncio.ensure_future(_phys(name, tag, variants, speed, n_launch))
