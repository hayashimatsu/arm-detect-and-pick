"""lab/live_pick.py -- phase 2: video-mode perception + continuous pick-and-place.

  * No detect() photo stops: the wrist D455 streams (lab/live_perception.py); the arm goes to view A once, the tracker fills in
    within ~1 s, then every pick uses the CURRENT tracks.
  * While the arm flies to the next object, the end point is corrected live from the track (retarget), so a stale or biased
    first estimate does not matter.
  * While an object is held (grasp -> release) the tracker is paused (a lifted object would look like a new object).
  * Records an annotated wrist video (class, confidence, world xyz, distance) -> outputs/lab/live/<run>/wrist.mp4.

    exec fast_pick.py, live_perception.py, live_pick.py ; start_live("sphere")
Result: outputs/lab/live/<time>/result.json
"""
import asyncio, builtins, datetime, json, math, os, subprocess, time, traceback
import numpy as np

LROOT = ROOT
LOUT = os.path.join(LROOT, "outputs", "lab", "live")


def _lab_yaws(fp, _orig=None):
    """Wrist angles to try (lab only; the original planner keeps its own list): circle as before; rectangle: across the short side, then every 10 deg around it
    (nearest first, up to +-80), then across the long side. The width across the fingers must still be <= 70 mm, so a long block only allows small angles."""
    if fp[0] == "circle":
        return _orig(fp)
    n = fp[3] + 90.0
    return [n] + [n + s * a for a in range(10, 81, 10) for s in (1, -1)] + [n + 90.0]


class LivePicker(FastPicker):
    def __init__(self):
        super().__init__()
        orig = self.planner.candidate_yaws
        self.planner.candidate_yaws = lambda fp, _o=orig: _lab_yaws(fp, _o)
        self._open_margin0 = self.planner.OPEN_MARGIN_M
        self._fb_margin = None                                           # None = the planner's normal 6 mm finger clearance; 0.002 = last-resort tier (a crowded cluster)
        orig_fb = self.planner.finger_boxes
        self.planner.finger_boxes = lambda c, yaw, margin=None, gap_m=self.planner.FINGER_GAP_OPEN_M, _o=orig_fb: _o(
            c, yaw, margin if margin is not None else (self._fb_margin if self._fb_margin is not None else self.planner.CLEARANCE_M), gap_m)

    async def scan(self, live, target, min_s=0.6, max_s=3.0):
        t0 = self.f
        while True:
            await self.frames(2)
            dt = (self.f - t0) / 60.0
            tk = live.tracks(target, 3)
            if dt >= min_s and tk and all(t["n"] >= 4 for t in tk):
                break
            if dt >= max_s:
                break
        return live.tracks(target, 3)

    def choose(self, live, target, tcp_xy, table_z, exclude_ids, failed_keys=None):
        """Plan every tracked target; return the graspable one nearest to tcp_xy."""
        allt = live.tracks(None, 2); self.last_skips = []
        objs = []
        for t in allt:
            o = dict(t["obs"]); o["center_xyz_m"] = [float(v) for v in t["center"]]; o["class_name"] = t["class_name"]; o["_tid"] = t["id"]; o["id"] = t["id"]; objs.append(o)     # id = track id (unique)
        cands = []
        for o in objs:
            if o["class_name"] != target or o["_tid"] in exclude_ids:
                continue
            fk = (round(o["center_xyz_m"][0] / 0.02), round(o["center_xyz_m"][1] / 0.02))
            if failed_keys and failed_keys.get(fk, 0) >= 2:                # failed twice at this place: leave it (like the original flow)
                self.last_skips.append(dict(track=o["_tid"], cls=o["class_name"], reason="failed twice at this place")); continue
            tk = next(t for t in allt if t["id"] == o["_tid"])
            if tk["n"] < 3:
                continue
            pl = self.planner.plan_grasp(o, [x for x in objs if x is not o], table_z)
            if not pl["ok"]:
                self.last_skips.append(dict(track=o["_tid"], cls=o["class_name"], reason=pl["reason"]))
            if pl["ok"]:
                width = next((t["width_mm"] for t in pl["tried"] if abs(t["yaw_deg"] - round(pl["yaw_deg"], 1)) < 0.2), None)
                cands.append(dict(obj=o, plan=pl, width=width, tid=o["_tid"], d=math.dist(pl["tcp_grasp"][:2], tcp_xy)))
        return min(cands, key=lambda c: c["d"]) if cands else None, len(objs)

    def _objs(self, live):
        objs = []
        for t in live.tracks(None, 2):
            o = dict(t["obs"]); o["center_xyz_m"] = [float(v) for v in t["center"]]; o["class_name"] = t["class_name"]; o["_tid"] = t["id"]; o["id"] = t["id"]; objs.append(o)
        return objs

    def plan_relocation(self, live, target, tcp_xy, table_z, exclude_ids, done_relocs):
        """Every target is blocked (fingers would hit a neighbour): find a neighbour that can be picked up and put down on a free spot of the table such that a blocked
        target becomes graspable, without blocking another one. Returns dict(blocker=q, spot=[x, y], unblocks=[track ids]) or None.
        Everything comes from the tracks (video), nothing from the stage."""
        pl_ = self.planner; objs = self._objs(live); cfg = self.oc["CFG"]
        blocked = []; self.reloc_diag = dict(blocked=[], blockers=[], spots_tried=0, why=None)
        for o in objs:
            if o["class_name"] != target or o["_tid"] in exclude_ids:
                continue
            pl = pl_.plan_grasp(o, [x for x in objs if x is not o], table_z)
            if pl["ok"]:
                return None                                              # a target is graspable after all
            ok_w = [t for t in pl["tried"] if t["width_mm"] <= pl_.MAX_OBJECT_WIDTH_M * 1000]
            if ok_w:
                blocked.append((o, min(ok_w, key=lambda t: len(t["blocked_by"]))["blocked_by"]))
                self.reloc_diag["blocked"].append(dict(tid=o["_tid"], cls=o["class_name"], blockers=blocked[-1][1], tried=[(t["yaw_deg"], t["width_mm"], t["blocked_by"]) for t in pl["tried"]]))
            else:
                self.reloc_diag["blocked"].append(dict(tid=o["_tid"], cls=o["class_name"], why="too wide for every yaw"))
        cands = []; seen_ids = set(); chain = []; frontier = [bid for _, ids in blocked for bid in ids]
        for _depth in range(3):                                      # a blocker may itself be blocked: follow the chain until a neighbour that CAN be picked up
            nxt = []
            for bid in frontier:
                b = next((x for x in objs if x["_tid"] == bid), None)
                if b is None or bid in seen_ids or done_relocs.get(bid, 0) >= 2:
                    continue
                seen_ids.add(bid)
                bp = pl_.plan_grasp(b, [x for x in objs if x is not b], table_z)
                self.reloc_diag["blockers"].append(dict(tid=bid, cls=b["class_name"], graspable=bool(bp["ok"]), reason=None if bp["ok"] else bp["reason"], done=done_relocs.get(bid, 0), depth=_depth))
                if bp["ok"]:
                    cands.append(dict(b=b, plan=bp))
                else:
                    chain.append(b)
                    ok_w = [t for t in bp["tried"] if t["width_mm"] <= pl_.MAX_OBJECT_WIDTH_M * 1000]
                    if ok_w:
                        nxt += min(ok_w, key=lambda t: len(t["blocked_by"]))["blocked_by"]
            frontier = nxt
            if cands:
                break
        cands.sort(key=lambda c: math.dist(c["plan"]["tcp_grasp"][:2], tcp_xy))
        wx, wy = cfg["workspace_x_m"], cfg["workspace_y_m"]; m = 0.05
        for c in cands:
            b = c["b"]; fpb = pl_.footprint(b); r_b = float(fpb[2]) if fpb[0] == "circle" else 0.5 * math.hypot(*fpb[2])
            rest = [x for x in objs if x is not b]
            xs = np.arange(wx[0] + m, wx[1] - m + 1e-9, 0.02); ys = np.arange(wy[0] + m, wy[1] - m + 1e-9, 0.02)
            pts = sorted(((x, y) for x in xs for y in ys), key=lambda q: math.dist(q, b["center_xyz_m"][:2]))
            for (x, y) in pts:
                if math.dist((x, y), b["center_xyz_m"][:2]) < 0.06:
                    continue
                boxes = pl_.finger_boxes([x, y], c["plan"]["yaw_deg"], 0.006, pl_.FINGER_GAP_OPEN_M) + [pl_._rect_corners(np.array([x, y]), (2 * r_b + 0.03, 2 * r_b + 0.03), 0.0)]
                self.reloc_diag["spots_tried"] += 1
                if pl_.blocked_by(boxes, rest):
                    continue
                moved = dict(b, center_xyz_m=[x, y, b["center_xyz_m"][2]])
                after = rest + [moved]
                freed = [o["_tid"] for o in [o_ for o_, _ in blocked] + chain if o is not b and pl_.plan_grasp(o, [z for z in after if z is not o], table_z)["ok"]]      # a target, or a blocked neighbour in the chain, becomes graspable
                others_ok = all(pl_.plan_grasp(o, [z for z in after if z is not o], table_z)["ok"] or not pl_.plan_grasp(o, [z for z in rest if z is not o], table_z)["ok"]
                                for o in objs if o["class_name"] == target and o["_tid"] not in exclude_ids and o is not b)
                if freed and others_ok:
                    return dict(blocker=dict(obj=b, plan=c["plan"], tid=b["_tid"], width=next((t["width_mm"] for t in c["plan"]["tried"] if abs(t["yaw_deg"] - round(c["plan"]["yaw_deg"], 1)) < 0.2), None)),
                                spot=[round(float(x), 4), round(float(y), 4)], unblocks=freed, r=r_b)
        return None

    async def run_live(self, target, max_picks=12, rec=True, debug_release=False, test=None):
        await _PK["ensure_sim"]()
        import omni.usd
        from pxr import Usd
        run = os.path.join(LOUT, datetime.datetime.now().strftime("run_%Y%m%d_%H%M%S")); os.makedirs(run, exist_ok=True)
        st = omni.usd.get_context().get_stage()
        if st.GetPrimAtPath("/World/TestBlocks").IsValid():
            st.SetEditTarget(Usd.EditTarget(st.GetSessionLayer()))
        reg = builtins._arm310d_ik_follow_controller_registry
        g = reg["instance"]._on_update.__globals__
        old_cap = g["MAX_JOINT_DELTA_PER_STEP"]; g["MAX_JOINT_DELTA_PER_STEP"] = P["ik_cap"]
        result = dict(run_dir=run, params=P, target=target); live = None
        try:
            self.t0 = time.time(); truth0 = self.truth(); self.watch = True
            table_z = self.oc["CFG"]["table_z_m"]
            live = Live(self.oc)
            if rec:
                live.rec_dir = os.path.join(run, "frames"); os.makedirs(live.rec_dir, exist_ok=True)
            await live.start()
            builtins._lab_live = live
            Zt = P["transit_z"]
            bmin, bmax = self.basket_box(); base_xy = self.arm_base_xy()
            key_ = (round(bmin[0], 3), round(bmin[1], 3), round(bmax[0], 3), round(bmax[1], 3))
            mem = getattr(builtins, "_lab_basket", None)                                                     # the basket is REMEMBERED between runs (layout() empties it and resets this)
            if not mem or mem["key"] != key_:
                mem = builtins._lab_basket = dict(key=key_, map=BasketMap(bmin, bmax, P["floor_z"], bmax[2], P["wall_margin"], known_empty=bool(getattr(builtins, "_lab_basket_empty", False))))
                builtins._lab_basket_empty = False
            bmap = live.basket_map = mem["map"]                                                              # + updated from the depth video whenever the hand is high near the basket
            live.basket_gate = lambda: (lambda t: t[2] >= Zt - 0.02 and t[0] >= bmin[0] - 0.35)(self.tcp()[0]); live.basket_on = True
            home_p, home_q = self.oc["_pose"]("/World/IKTarget")            # where the arm started = where it returns to
            await self.open_to(0.0, 3.0, 40)                          # start with an empty, open gripper
            # 1) to view A (one continuous leg), scan while video runs
            va = self.oc["_view_poses"]()[0]
            cur_p, _ = self.oc["_pose"]("/World/IKTarget")
            await self.fly([[cur_p[0], cur_p[1], max(cur_p[2], va["position_m"][2])], va["position_m"]], va["quaternion_wxyz"], "to_view", [P["fillet"]])
            await self.arrive(va["position_m"], 0.004, 90, va["quaternion_wxyz"])
            self.mark("at_view", basket_known=round(float(np.mean(~np.isnan(bmap.top[3:-3, 3:-3]))), 3), stamps=len(bmap.stamps)); live.hud = "scan"
            tk = await self.scan(live, target)
            self.mark("scan_done", n_tracks=len(live.tracks()), n_target=len(tk))
            result["scan_s"] = round(self.f / 60.0, 2)
            cells = self.basket_cells(bmin, bmax, base_xy)                                                 # only the last resort when the map finds no free spot
            cycles = []; done_ids = set(); tries = {}; failed_keys = {}; k = 0; done_relocs = {}; n_reloc = 0
            while k < max_picks + P["max_relocations"]:
                tcp_p, _ = self.tcp(); reloc = None; self._fb_margin = None; self.planner.OPEN_MARGIN_M = self._open_margin0
                if k > 0 and not [t for t in live.tracks(target, 3) if t["id"] not in done_ids]:
                    self.mark("all_targets_done")                           # every target seen has been picked: go home now, no extra look
                    break
                q, n_obj = self.choose(live, target, tcp_p[:2], table_z, done_ids, failed_keys)
                if q is None:
                    tk = await self.scan(live, target, 0.4, 1.0); q, n_obj = self.choose(live, target, tcp_p[:2], table_z, done_ids, failed_keys)
                    if q is None:
                        reloc = self.plan_relocation(live, target, tcp_p[:2], table_z, done_ids, done_relocs) if n_reloc < P["max_relocations"] else None
                        if reloc is None:                                             # crowded cluster: last resort, finger clearance 2 mm instead of 6 mm (first for the target itself, then for a neighbour)
                            self._fb_margin = P["tight_margin"]; self.planner.OPEN_MARGIN_M = P["tight_open_margin"]            # also open the gripper only 10 mm (not 20 mm) wider than the block
                            q, n_obj = self.choose(live, target, tcp_p[:2], table_z, done_ids, failed_keys)
                            if q is not None:
                                c_tight = True
                            elif n_reloc < P["max_relocations"]:
                                reloc = self.plan_relocation(live, target, tcp_p[:2], table_z, done_ids, done_relocs)
                            if q is None and reloc is None:                          # still nothing (a ring of blocks that block each other): fingers may touch a neighbour by 3 mm (pads slide over its edge / nudge it)
                                self._fb_margin = P["touch_margin"]
                                q, n_obj = self.choose(live, target, tcp_p[:2], table_z, done_ids, failed_keys)
                            if q is None and reloc is None:
                                self.mark("nothing_graspable", skipped=self.last_skips, reloc=getattr(self, "reloc_diag", None))
                                break
                        if reloc is not None:
                            q = reloc["blocker"]; n_reloc += 1                      # every target is blocked by a neighbour: move that neighbour out of the way first
                pl = q["plan"]; g_ = list(pl["tcp_grasp"]); qd = list(self.pm.quat_down_with_yaw(pl["yaw_deg"]))
                fpo = self.planner.footprint(q["obj"]); r_obj = float(fpo[2]) if fpo[0] == "circle" else 0.5 * math.hypot(*fpo[2])
                gap_open = self.planner.FINGER_GAP_OPEN_M                    # the release opens the gripper fully: the open fingers must also find room
                if reloc:
                    spot = dict(xy=list(reloc["spot"]), kind="table", unblocks=reloc["unblocks"])
                else:
                    spot = bmap.pick_spot(r_obj, pl["yaw_deg"], gap_open, self.planner, base_xy)
                    if spot is None:
                        spot = dict(xy=list(cells[k % len(cells)]), gap_mm=None, n_stamps=len(bmap.stamps))     # basket full: best effort
                cell = list(spot["xy"])
                c = dict(k=k + 1, tid=q["tid"], cls=q["obj"]["class_name"], conf=q["obj"]["conf"], plan_grasp=g_, cell=list(cell), spot=spot, place_changes=[], kind="relocate" if reloc else "pick", tight=self._fb_margin is not None)
                live.hud = f"{'move aside' if reloc else 'pick'} {k+1}: {c['cls']} track {q['tid']}"
                preshape = [pl["open_deg"]]                              # applied only once the hand is clear of the block it just released (feedback-2 #1)
                def _preshape_hook():
                    if preshape and self.tcp()[0][2] >= Zt - P["preshape_clear_m"]:
                        self.grip["gripper_set_angle_deg"](preshape.pop())
                cur_p, _ = self.oc["_pose"]("/World/IKTarget")
                pts, rad = [], []
                if cur_p[2] < Zt - 0.02:
                    pts.append([cur_p[0], cur_p[1], Zt]); rad.append(P["fillet"])
                pts.append([g_[0], g_[1], Zt]); rad.append(P["fillet_down"]); pts.append(g_)
                tid = q["tid"]
                gt_name = min(truth0, key=lambda n: math.dist(truth0[n][:2], g_[:2])) if truth0 else None     # report only

                if test and test.get("shift") and k == 0:                 # test only: move the block after the scan (stage edit, invisible to the method)
                    from isaacsim.core.experimental.prims import RigidPrim
                    rp_ = self.oc["_cached_prim"](RigidPrim, f"/World/TestBlocks/{gt_name}")
                    pos_, quat_ = rp_.get_world_poses(); pos_ = pos_.numpy().copy(); pos_[0, 0] += test["shift"][0]; pos_[0, 1] += test["shift"][1]
                    rp_.set_world_poses(pos_, quat_.numpy().copy()); rp_.set_velocities(np.zeros((1, 3)), np.zeros((1, 3))); c["test_shift_m"] = test["shift"]
                g0 = [g_[0], g_[1]]; end_xy = [g_[0], g_[1]]; c["refine_log"] = []
                others_xy = [np.array(t["center"][:2]) for t in live.tracker.tr if t["id"] != tid]       # the neighbours we already know: never the block we are going for
                fp_ = self.planner.footprint(q["obj"])                       # expected blob extent (mm): a circle -> diameter; a rectangle -> short side .. diagonal (any yaw)
                ext_lo, ext_hi = (2000.0 * fp_[2],) * 2 if fp_[0] == "circle" else (1000.0 * min(fp_[2]), 1000.0 * math.hypot(*fp_[2]))

                def retarget(tp, tid=tid, g_=g_):
                    mode = P["visual_refine"]
                    if mode and g_[2] + P["refine_z_m"][0] <= tp[2] <= g_[2] + P["refine_z_m"][1]:
                        # Final correction from the video while the hand comes down (no stop, no extra leg). The class-free depth blob of the target:
                        # "change": used only when it is far from the scanned position (the block moved), because its centroid is 2-6 mm off (worse than the
                        # still-scan track, 0.1-0.6 mm); "estimate": always (kept to measure the difference).
                        b, age = live.blob_near(end_xy, 0.05, 0.25)
                        if b is not None and (not (ext_lo - 15 <= min(b["extent_mm"]) and max(b["extent_mm"]) <= ext_hi + 15)
                                              or any(float(np.hypot(b["xy"][0] - o_[0], b["xy"][1] - o_[1])) < 0.02 for o_ in others_xy)):
                            b = None                                       # not our block: wrong size (against ITS footprint: short side .. diagonal), or a known neighbour (or the hand)
                        if b is not None:
                            d = float(np.hypot(b["xy"][0] - g0[0], b["xy"][1] - g0[1]))
                            c["refine_log"].append(dict(tcp_z=round(float(tp[2]), 3), d_mm=round(1000 * d, 1)))
                            if mode == "estimate" or d > P["moved_m"]:
                                end_xy[0], end_xy[1] = b["xy"][0], b["xy"][1]
                                return b["xy"]
                        return None
                    if tp[2] < g_[2] + 0.10:                              # last 10 cm: the hand is in the way of the camera
                        return None
                    tk = next((t for t in live.tracker.tr if t["id"] == tid), None)
                    if tk is None or tk["n"] < 3 or live.sim_f / 60.0 - tk["last_t"] > 0.5:
                        return None
                    return tk["center"][:2]
                self.mark(f"k{k+1}:leg_start")
                self.retargets = 0
                await self.fly(pts, qd, f"k{k+1}:to_grasp", rad, retarget=retarget, on_frame=_preshape_hook)
                if preshape:
                    self.grip["gripper_set_angle_deg"](preshape.pop())
                c["retargets"] = self.retargets
                live.paused = True                                       # from here the hand hides the object: freeze the tracker
                await self.arrive([end_xy[0], end_xy[1], g_[2]], 0.003, 150, qd)      # the corrected end point, not the planned one (else it waits the full 2.5 s)
                tkn = next((t for t in live.tracker.tr if t["id"] == tid), None)
                tcp_p, _ = self.tcp()
                c["tcp_at_close"] = [round(float(v), 4) for v in tcp_p]
                tru_ = self.truth().get(gt_name) if gt_name else None                       # report only: how far from the block's true centre the hand is when it closes
                if tru_:
                    c["grasp_err_mm"] = round(1000 * math.dist(tcp_p[:2], tru_[:2]), 1)
                self.mark(f"k{k+1}:at_grasp"); t_arr = self.f
                if getattr(self, "dbg", None) is not None and debug_release == "next":
                    c["release_debug_prev"] = dict(rows=self.dbg.rows, api_error=self.dbg.err); self.dbg.close(); self.dbg = None
                c["grip"] = await self.close_once(q["width"] or 40.0)
                self.mark(f"k{k+1}:closed", holding=c["grip"]["holding"], frames=c["grip"]["frames"])
                pred_c = c["grip"]["theta_contact_pred"]; con_c = c["grip"]["contact_deg"]
                c["width_mismatch_deg"] = None if con_c is None else round(con_c - pred_c, 2)       # information only: the squeeze corrects small yaw errors
                gap_c = None if con_c is None else self.gap_mm_for_deg(con_c)
                short_mm = self.short_side_mm(q["obj"]); plan_w = q["width"] or short_mm
                c["grip_width_mm"] = dict(closed=None if gap_c is None else round(gap_c, 1), planned=round(plan_w, 1), short_side=round(short_mm, 1))
                c["width_mismatch"] = bool(gap_c is None or not (P["width_tol_mm"][0] * short_mm <= gap_c <= plan_w + P["width_tol_mm"][1]))
                firm = c["grip"]["holding"] and not c["width_mismatch"]
                tugs = 0
                x_p, y_p = g_[0], g_[1]
                if firm:                                                      # tug test: lift 12 mm, the finger must not move and must keep pushing
                    while True:
                        a0, _ = self.grip_firm()
                        await self.fly([[x_p, y_p, g_[2] + P["tug_lift_m"]]], qd, f"k{k+1}:tug", [], carrying=True)
                        a1, lag1 = self.grip_firm(); tugs += 1
                        if a1 - a0 < P["tug_slip_deg"] and lag1 >= P["min_lag_deg"]:
                            break                                             # firm: carry on
                        if tugs >= 3:
                            firm = False; break
                        await self.fly([g_], qd, f"k{k+1}:back_down", [], carrying=True)       # slipping: set it down, squeeze harder, try again
                        await self.squeeze_more(P["squeeze_extra_deg"])
                    c["tug"] = dict(tests=tugs, finger_deg=round(a1, 2), lag_deg=round(lag1, 2))
                if firm:
                    cur_p, _ = self.oc["_pose"]("/World/IKTarget")
                    hold_off = g_[2] - table_z
                    z_place = (table_z if reloc else P["floor_z"]) + hold_off + P["place_clear"]
                    pts = [[cur_p[0], cur_p[1], Zt], [cell[0], cell[1], Zt], [cell[0], cell[1], z_place]]

                    def retarget_place(tp):
                        # In flight, while the hand is still high and far from the spot: the depth video now sees the basket. If the chosen spot is not free any more
                        # (something rolled there / was never seen), take the nearest free one -- the end point of the leg is blended over to it, no stop.
                        if tp[2] < Zt - 0.03 or math.dist(tp[:2], cell) < 0.10 or bmap.fresh < 2:
                            return None
                        free, verified = bmap.evaluate(cell, r_obj, pl["yaw_deg"], gap_open, self.planner)
                        c["place_verified"] = verified
                        if free:
                            return None
                        new = bmap.pick_spot(r_obj, pl["yaw_deg"], gap_open, self.planner, base_xy, near=cell)
                        if new is None or math.dist(new["xy"], cell) < 0.01:
                            return None
                        c["place_changes"].append(dict(from_xy=[round(v, 4) for v in cell], to_xy=new["xy"], tcp_xy=[round(float(v), 3) for v in tp[:2]]))
                        cell[0], cell[1] = new["xy"]
                        return list(cell)
                    bmap.fresh = 0
                    await self.fly(pts, qd, f"k{k+1}:carry_to_place", [P["fillet"], P["fillet_down"]], carrying=True, retarget=None if reloc else retarget_place)
                    c["cell"] = [round(v, 4) for v in cell]; c["basket_frames_seen"] = bmap.fresh
                    await self.arrive([cell[0], cell[1], z_place], 0.004, 150, qd)
                    c["finger_deg_at_place"] = round(self.finger_deg(), 2); self.mark(f"k{k+1}:at_place")
                    if debug_release:                                         # diagnosis: per-frame block/hand record, original fast leave (no slow first lift)
                        self.dbg = ReleaseDebug(self, gt_name); self.dbg_label = "release"
                    c["release"] = await self.release(gt_name)
                    if not reloc:
                        bmap.stamps.append((cell[0], cell[1], r_obj + 0.005))
                    self.mark(f"k{k+1}:released", frames=c["release"]["frames"])
                    cur_p, _ = self.oc["_pose"]("/World/IKTarget")
                    lift_tr = []
                    if debug_release == "next":                               # keep recording through the NEXT pick's approach (normal flow, nothing changed)
                        self.dbg_label = "next"
                    elif debug_release:
                        self.dbg_label = "leave"
                        if debug_release == "rounded":                       # as in t5: the next leg starts up, but the corner is rounded (r = P["fillet"]) so sideways motion starts early
                            await self.fly([[cur_p[0], cur_p[1], Zt], [cur_p[0] - 0.30, cur_p[1], Zt]], qd, f"k{k+1}:leave_rounded", [P["fillet"]])
                        else:                                                # straight up to the travel height
                            await self.fly([[cur_p[0], cur_p[1], Zt]], qd, f"k{k+1}:leave_fast", [])
                        await self.frames(20)
                        c["release_debug"] = dict(rows=self.dbg.rows, api_error=self.dbg.err); self.dbg.close(); self.dbg = None
                    elif P.get("slow_leave", True):                              # leave slowly first; trace where the block is relative to the hand
                        def _tr():
                            if self.f % 3 == 0:
                                lift_tr.append(self.block_in_hand_mm(gt_name))
                        await self.fly([[cur_p[0], cur_p[1], cur_p[2] + 0.03]], qd, f"k{k+1}:leave_slow", [], on_frame=_tr, vmax=0.12, amax=1.0)
                    c["leave_trace_block_in_hand_mm"] = lift_tr
                    c["from_arrive_to_release_s"] = round((self.f - t_arr) / 60.0, 2)
                    c["holding_at_place"] = bool(self.hold_cmd - c["finger_deg_at_place"] >= P["min_lag_deg"])
                else:                                                         # do not carry a bad grasp: open, go up, look again
                    await self.open_to(0.0, 3.0, 30)
                    cur_p, _ = self.oc["_pose"]("/World/IKTarget")
                    await self.fly([[cur_p[0], cur_p[1], Zt]], qd, f"k{k+1}:abort_up", [])
                    self.mark(f"k{k+1}:abort", pred=pred_c, contact=con_c, tugs=tugs)
                    c["finger_deg_at_place"] = None; c["from_arrive_to_release_s"] = round((self.f - t_arr) / 60.0, 2); c["holding_at_place"] = False
                ok_pick = c["holding_at_place"]
                c["ok"] = ok_pick
                tries[tid] = tries.get(tid, 0) + 1
                if reloc:
                    done_relocs[tid] = done_relocs.get(tid, 0) + 1
                elif ok_pick or tries[tid] >= 2:
                    done_ids.add(tid)
                for t_ in live.tracker.tr:
                    if t_["id"] == tid and ok_pick and not reloc:
                        t_["picked"] = True
                if not ok_pick or reloc:                                           # failed grasp: forget the track, look again
                    fk = (round(g_[0] / 0.02), round(g_[1] / 0.02))
                    if not reloc:
                        failed_keys[fk] = failed_keys.get(fk, 0) + 1
                    t_ = next((t for t in live.tracker.tr if t["id"] == tid), None)
                    if t_ is not None:
                        live.tracker.tr.remove(t_)
                await self.frames(6)
                live.paused = False
                if not ok_pick or reloc:                                  # back to view A to find the object again (a moved neighbour: see where it is now)
                    cur_p, _ = self.oc["_pose"]("/World/IKTarget")
                    await self.fly([[cur_p[0], cur_p[1], Zt], va["position_m"]], va["quaternion_wxyz"], f"k{k+1}:rescan_view", [P["fillet"]])
                    await self.scan(live, target, 1.2 if reloc else 0.8, 2.5)
                cycles.append(c); k += 1
            live.hud = "done"
            cur_p, _ = self.oc["_pose"]("/World/IKTarget")
            await self.fly([[cur_p[0], cur_p[1], Zt], [home_p[0], home_p[1], Zt], home_p], home_q, "to_home", [P["fillet"]] * 2)
            await self.arrive(home_p, 0.004, 60); self.mark("home")
            truth1 = self.truth()
            json.dump(self.watch_log, open(os.path.join(run, "truth_watch.json"), "w"))
            result.update(cycles=cycles, truth_before=truth0, truth_after=truth1, events=self.ev,
                          in_basket_after={n: bool(bmin[0] < p[0] < bmax[0] and bmin[1] < p[1] < bmax[1] and p[2] < bmax[2] + 0.02) for n, p in truth1.items()},
                          total_sim_s=round(self.f / 60.0, 2), wall_s=round(time.time() - self.t0, 1),
                          live=dict(app_frames=live.sim_f, cam_frames=live.n_frames, processed=live.n_proc,
                                    latency_ms_mean=round(float(np.mean(live.lat_ms)), 0) if live.lat_ms else None, errors=live.errors[:5]))
            # truth-vs-track report (stage truth only for the report)
            rows = []
            for t in live.tracker.tr:
                nm = min(truth0, key=lambda n: math.dist(truth0[n][:2], t["center"][:2]))
                rows.append(dict(track=t["id"], cls=t["class_name"], conf=round(t["conf"], 2), n=t["n"], truth=nm,
                                 err_xy_mm=round(1000 * math.dist(truth0[nm][:2], t["center"][:2]), 1), picked=t["picked"]))
            result["tracks_vs_truth"] = rows
            json.dump(live.obs_log, open(os.path.join(run, "obs_log.json"), "w"))
            result["done"] = True
        except Exception:
            result["error"] = traceback.format_exc(); result["done"] = True; result["events"] = self.ev
        finally:
            g["MAX_JOINT_DELTA_PER_STEP"] = old_cap
            try:
                if live is not None:
                    await live.stop()
            except Exception:
                pass
            json.dump(result, open(os.path.join(run, "result.json"), "w"), indent=1, default=str)
            if rec and live is not None and live.rec_n:
                try:
                    subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-framerate", "12", "-i", os.path.join(run, "frames", "f%05d.jpg"),
                                      "-c:v", "libx264", "-pix_fmt", "yuv420p", os.path.join(run, "wrist.mp4")])
                except Exception:
                    pass
            builtins._lab_live_last = run
            try:
                _write_html(run, result)
            except Exception:
                pass
        return result


def _write_html(run, r):
    """Small report: video + per-pick times + what ended up in the basket (stage truth is for this report only)."""
    rows = "".join(f"<tr><td>{c['k']}</td><td>{c['cls']}{' (moved aside)' if c.get('kind') == 'relocate' else ''}</td><td>{c['conf']:.2f}</td><td>{'yes' if c.get('ok') else 'no'}</td>"
                   f"<td>{c['from_arrive_to_release_s']}</td></tr>" for c in r.get("cycles", []))
    ev = {e["name"]: e["sim_s"] for e in r.get("events", [])}
    rel = [e["sim_s"] for e in r.get("events", []) if e["name"].endswith(":released")]
    gaps = ", ".join(f"{b - a:.1f}" for a, b in zip(rel, rel[1:])) or "-"
    tb, ta, inb = r.get("truth_before", {}), r.get("truth_after", {}), r.get("in_basket_after", {})
    moved = "".join(f"<tr><td>{n}</td><td>{'in basket' if inb.get(n) else 'on table'}</td></tr>" for n in ta if ta[n] != tb.get(n))
    html = f"""<!doctype html><meta charset=utf-8><title>detect_pick (video mode)</title>
<style>body{{font:14px system-ui;max-width:1000px;margin:24px auto;padding:0 16px}}td,th{{border-bottom:1px solid #ddd;padding:6px;text-align:left}}video{{max-width:100%}}</style>
<h2>detect_pick({r.get('target')!r}) -- video mode</h2>
<p>Scan {ev.get('scan_done', 0):.1f} s, total {r.get('total_sim_s')} sim-s. Seconds from arriving at an object to releasing it: see table.
Seconds from one release to the next: {gaps}.</p>
<video src="wrist.mp4" controls autoplay loop muted></video>
<table><tr><th>pick</th><th>class</th><th>conf</th><th>holding at the basket</th><th>arrive to release (s)</th></tr>{rows}</table>
<h3>Blocks that moved (stage truth, report only)</h3><table>{moved}</table>
<p>{r.get('error') or ''}</p>"""
    open(os.path.join(run, "index.html"), "w", encoding="utf-8").write(html)


def start_live(target="sphere", max_picks=12, rec=True, debug_release=False, test=None):
    builtins._lab_live_task = asyncio.ensure_future(LivePicker().run_live(target, max_picks, rec, debug_release, test))
