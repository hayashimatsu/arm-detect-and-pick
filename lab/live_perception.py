"""lab/live_perception.py -- video mode: the wrist D455 stays on, every rendered frame goes through YOLO + depth, and
object class / confidence / world position are kept up to date in a tracker while the arm moves.

Pieces
  LiveCam    render product always on; annotators rgb + depth + camera_params (the camera pose of THE RENDERED frame,
             so a frame that arrives 1-2 app frames late is still converted with the right pose)
  worker     (thread) PNG -> /dev/shm, YOLO (lab/yolo_live.py, port 18778), depth -> world, per-object estimate
  Tracker    world-frame tracks (class vote, running median of the centre), poll with live.tracks()
Use:  live = Live(oc); await live.start(); ... live.tracks("sphere") ...; await live.stop()
"""
import asyncio, builtins, concurrent.futures, json, math, os, socket, subprocess, sys, time
import numpy as np

def _project_root():
    """Project root: parent of lab/ when loaded with __file__ set, else parent of the folder holding the open scene."""
    f = globals().get("__file__")
    if f:
        return os.path.dirname(os.path.dirname(os.path.abspath(f)))
    import omni.usd
    return os.path.dirname(os.path.dirname(os.path.abspath(omni.usd.get_context().get_stage().GetRootLayer().realPath)))


ROOT = _project_root()
LIVE_PORT = 18778
SHM = "/dev/shm/nhc_live"


def _req(obj, timeout=10):
    s = socket.create_connection(("127.0.0.1", LIVE_PORT), timeout=3); s.settimeout(timeout)
    try:
        d = json.dumps(obj).encode(); s.sendall(len(d).to_bytes(4, "big") + d)
        head = b""
        while len(head) < 4:
            head += s.recv(4 - len(head))
        n = int.from_bytes(head, "big"); buf = bytearray()
        while len(buf) < n:
            buf.extend(s.recv(n - len(buf)))
        return json.loads(buf)
    finally:
        s.close()


def box_class(d):
    """cube vs cuboid from the depth dimensions (independent of YOLO). Catalog: a cube has height = both top edges (40/50/60 mm); every cuboid is 60 mm long and lies on its
    30 / 40 / 20 mm side, so its height is never its long top edge. Uses the long top edge and the height only: a neighbour touching the block can clip the SHORT
    top edge (a 60 mm cube measured 60 x 46.6), but not the height. Measured with depth: top edges and heights within 1 mm of the stage truth (class_eval_c1.json)."""
    top, h = d.get("top_face_size_mm"), d.get("height_mm")
    if not top or not h:
        return None
    long_, short_ = float(top[0]), float(top[1])
    if long_ <= 0 or h <= 0:
        return None
    return "cube" if abs(h - long_) <= 0.12 * long_ and short_ >= 0.55 * long_ else "cuboid"


class BasketMap:
    """Top-view height map of the basket floor, filled from the wrist depth while the hand carries a block over the basket (Live._process calls update()).
    The basket floor is a lattice and the table shows through it, so the height of a bin is the highest point seen in it above the lattice (floor_z).
    A spot is free when the block's footprint AND the two open fingers (they go down beside the block) touch no occupied bin, no basket wall and no earlier
    placement of this run (stamps). Bins never seen count as free (the spot is verified again in flight, when the camera sees it)."""
    RES = 0.01

    def __init__(self, bmin, bmax, floor_z, rim_z, centre_margin=0.06, wall_m=0.03, occ_h=0.012, known_empty=False):
        import threading
        self.x0, self.y0 = float(bmin[0]), float(bmin[1]); R = self.RES
        self.nx = int(math.ceil((bmax[0] - bmin[0]) / R)); self.ny = int(math.ceil((bmax[1] - bmin[1]) / R))
        self.top = np.full((self.nx, self.ny), 0.0 if known_empty else np.nan, np.float32); self.seen_n = 0       # known_empty: layout() just emptied the basket, nothing to look at
        self.floor_z, self.rim_z, self.cm, self.wall_m, self.occ_h = floor_z, rim_z, centre_margin, wall_m, occ_h
        self.stamps = []; self.lock = threading.Lock(); self.fresh = 0

    def update(self, P, ok):
        R = self.RES
        sel = ok & (P[..., 0] >= self.x0) & (P[..., 0] < self.x0 + self.nx * R) & (P[..., 1] >= self.y0) & (P[..., 1] < self.y0 + self.ny * R) & (P[..., 2] < self.rim_z - 0.004)
        if int(sel.sum()) < 3000:
            return                                                          # the camera does not look into the basket in this frame
        pts = P[sel]; ix = ((pts[:, 0] - self.x0) / R).astype(int); iy = ((pts[:, 1] - self.y0) / R).astype(int)
        cur = np.full((self.nx, self.ny), -9.0, np.float32); np.maximum.at(cur, (ix, iy), pts[:, 2].astype(np.float32))
        seen = cur > -8
        with self.lock:
            self.top[seen] = cur[seen] - self.floor_z; self.fresh += 1
            if self.stamps:                                        # a remembered placement the depth now sees EMPTY (block taken out / basket emptied) is forgotten
                import cv2
                keep = []
                for x, y, r in self.stamps:
                    m_ = np.zeros((self.nx, self.ny), np.uint8); cv2.circle(m_, (int((y - self.y0) / R), int((x - self.x0) / R)), max(1, int(round(r / R))), 1, -1); m_ = m_ > 0
                    if m_.any() and seen[m_].mean() >= 0.8 and float(np.nan_to_num(self.top[m_], nan=0.0).max()) < self.occ_h:
                        continue
                    keep.append((x, y, r))
                self.stamps = keep

    def _grids(self):
        import cv2
        R = self.RES
        with self.lock:
            top = self.top.copy()
        occ = np.nan_to_num(top, nan=0.0) > self.occ_h
        for x, y, r in self.stamps:
            m_ = occ.astype(np.uint8); cv2.circle(m_, (int((y - self.y0) / R), int((x - self.x0) / R)), int(round(r / R)), 1, -1); occ = m_ > 0
        wall = np.zeros_like(occ); w = int(round(self.wall_m / R))
        wall[:w, :] = wall[-w:, :] = True; wall[:, :w] = wall[:, -w:] = True
        return occ, wall, ~np.isnan(top)

    def _mask(self, xy, r, yaw_deg, gap_m, planner):
        """Bins covered by the block (disc r) and the two open fingers at xy with closing axis yaw_deg."""
        import cv2
        R = self.RES; m = np.zeros((self.nx, self.ny), np.uint8)
        px = lambda p: (int(round((p[1] - self.y0) / R)), int(round((p[0] - self.x0) / R)))      # cv2 wants (col, row) = (y bin, x bin)
        cv2.circle(m, px(xy), max(1, int(round(r / R))), 1, -1)
        for b in planner.finger_boxes(np.asarray(xy, float), yaw_deg, 0.004, gap_m):
            cv2.fillConvexPoly(m, np.array([px(c) for c in b], np.int32), 1)
        return m > 0

    def evaluate(self, xy, r, yaw_deg, gap_m, planner, gap=0.008):
        """(free, verified): free = nothing occupied within `gap` of the block+fingers; verified = most of that area was actually seen."""
        import cv2
        occ, wall, seen = self._grids(); k = max(1, int(round(gap / self.RES)))
        bad = cv2.dilate((occ | wall).astype(np.uint8), cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * k + 1, 2 * k + 1))) > 0
        m = self._mask(xy, r, yaw_deg, gap_m, planner)
        return (not bool((bad & m).any())), bool(seen[m].mean() >= 0.7)

    def pick_spot(self, r, yaw_deg, gap_m, planner, base_xy, near=None):
        """Free spot for a block of radius r held with closing axis yaw_deg: nearest to `near` (in flight: smallest change) else to the arm base. Gaps 15 / 8 / 3 mm."""
        import cv2
        occ, wall, seen = self._grids(); R = self.RES
        xs = np.arange(self.x0 + self.cm, self.x0 + self.nx * R - self.cm, 0.02); ys = np.arange(self.y0 + self.cm, self.y0 + self.ny * R - self.cm, 0.02)
        ref = near if near is not None else base_xy
        if np.isnan(self.top).any():                              # parts never seen: prefer spots the video has seen (an unseen spot may hold something)
            cost = lambda c: math.dist(c, ref) + (0.0 if seen[self._mask(c, r, yaw_deg, gap_m, planner)].mean() >= 0.7 else 1.0)
        else:
            cost = lambda c: math.dist(c, ref)
        cands = sorted(((x, y) for x in xs for y in ys), key=cost)
        for gap in (0.015, 0.008, 0.003):
            k = max(1, int(round(gap / R)))
            bad = cv2.dilate((occ | wall).astype(np.uint8), cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * k + 1, 2 * k + 1))) > 0
            for c in cands:
                if not (bad & self._mask(c, r, yaw_deg, gap_m, planner)).any():
                    return dict(xy=[round(float(c[0]), 4), round(float(c[1]), 4)], gap_mm=round(1000 * gap, 1), n_stamps=len(self.stamps))
        return None


class Tracker:
    def __init__(self, gate_m=0.03, keep=12):
        self.tr = []; self.gate = gate_m; self.keep = keep; self.next_id = 1

    def update(self, obs, t, cam_pos):
        for o in obs:
            c = np.asarray(o["center_xyz_m"], float)
            best, bd = None, self.gate
            for tk in self.tr:
                d = float(np.linalg.norm(tk["center"][:2] - c[:2]))
                if d < bd:
                    best, bd = tk, d
            if best is not None and best["n"] >= 3 and bd > 0.012:       # established track: ignore a jump (partial view / blur)
                continue
            if best is None:
                best = dict(id=self.next_id, hist=[], votes={}, first_t=t, picked=False); self.next_id += 1; self.tr.append(best)
            best["hist"].append(c); best["hist"] = best["hist"][-self.keep:]
            best["center"] = np.median(np.array(best["hist"]), 0)
            best["votes"][o["class_name"]] = best["votes"].get(o["class_name"], 0.0) + o["conf"]
            best["class_name"] = max(best["votes"], key=best["votes"].get)
            best["conf"] = o["conf"]; best["n"] = len(best["hist"]); best["last_t"] = t; best["obs"] = o
            best["cam_dist_m"] = float(np.linalg.norm(c - np.asarray(cam_pos)))

    def snapshot(self, cls=None, min_n=1):
        return [tk for tk in self.tr if not tk["picked"] and tk["n"] >= min_n and (cls is None or tk["class_name"] == cls)]


class Live:
    def __init__(self, oc):
        self.oc = oc; self.geo = None; self.tracker = Tracker(); self.running = False
        self.n_frames = 0; self.n_proc = 0; self.lat_ms = []; self.fps_log = []
        self.pool = concurrent.futures.ThreadPoolExecutor(1); self.job = None
        self.cfg = oc["CFG"]; self.log = []; self.sim_f = 0; self.errors = []
        self.blobs_latest = None; self.cam_hist = []; self.view_h = (0.40, 0.62); self.max_speed = 0.05; self.min_conf = 0.5; self.rescue_fg = 0.6; self.basket_map = None; self.basket_on = False; self.basket_gate = None; self.obs_log = []; self.paused = False; self.min_cam_h = 0.40; self.rec_dir = None; self.rec_n = 0; self.hud = ""

    async def start(self):
        import omni.kit.app, omni.replicator.core as rep, omni.usd
        os.makedirs(SHM, exist_ok=True)
        try:
            r = _req({"cmd": "ping"}, 3)
            if r.get("src_mtime") != os.path.getmtime(os.path.join(ROOT, "lab", "yolo_live.py")):
                _req({"cmd": "quit"}, 3); raise RuntimeError("restart")
        except Exception:
            py = os.path.join(ROOT, self.cfg["yolo_python"]) if not os.path.isabs(self.cfg["yolo_python"]) else self.cfg["yolo_python"]
            subprocess.Popen([py, os.path.join(ROOT, "lab", "yolo_live.py")], cwd=ROOT, stdout=open(os.path.join(ROOT, "outputs", "lab", "yolo_live.log"), "a"),
                             stderr=subprocess.STDOUT, start_new_session=True)
            for _ in range(60):
                await asyncio.sleep(1)
                try:
                    _req({"cmd": "ping"}, 3); break
                except Exception:
                    pass
        gp = os.path.join(ROOT, "perception", "geometry.py"); g = {"__file__": gp}
        exec(compile(open(gp, encoding="utf-8").read(), gp, "exec"), g); self.geo = type("G", (), g)
        self.ns = self.oc["_load_arm_stack"](); await self.ns["_setup"]()
        reg = self.ns["_registry"](); self.rp = reg["render_product"]; self.rgb_ann = reg["rgb"]; self.depth_ann = reg["depth"]
        self.cp = rep.AnnotatorRegistry.get_annotator("camera_params"); self.cp.attach(self.rp)
        self.K = self.ns["camera_intrinsics"](omni.usd.get_context().get_stage(), self.ns["CAMERA_PATHS"]["left"])
        self.ns["_enable"](self.rp, True)
        for _ in range(8): await omni.kit.app.get_app().next_update_async()
        self.running = True; self.prev_pos = None; self.t0 = time.time()
        self.task = asyncio.ensure_future(self._loop())

    async def stop(self):
        self.running = False
        try:
            await self.task
        except Exception:
            pass
        self.ns["_enable"](self.rp, False)

    # ------------------------------------------------------------ main-thread loop: grab frames, hand to worker
    async def _loop(self):
        import omni.kit.app
        app = omni.kit.app.get_app()
        while self.running:
            await app.next_update_async(); self.sim_f += 1
            try:
                self._poll_job()
                d = self.cp.get_data(); v = np.asarray(d["cameraViewTransform"], float).reshape(4, 4)
                inv = np.linalg.inv(v); pos = inv[3, :3]
                if self.prev_pos is None or not np.allclose(pos, self.prev_pos, atol=1e-7):
                    self.n_frames += 1                          # camera moved = a new rendered frame (a still camera gives the same pose)
                self.prev_pos = pos
                if self.job is not None:
                    continue                                    # worker busy: drop this frame (always process the newest)
                r = np.asarray(self.rgb_ann.get_data()); dp = np.asarray(self.depth_ann.get_data(), dtype=np.float32)
                if r.ndim != 3 or dp.ndim != 2 or r[:, :, :3].mean() < 1.0:
                    continue
                cam = dict(position_m=pos.tolist(), right=inv[0, :3].tolist(), up=inv[1, :3].tolist(), forward=(-inv[2, :3]).tolist())
                if self.basket_on and self.basket_map is not None and self.sim_f % 2 == 0 and (self.basket_gate is None or self.basket_gate()):
                    try:                                         # basket height map from every 2nd depth frame while the hand is high near the basket (no YOLO needed, the arm keeps moving)
                        self.basket_map.update(self.geo.world_points(dp, cam, self.K, self.cfg.get("pixel_center_offset", 0.5)), np.isfinite(dp) & (dp > 0) & (dp < 5))
                    except Exception as e_:
                        self.errors.append("basket_map: " + repr(e_))
                self.cam_hist.append((self.sim_f, pos.copy())); self.cam_hist = self.cam_hist[-40:]
                older = [(f, p) for f, p in self.cam_hist if self.sim_f - f >= 6]
                cam["speed"] = float(np.linalg.norm(pos - older[-1][1]) / ((self.sim_f - older[-1][0]) / 60.0)) if older else 9.9
                self.job = (self.pool.submit(self._process, r[:, :, :3].astype(np.uint8).copy(), dp.copy(), cam), self.sim_f / 60.0, time.time(), cam)
            except Exception as e:
                self.errors.append(repr(e))

    def _poll_job(self):
        if self.job is None or not self.job[0].done():
            return
        fut, t_sim, t_wall, cam = self.job; self.job = None
        try:
            obs, info = fut.result()
        except Exception as e:
            self.errors.append("worker: " + repr(e)); return
        self.n_proc += 1; self.lat_ms.append((time.time() - t_wall) * 1000)
        for o in obs:
            self.obs_log.append((round(t_sim, 3), o["class_name"], round(o["conf"], 3), o["center_xyz_m"], round(cam["position_m"][2], 3), o.get("mask_pixels"), bool(self.paused), round(cam.get("speed", -1), 3),
                                 o.get("yolo_class"), o.get("top_face_size_mm"), o.get("height_mm"), o.get("fg_frac")))
        h_cam = cam["position_m"][2] - self.cfg["table_z_m"]
        good = (not self.paused and self.view_h[0] <= h_cam <= self.view_h[1]    # only views like the training data (0.43-0.56 m above the table)
                and cam.get("speed", 9.9) <= self.max_speed)                     # and a camera that is (nearly) still: a moving one blurs the path-traced image
        if good:
            self.tracker.update([o for o in obs if o["conf"] >= self.min_conf or o.get("fg_frac", 0.0) >= self.rescue_fg], t_sim, cam["position_m"])
        self.last_info = info; self.last_cam = cam
        self.blobs_latest = (t_sim, info.get("blobs", []), cam)

    # ------------------------------------------------------------ worker thread
    def _process(self, rgb, depth, cam):
        import cv2
        t = time.time(); geo = self.geo; cfg = self.cfg
        png = os.path.join(SHM, "f.png"); cv2.imwrite(png, rgb[:, :, ::-1], [cv2.IMWRITE_PNG_COMPRESSION, 1])
        mpath = os.path.join(SHM, "m.npy")
        rep = _req({"cmd": "detect", "rgb_path": png, "masks_path": mpath, "conf": cfg["candidate_conf"]})
        if not rep.get("ok"):
            raise RuntimeError(rep.get("error"))
        t_yolo = time.time() - t
        masks_all = np.load(mpath).astype(bool)
        P = geo.world_points(depth, cam, self.K, cfg.get("pixel_center_offset", 0.5))
        mg = cfg.get("detection_region_margin_m", 0.0)
        wx = [cfg["workspace_x_m"][0] - mg, cfg["workspace_x_m"][1] + mg]; wy = [cfg["workspace_y_m"][0] - mg, cfg["workspace_y_m"][1] + mg]
        fg = geo.foreground(depth, P, cfg["table_z_m"], wx, wy)
        raw = [dict(cls=d["class_name"], conf=d["conf"], px=d["mask_pixels"], bbox=d["bbox_xyxy"]) for d in rep["detections"]]
        dets, masks = geo.select_detections(rep["detections"], masks_all, fg, cfg["conf"])
        kept_boxes = {tuple(d["bbox_xyxy"]) for d in dets}
        for r_ in raw:
            r_["reason"] = "kept" if tuple(r_["bbox"]) in kept_boxes else "dropped: conf<%.2f and not rescued by depth" % cfg["conf"]
        union = masks.any(0) if len(masks) else None
        out = []; H, W = depth.shape
        for i, d in enumerate(dets):
            m = masks[i]
            ys, xs = np.nonzero(m)
            if len(xs) == 0 or xs.min() <= 1 or ys.min() <= 1 or xs.max() >= W - 2 or ys.max() >= H - 2:
                self._mark_raw(raw, d, "dropped: mask touches the image border")
                continue                                        # clipped by the image border -> centre would be biased
            others = (union & ~m) if union is not None else None
            d["yolo_class"] = d["class_name"]
            d.update(geo.estimate(m, depth, P, cam, d["class_name"], cfg["table_z_m"], exclude=others))
            if d.get("center_xyz_m") is None:
                self._mark_raw(raw, d, "dropped: too few valid depth points")
                continue
            d["fg_frac"] = round(float((m & fg).sum()) / max(1, int(m.sum())), 3)        # share of the mask that stands on the table in depth (a real object, whatever YOLO's confidence)
            if d["yolo_class"] in ("cube", "cuboid"):
                sc = box_class(d)
                if sc:
                    d["class_name"] = sc
            c = d["center_xyz_m"]
            if wx[0] <= c[0] <= wx[1] and wy[0] <= c[1] <= wy[1]:
                out.append(d)
            else:
                self._mark_raw(raw, d, "dropped: centre outside the table workspace")
        self.last_raw = raw
        blobs = []                                            # class-free depth blobs standing on the table (usable close to the table where YOLO is not)
        try:
            m8 = cv2.morphologyEx(fg.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            nb, lab, stats, _ = cv2.connectedComponentsWithStats(m8, connectivity=8)
            for k_ in range(1, nb):
                if stats[k_, cv2.CC_STAT_AREA] < 80:
                    continue
                pts = P[lab == k_]
                blobs.append(dict(xy=[round(float(pts[:, 0].mean()), 4), round(float(pts[:, 1].mean()), 4)], top_z=round(float(np.percentile(pts[:, 2], 98)), 4),
                                  px=int(stats[k_, cv2.CC_STAT_AREA]), extent_mm=[round(1000 * float(np.ptp(pts[:, 0])), 1), round(1000 * float(np.ptp(pts[:, 1])), 1)]))
        except Exception as e_:
            self.errors.append("blobs: " + repr(e_))
        if self.rec_dir:
            try:
                self._record(rgb, masks, dets, out, cam)
            except Exception as e:
                self.errors.append("rec: " + repr(e))
        return out, dict(yolo_s=round(t_yolo, 3), total_s=round(time.time() - t, 3), n_det=len(out), predict_ms=rep.get("predict_ms"), blobs=blobs)

    @staticmethod
    def _mark_raw(raw, d, why):
        for r_ in raw:
            if tuple(r_["bbox"]) == tuple(d["bbox_xyxy"]):
                r_["reason"] = why

    def _record(self, rgb, masks, dets, out, cam):
        """Annotated wrist frame: outline + #id on each object, and ONE list in the top-left corner
        (id, class, confidence, world x y z, distance from the camera) that changes as the arm moves."""
        import cv2
        img = np.ascontiguousarray(rgb[:, :, ::-1]); keep = {id(d) for d in out}
        tracks = list(self.tracker.tr); rows = []
        for i, d in enumerate(dets):
            if i >= len(masks) or id(d) not in keep:
                continue
            col = [(66, 135, 245), (60, 60, 235), (80, 220, 90), (40, 200, 245), (200, 70, 230)][d.get("class_id", 0) % 5]
            cs, _ = cv2.findContours(masks[i].astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(img, cs, -1, col, 2)
            c = d["center_xyz_m"]
            near = [t for t in tracks if np.linalg.norm(t["center"][:2] - np.asarray(c[:2])) < 0.03]
            tid = near[0]["id"] if near else 0
            x0, y0 = int(d["bbox_xyxy"][0]), int(max(12, d["bbox_xyxy"][1]))
            cv2.putText(img, f"#{tid}", (x0, y0 - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(img, f"#{tid}", (x0, y0 - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
            rows.append((tid, d["class_name"], d["conf"], c, d["distance_from_camera_m"], col))
        rows.sort(key=lambda r: r[0])
        lines = [(self.hud or "video", (0, 255, 255))] + \
                [(f"#{t:<2d} {n:<9s} {cf:4.2f}  x{c[0]:7.3f} y{c[1]:7.3f} z{c[2]:6.3f}  d{dd:5.3f}m", col) for t, n, cf, c, dd, col in rows]
        h = 6 + 16 * len(lines)
        ov = img.copy(); cv2.rectangle(ov, (0, 0), (420, h), (0, 0, 0), -1); img = cv2.addWeighted(ov, 0.55, img, 0.45, 0)
        for k, (txt, col) in enumerate(lines):
            cv2.putText(img, txt, (5, 16 + 16 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1, cv2.LINE_AA)
        self.rec_n += 1
        cv2.imwrite(os.path.join(self.rec_dir, f"f{self.rec_n:05d}.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 88])

    def blob_near(self, xy, radius=0.04, max_age_s=0.25):
        """Newest class-free blob within `radius` of xy: (blob, age_s), or (None, age_s) / (None, None)."""
        if not self.blobs_latest:
            return None, None
        t, bl, cam = self.blobs_latest; age = self.sim_f / 60.0 - t
        if age > max_age_s:
            return None, age
        best = None
        for b in bl:
            d = float(np.hypot(b["xy"][0] - xy[0], b["xy"][1] - xy[1]))
            if d <= radius and (best is None or d < best[0]):
                best = (d, b)
        return (best[1] if best else None), age

    def tracks(self, cls=None, min_n=1):
        return self.tracker.snapshot(cls, min_n)


async def selftest(tag="t1", seconds=6.0):
    """Arm moves along a sweep while video mode runs; report rates, latency, and track stability vs stage truth."""
    import omni.kit.app
    app = omni.kit.app.get_app(); out = {}
    try:
        oc = builtins._lab_oc; pose = oc["_pose"]
        live = Live(oc); await live.start()
        # go to view A (existing config) and let the arm settle while the stream runs
        va = oc["_view_poses"]()[0]
        p0, q0 = pose("/World/IKTarget")
        n = 90
        for i in range(1, n + 1):
            u = i / n; s = u * u * (3 - 2 * u)
            pose("/World/IKTarget", (np.asarray(p0) + s * (np.asarray(va["position_m"]) - np.asarray(p0))).tolist(), oc["_slerp"](q0, va["quaternion_wxyz"], s))
            await app.next_update_async()
        t0 = time.time(); f0 = live.sim_f; nf0, np0 = live.n_frames, live.n_proc
        for i in range(int(seconds * 60)):
            await app.next_update_async()
        dt = time.time() - t0
        tk = live.tracks()
        out = dict(wall_s=round(dt, 2), app_fps=round((live.sim_f - f0) / dt, 1), cam_frames_per_s=round((live.n_frames - nf0) / dt, 1),
                   processed_per_s=round((live.n_proc - np0) / dt, 1), latency_ms_mean=round(float(np.mean(live.lat_ms)), 0) if live.lat_ms else None,
                   latency_ms_p95=round(float(np.percentile(live.lat_ms, 95)), 0) if live.lat_ms else None, last_info=getattr(live, "last_info", None),
                   errors=live.errors[:5],
                   tracks=[dict(id=t["id"], cls=t["class_name"], conf=round(t["conf"], 2), n=t["n"], center=[round(float(v), 4) for v in t["center"]],
                                spread_mm=round(1000 * float(np.max(np.std(np.array(t["hist"]), 0))), 2)) for t in tk])
        await live.stop()
    except Exception:
        import traceback; out["error"] = traceback.format_exc()
    os.makedirs(os.path.join(ROOT, "outputs", "lab", "live"), exist_ok=True)
    json.dump(out, open(os.path.join(ROOT, "outputs", "lab", "live", f"selftest_{tag}.json"), "w"), indent=1, default=str)


def start_selftest(tag="t1", seconds=6.0):
    builtins._lab_self_task = asyncio.ensure_future(selftest(tag, seconds))
