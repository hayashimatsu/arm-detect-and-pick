"""Compare estimated world-frame object centers with the stage ground truth of a run.

Usage: python perception/eval_centers.py [run_dir ...]   (default: latest outputs/oneclick run)
Reads result.json + gt_eval_only.json, writes eval.json next to them and prints a table.

Error metric (the 1% requirement):
  rel_err = |estimated center - true center| / distance(camera, true center)
  -> 1% at ~0.55 m camera distance is ~5.5 mm.
Also reported: absolute 3D error and per-axis error in mm.
"""
import json, math, os, sys, glob

TABLE_Z = 0.5952
MATCH_M = 0.05   # an estimate farther than this from every true object = false positive
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def gt_class(name):
    return "cube" if name == "object" else name.split("_")[0]


def evaluate(run):
    res = json.load(open(os.path.join(run, "result.json")))
    gt = json.load(open(os.path.join(run, "gt_eval_only.json")))
    views = {v["name"]: v for v in res["views"]}
    rows, false_pos = [], []
    for o in res["objects"]:
        items = list(o["per_view"].items())
        if o.get("center_from_view"):
            items.append(("final", dict(class_name=o["class_name"], conf=o["conf"], center_xyz_m=o.get("center_xyz_m"),
                                        cam_view=o["center_from_view"])))
        for vname, pv in items:
            c = pv.get("center_xyz_m")
            if c is None:
                continue
            name, g = min(gt.items(), key=lambda kv: math.dist(kv[1]["center_xyz_m"][:2], c[:2]))
            if math.dist(g["center_xyz_m"][:2], c[:2]) > MATCH_M:
                false_pos.append(dict(id=o["id"], view=vname, class_pred=pv["class_name"], conf=pv["conf"], center_xyz_m=c))
                continue
            gc = g["center_xyz_m"]; cam = views[pv.get("cam_view", vname)]["camera"]["position_m"]
            err = [1000 * (c[i] - gc[i]) for i in range(3)]
            e3 = math.sqrt(sum(x * x for x in err))
            true_h = g["size_mm"][2]; est_h = 2 * (c[2] - TABLE_Z) * 1000
            rows.append(dict(id=o["id"], view=vname, gt=name, class_pred=pv["class_name"], conf=pv["conf"],
                             height_err_pct=round(100 * (est_h - true_h) / true_h, 3),
                             err_xyz_mm=[round(x, 2) for x in err], err_mm=round(e3, 2),
                             rel_err_pct=round(100 * e3 / 1000 / math.dist(cam, gc), 3)))
    # classification: per view and the merged (max-confidence) answer
    cls_rows = []
    for o in res["objects"]:
        c = o.get("center_xyz_m")
        if c is None:
            continue
        name = min(gt, key=lambda k: math.dist(gt[k]["center_xyz_m"][:2], c[:2]))
        if math.dist(gt[name]["center_xyz_m"][:2], c[:2]) > MATCH_M:
            continue
        t = gt_class(name)
        cls_rows.append(dict(id=o["id"], gt=name, true_class=t,
                             merged=dict(class_name=o["class_name"], conf=o["conf"], ok=o["class_name"] == t),
                             per_view={k: dict(class_name=v["class_name"], conf=v["conf"], ok=v["class_name"] == t,
                                               yolo_class=v.get("yolo_class") or v["class_name"],
                                               yolo_ok=(v.get("yolo_class") or v["class_name"]) == t)
                                       for k, v in o["per_view"].items()}))
    summary = {}
    for vname in list(views) + ["final"]:
        r = [x for x in rows if x["view"] == vname]
        if r:
            summary[vname] = dict(n=len(r), max_rel_pct=max(x["rel_err_pct"] for x in r),
                                  mean_rel_pct=round(sum(x["rel_err_pct"] for x in r) / len(r), 3),
                                  max_err_mm=max(x["err_mm"] for x in r),
                                  max_abs_height_err_pct=max(abs(x["height_err_pct"]) for x in r),
                                  pass_1pct=sum(x["rel_err_pct"] < 1 for x in r))
    names = [v for v in views]
    summary["classification"] = dict(n=len(cls_rows), merged_correct=sum(r["merged"]["ok"] for r in cls_rows),
                                     **{f"{v}_correct": sum(r["per_view"].get(v, {}).get("ok", False) for r in cls_rows) for v in names},
                                     **{f"{v}_yolo_only_correct": sum(r["per_view"].get(v, {}).get("yolo_ok", False) for r in cls_rows) for v in names})
    summary["n_gt_objects"] = len(gt)
    summary["false_positives"] = len({(f["id"]) for f in false_pos if f["view"] == res["views"][0]["name"]})
    found = {r["gt"] for r in rows if r["view"] == res["views"][0]["name"]}
    summary["missed_in_first_view"] = sorted(set(gt) - found)
    out = dict(run=os.path.basename(run), summary=summary, rows=rows, classification=cls_rows, false_positives=false_pos)
    json.dump(out, open(os.path.join(run, "eval.json"), "w"), indent=1)
    return out


if __name__ == "__main__":
    runs = sys.argv[1:] or [sorted(glob.glob(os.path.join(ROOT, "outputs/oneclick/2*")))[-1]]
    for run in runs:
        out = evaluate(run)
        print(out["run"], json.dumps(out["summary"]))
        for x in out["rows"]:
            print(f'{x["id"]:>2} {x["view"]:6} {x["gt"]:18} {x["class_pred"]:8} {x["conf"]:.2f} err={x["err_mm"]:6.2f}mm rel={x["rel_err_pct"]:.2f}% xyz={x["err_xyz_mm"]}')
