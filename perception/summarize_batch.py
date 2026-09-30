"""Summarize a batch: python perception/summarize_batch.py [outputs/batch/<ts>]  -> prints + writes report.json"""
import json, os, sys, glob, statistics as st
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "perception"))
from eval_centers import evaluate

bdir = sys.argv[1] if len(sys.argv) > 1 else sorted(glob.glob(os.path.join(ROOT, "outputs/batch/2*")))[-1]
runs = [r["run"] for r in json.load(open(os.path.join(bdir, "summary.json")))["runs"] if r.get("run")]
rows, cls, ngt, fp, missed = [], [], 0, 0, []
for r in runs:
    e = evaluate(os.path.join(ROOT, "outputs/oneclick", r))
    rows += [dict(x, run=r) for x in e["rows"]]; cls += [dict(c, run=r) for c in e["classification"]]
    ngt += e["summary"]["n_gt_objects"]; fp += e["summary"]["false_positives"]; missed += [(r, m) for m in e["summary"]["missed_in_first_view"]]
rep = dict(batch=os.path.basename(bdir), layouts=len(runs), gt_objects=ngt, false_positives_view1=fp, missed_view1=missed, views={})
for v in sorted({x["view"] for x in rows}):
    rr = [x for x in rows if x["view"] == v]
    rel = [x["rel_err_pct"] for x in rr]; mm = [x["err_mm"] for x in rr]
    rep["views"][v] = dict(n=len(rr), pass_1pct=sum(x < 1 for x in rel), max_rel_pct=max(rel), median_rel_pct=round(st.median(rel), 3),
                           max_err_mm=max(mm), median_err_mm=round(st.median(mm), 2),
                           max_abs_height_err_pct=max(abs(x["height_err_pct"]) for x in rr),
                           worst=sorted(rr, key=lambda x: -x["rel_err_pct"])[:5])
# per class, per axis mean |error| (mm) for each view
rep["axis_by_class"] = {}
for v in rep["views"]:
    for c in ("cube", "cuboid", "cylinder", "sphere"):
        rr = [x for x in rows if x["view"] == v and (x["gt"].split("_")[0] if x["gt"] != "object" else "cube") == c]
        if rr:
            rep["axis_by_class"][f"{v}/{c}"] = dict(n=len(rr), **{f"mean_abs_{a}": round(sum(abs(x["err_xyz_mm"][i]) for x in rr) / len(rr), 3) for i, a in enumerate("xyz")},
                                                    **{f"max_abs_{a}": max(abs(x["err_xyz_mm"][i]) for x in rr) for i, a in enumerate("xyz")})
views = sorted({k for c in cls for k in c["per_view"]})
both = [c for c in cls if len(c["per_view"]) == len(views)]
rep["classification"] = dict(n=len(cls), merged_correct=sum(c["merged"]["ok"] for c in cls),
                             **{f"{v}_correct": sum(c["per_view"].get(v, {}).get("ok", False) for c in cls) for v in views},
                             **{f"{v}_yolo_only_correct": sum(c["per_view"].get(v, {}).get("yolo_ok", False) for c in cls) for v in views},
                             matched_in_all_views=len(both))
if len(views) == 2:
    a, b = views[0], views[1]
    d = [c["per_view"][b]["conf"] - c["per_view"][a]["conf"] for c in both]
    rep["confidence_second_view"] = dict(view_pair=[a, b], higher=sum(x > 0 for x in d), lower=sum(x < 0 for x in d),
                                         median_change=round(st.median(d), 3),
                                         low_conf_first=[dict(gt=c["gt"], **{v: (c["per_view"][v]["class_name"], c["per_view"][v]["conf"]) for v in views},
                                                              merged=c["merged"]["class_name"], true=c["true_class"]) for c in both if c["per_view"][a]["conf"] < 0.7])
    rep["classification"]["wrong"] = [dict(gt=c["gt"], run=c["run"], **{v: (c["per_view"].get(v, {}).get("class_name"), c["per_view"].get(v, {}).get("conf")) for v in views},
                                           merged=c["merged"]["class_name"]) for c in cls if not (c["merged"]["ok"] and all(p["ok"] for p in c["per_view"].values()))]
json.dump(rep, open(os.path.join(bdir, "report.json"), "w"), indent=1)
print(json.dumps({k: v for k, v in rep.items()}, indent=1)[:6000])
