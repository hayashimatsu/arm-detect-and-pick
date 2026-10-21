"""lab/refine_report.py -- table for feedback-4 #2 from outputs/lab/live/refine_<tag>.json:  python3 lab/refine_report.py static [moved] -> refine_report.html"""
import json, os, sys, statistics as st
D = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "outputs", "lab", "live")


def table(tag):
    d = json.load(open(os.path.join(D, f"refine_{tag}.json"))); rows = [r for r in d["rows"] if "grasp_err_mm" in r and r["grasp_err_mm"] is not None]
    out = [f"<h3>refine_{tag}.json (done={d['done']}, runs={len(d['rows'])}, shift={d['rows'][0].get('shift') if d['rows'] else None})</h3><table><tr><th>class</th><th>mode</th><th>n</th><th>hand-to-block distance at close, mm (mean / median / max)</th><th>first-try success</th><th>leg start to grasp, s (mean / max)</th><th>leg start to release, s (mean / max)</th><th>errors</th></tr>"]
    summ = {}
    for cls in ("sphere", "cube", "cuboid", "cylinder"):
        for mode in (False, "change", "estimate"):
            rr = [r for r in rows if r["cls"] == cls and r["mode"] == mode]
            if not rr:
                continue
            e = [r["grasp_err_mm"] for r in rr]; ft = sum(1 for r in rr if r.get("first_try")); lg = [r["leg_to_grasp_s"] for r in rr if r.get("leg_to_grasp_s") is not None]; lr = [r["leg_to_release_s"] for r in rr if r.get("leg_to_release_s") is not None]
            err = sum(1 for r in d["rows"] if r["cls"] == cls and r["mode"] == mode and r.get("error"))
            summ[(cls, mode)] = dict(n=len(rr), mean=st.mean(e), median=st.median(e), mx=max(e), ft=ft, leg=st.mean(lg) if lg else None, rel=st.mean(lr) if lr else None)
            out.append(f"<tr><td>{cls}</td><td>{'off' if mode is False else mode}</td><td>{len(rr)}</td><td>{st.mean(e):.1f} / {st.median(e):.1f} / {max(e):.1f}</td><td>{ft}/{len(rr)}</td>"
                       f"<td>{st.mean(lg):.2f} / {max(lg):.2f}</td><td>{st.mean(lr):.2f} / {max(lr):.2f}</td><td>{err}</td></tr>")
    out.append("</table>")
    return "\n".join(out), summ


if __name__ == "__main__":
    html = ["<!doctype html><meta charset=utf-8><title>visual final correction</title><style>body{font:13px system-ui;max-width:1100px;margin:20px auto}td,th{border-bottom:1px solid #ddd;padding:4px 6px;text-align:left}</style><h2>Visual final correction (off / change / estimate)</h2>"]
    for tag in sys.argv[1:] or ["static"]:
        h, s = table(tag); html.append(h)
        for cls in ("sphere", "cube", "cuboid", "cylinder"):
            base = s.get((cls, False))
            for mode in ("change", "estimate"):
                m = s.get((cls, mode))
                if base and m:
                    print(tag, cls, mode, "vs off: err mean %.1f -> %.1f mm, first-try %d/%d -> %d/%d, leg->release %.2f -> %.2f s" % (base["mean"], m["mean"], base["ft"], base["n"], m["ft"], m["n"], base["rel"], m["rel"]))
    open(os.path.join(D, "refine_report.html"), "w").write("\n".join(html))
