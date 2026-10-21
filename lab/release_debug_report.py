"""lab/release_debug_report.py -- builds outputs/lab/live/release_debug_report.html (table + SVG chart) from release_debug_<tag>_<seed>.json."""
import json, os, sys
D = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "outputs", "lab", "live")
RUNS = [("r1", "cube", "straight up after release", "none (single block)"), ("r2", "cube", "up with rounded corner, as t5", "none (single block)"),
        ("r3", "cube", "normal flow: next pick's pre-shape + leg", "pre-shape command right after release")]


def summarize(tag, seed, idx=None):
    p = os.path.join(D, f"release_debug_{tag}_{seed}.json" if idx is None else f"release_debug_{tag}_{seed}_{idx}.json")
    if not os.path.exists(p):
        return None
    rows = json.load(open(p))["rows"]; z0 = rows[0]["block"][2]
    carried = max(1000 * (r["block"][2] - z0) for r in rows) > 8
    grip = sorted({c for r in rows for c in r["contacts"] if "HeavyDutyPacking" not in c})
    first_grip = next((r["f"] for r in rows if any("HeavyDutyPacking" not in c for c in r["contacts"])), None)
    cmd_change = next((r["f"] for r in rows if r["phase"] == "next" and r["cmd"] > 1.0), None)
    return dict(seed=seed, carried=carried, gripper_contacts=grip, first_gripper_contact_frame=first_grip, preshape_cmd_frame=cmd_change,
                max_block_dz_mm=round(max(1000 * (r["block"][2] - z0) for r in rows), 1), frames=len(rows), rows=rows)


def svg(rows):
    W, H, M = 760, 260, 36; f0 = rows[0]["f"]; n = max(1, rows[-1]["f"] - f0); z0 = rows[0]["block"][2]
    X = lambda f: M + (f - f0) / n * (W - 2 * M)
    ser = [("finger angle (deg)", [r["finger"] for r in rows], "#d62728", 0, 30), ("finger command (deg)", [r["cmd"] for r in rows], "#ff9896", 0, 30),
           ("hand height above release (mm)", [1000 * (r["tcp"][2] - rows[0]["tcp"][2]) for r in rows], "#1f77b4", 0, 140),
           ("block height change (mm)", [1000 * (r["block"][2] - z0) for r in rows], "#2ca02c", -10, 140)]
    out = [f'<svg viewBox="0 0 {W} {H}" width="100%" style="background:#fff;border:1px solid #ccc">']
    for name, ys, col, lo, hi in ser:
        pts = " ".join(f"{X(r['f']):.1f},{H - M - (y - lo) / (hi - lo) * (H - 2 * M):.1f}" for r, y in zip(rows, ys))
        out.append(f'<polyline fill="none" stroke="{col}" stroke-width="2" points="{pts}"/>')
    for r in rows:
        if any("HeavyDutyPacking" not in c for c in r["contacts"]):
            out.append(f'<rect x="{X(r["f"]) - 2:.1f}" y="{M - 16}" width="5" height="12" fill="#9467bd"/>')
    for k, (name, ys, col, lo, hi) in enumerate(ser):
        out.append(f'<text x="{M}" y="{H - 4 - 0}" font-size="11" fill="{col}" transform="translate({k * 175},0)">{name}</text>')
    out.append(f'<text x="{M}" y="12" font-size="11" fill="#9467bd">purple tick = a gripper part touches the block (PhysX contact report)</text></svg>')
    return "".join(out)


html = ["<!doctype html><meta charset=utf-8><title>release debug</title><style>body{font:14px system-ui;max-width:900px;margin:24px auto;padding:0 16px}td,th{border-bottom:1px solid #ddd;padding:6px;text-align:left}</style>",
        "<h2>feedback-2 #1: what happens to the block after release (single block, per frame)</h2>",
        "<table><tr><th>run</th><th>leave / next move</th><th>seed</th><th>block carried?</th><th>max block rise (mm)</th><th>gripper parts that touched the block</th><th>first touch frame</th><th>pre-shape command frame</th></tr>"]
chart = None
for tag, tgt, desc, _ in RUNS:
    for sd in (33, 36, 41):
        s = summarize(tag, sd)
        if not s:
            continue
        html.append(f"<tr><td>{tag}</td><td>{desc}</td><td>{sd}</td><td>{'YES' if s['carried'] else 'no'}</td><td>{s['max_block_dz_mm']}</td><td>{', '.join(s['gripper_contacts']) or '-'}</td><td>{s['first_gripper_contact_frame']}</td><td>{s['preshape_cmd_frame']}</td></tr>")
for tag, desc in (("r4", "BEFORE fix: normal flow, pre-shape at the start of the next leg"), ("r5", "AFTER fix: pre-shape only when the hand is at travel height")):
    for i in range(5):
        s = summarize(tag, 33, i)
        if not s:
            continue
        html.append(f"<tr><td>{tag}</td><td>{desc}</td><td>33 #{i}</td><td>{'YES' if s['carried'] else 'no'}</td><td>{s['max_block_dz_mm']}</td><td>{', '.join(s['gripper_contacts']) or '-'}</td><td>{s['first_gripper_contact_frame']}</td><td>{s['preshape_cmd_frame']}</td></tr>")
        if tag == "r4" and s["carried"] and chart is None:
            chart = svg(s["rows"])
html.append("</table>")
if chart:
    html += ["<h3>r4 (before the fix), a trial where the block was carried: finger command, finger angle, hand and block height</h3>", chart]
open(os.path.join(D, "release_debug_report.html"), "w").write("\n".join(html))
print("ok")
