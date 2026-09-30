"""Accuracy batch (run in the Isaac Sim Script Editor): N random layouts -> one-click each -> compare with stage truth.

Edit SEEDS / N_BLOCKS below, Ctrl+Enter. Progress + summary: outputs/batch/<timestamp>/summary.json
Each layout is a normal one-click run (outputs/oneclick/<ts>/ with eval.json). Test blocks live in the
session layer only; the scene file is never modified.
"""
import asyncio, builtins, datetime, json, os, sys, traceback

import omni.usd
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(omni.usd.get_context().get_stage().GetRootLayer().realPath)))
SEEDS = [11, 12, 13, 14, 15, 16, 17, 18]
N_BLOCKS = 10


def _load(name, extra=None):
    g = dict(NO_AUTORUN=True, **(extra or {}))
    p = os.path.join(ROOT, "perception", name)
    g["__file__"] = p
    exec(compile(open(p, encoding="utf-8").read(), p, "exec"), g)
    return g


async def _batch(seeds, n_blocks):
    out = os.path.join(ROOT, "outputs", "batch", datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
    os.makedirs(out, exist_ok=True)
    runs = []
    for seed in seeds:
        try:
            ptb = _load("place_test_blocks.py")
            ptb["place_test_blocks"](n=n_blocks, seed=seed, catalog=ptb["CATALOG_VISION"], physics=False)
            oc = _load("one_click.py"); oc["CFG"]["open_browser"] = False
            oc["CFG"]["second_view_policy"] = "always"   # evaluate view B on every layout; the trigger reasons are still recorded
            import omni.kit.app
            for _ in range(30):
                await omni.kit.app.get_app().next_update_async()
            doc = await oc["_run"]()
            ev = _load("eval_centers.py")["evaluate"](os.path.join(ROOT, "outputs", "oneclick", doc["run_id"]))
            runs.append(dict(seed=seed, run=doc["run_id"], summary=ev["summary"]))
        except Exception:
            runs.append(dict(seed=seed, error=traceback.format_exc()))
        json.dump(dict(state="running", runs=runs), open(os.path.join(out, "summary.json"), "w"), indent=1)
    json.dump(dict(state="done", runs=runs), open(os.path.join(out, "summary.json"), "w"), indent=1)


if not globals().get("NO_AUTORUN"):
    builtins._batch_task = asyncio.ensure_future(_batch(SEEDS, N_BLOCKS))
    print("batch scheduled")
