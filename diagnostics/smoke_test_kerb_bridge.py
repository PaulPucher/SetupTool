# Headless smoke test for the kerb-gap bridge in the trace windows, on one
# CACHED outing (data/analysis_cache sidecar, read directly and read-only:
# nothing is written to the DB or the cache). Drives CornerTraceDialog and
# LapTraceDialog with the bridge on and off and checks that bridges appear
# only when on, and that the analysis arrays are byte-identical before and
# after rendering. Run: python -m diagnostics.smoke_test_kerb_bridge

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import gzip
import hashlib
import pickle

import numpy as np
from PyQt6.QtWidgets import QApplication

app = QApplication([])

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, summarise_corners
from diagnostics.inspect_frame_stage2_parity import classify_fn
from ui.views.corner_trace_dialog import CornerTraceDialog, LapTraceDialog

SIDECAR = "data/analysis_cache/outing_4.pkl.gz"

with gzip.open(SIDECAR, "rb") as f:
    identity = pickle.load(f)
    payload = pickle.load(f)
print(f"cached outing: {identity['csv_path']}, sideslip {identity['sideslip_source']}, lap_filter {identity['lap_filter']}")
data = parse_csv(identity["csv_path"])
state, cs, stab = payload["state"], payload["cs"], payload["stab"]
summaries = summarise_corners(payload["corners"], cs, stab, state, fz=payload.get("fz"), ls=payload.get("ls"))
result = dict(payload, summaries=summaries, lap_filter=None)


def digest():
    h = hashlib.md5()
    for a in (cs["CS_ratio_f"], cs["CS_ratio_r"], stab["stability_observed_Nm_per_deg"], state["kerb_mask"]):
        h.update(np.ascontiguousarray(a).tobytes())
    return h.hexdigest()


before = digest()
cfg = load_parameters()["corner_trace_display"]
saved = cfg.get("kerb_gap_bridge", True)
counts = {}
try:
    for flag in (True, False):
        cfg["kerb_gap_bridge"] = flag
        ctd = CornerTraceDialog()
        n_corner = 0
        for s in summaries:
            ctd.show_corner(s, result, data)
            n_corner += sum(len(v) for v in ctd.lap_bridge_items.values())
        ltd = LapTraceDialog()
        valid = [l["lap_number"] for l in data["laps"] if l.get("is_valid_for_analysis")]
        ltd.show_lap(valid[0], result, data, classify_fn)
        n_lap = sum(len(v) for v in ltd.lap_bridge_items.values())
        counts[flag] = (n_corner, n_lap, "continuity only" in ltd.legend_label.text() or n_lap == 0)
        print(f"bridge {'on' if flag else 'off'}: corner-window bridge items {n_corner}, lap-window {n_lap}")
finally:
    cfg["kerb_gap_bridge"] = saved
after = digest()
assert counts[True][0] > 0 and counts[True][1] > 0, "expected bridges on a session with kerb strikes"
assert counts[False] [0] == 0 and counts[False][1] == 0, "flag off must draw no bridge"
assert counts[True][2], "lap-window legend must name the bridge when one is drawn"
assert before == after, "analysis arrays changed during rendering"
print("smoke OK: bridges only with the flag on; analysis arrays byte-identical before/after rendering")
