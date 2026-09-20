# Metrology Phase 1 supplement: corner-map composition figure.
# GPS (log_gps_lat/lon) is not valid on either real session -- censused
# directly (inspect_metrology_phase1_sensitivity.py's own saved apex_xy
# field came back all-None for every corner, both sessions) -- so
# apex_position_x/y_m (prepare_vehicle_state's GPS-projection output) is
# never populated here. A true 2-D corner map is not available from this
# telemetry. This script substitutes each corner's own mean lap-distance
# position (apex_lap_distance_m, a real field that does not depend on
# GPS) as a 1-D "corner sequence" x-axis instead -- an honestly-labelled
# proxy for "where on the lap", not a geometric map.

import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from modules.csv_parser import parse_csv

RESULTS_DIR = Path("diagnostics/results_metrology")
PLOTS_DIR = Path("diagnostics/plots_metrology")

DUBAI_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
V3_FILE = "GT3_PRC_MLA-v3.txt"
FILES = {"dubai": DUBAI_FILE, "v3": V3_FILE}

INPUTS = ["mass", "front_fraction", "wheelbase", "yaw_inertia", "steering_ratio"]


def _load(label):
    p = RESULTS_DIR / f"{label}.json"
    return json.load(open(p, encoding="utf-8")) if p.exists() else None


def _is_flip(o, n):
    oa = "understeer" if "understeer" in o else ("oversteer" if "oversteer" in o else None)
    na = "understeer" if "understeer" in n else ("oversteer" if "oversteer" in n else None)
    return oa is not None and na is not None and oa != na


def marginal_corners_for(session):
    base = _load(f"{session}_baseline_base")
    if base is None:
        return None, set()
    marginal = set()
    for input_name in INPUTS:
        for tag in ("plus1", "minus1"):
            var = _load(f"{session}_{input_name}_{tag}")
            if var is None:
                continue
            for key, base_v in base["verdicts"].items():
                var_v = var["verdicts"].get(key)
                if var_v is None or base_v == var_v:
                    continue
                if _is_flip(base_v[1], var_v[1]):
                    marginal.add(key.split("|", 1)[0])
    return base, marginal


def main():
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)
    for session, csv_file in FILES.items():
        base, marginal = marginal_corners_for(session)
        if base is None:
            print(f"{session}: no baseline result, skipped")
            continue
        data = parse_csv(csv_file)
        corners = data.get("corners", [])
        pos_by_corner = {}
        for c in corners:
            cid = c.get("stable_corner_id")
            d = c.get("apex_lap_distance_m")
            if cid is None or d is None:
                continue
            pos_by_corner.setdefault(cid, []).append(d)
        mean_pos = {cid: float(np.mean(ds)) for cid, ds in pos_by_corner.items()}

        fig, ax = plt.subplots(figsize=(10, 3))
        for cid, pos in sorted(mean_pos.items(), key=lambda kv: kv[1]):
            is_marginal = str(cid) in marginal
            ax.scatter(pos, 0, color=("red" if is_marginal else "tab:blue"),
                       s=120 if is_marginal else 60, zorder=3)
            ax.annotate(f"C{cid}", (pos, 0), textcoords="offset points", xytext=(0, 10),
                        ha="center", fontsize=8)
        ax.set_yticks([])
        ax.set_xlabel("mean apex lap-distance (m)")
        ax.set_title(f"{session}: corner sequence, GPS unavailable -- lap-distance proxy "
                     f"(red = at least one <=1% marginal phase)")
        fig.tight_layout()
        out = PLOTS_DIR / f"{session}_corner_sequence_marginal.png"
        fig.savefig(out, dpi=120)
        plt.close(fig)
        print(f"{session}: {len(marginal)} marginal corner(s) of {len(mean_pos)} -- figure saved: {out}")


if __name__ == "__main__":
    main()
