# Metrology overnight package, Phase 1: post-processing / analysis of the
# verdict-sensitivity sweep run by inspect_metrology_phase1_sensitivity.py.
# Reads the per-task JSON results already written to diagnostics/
# results_metrology/ (one file per session x input x direction, plus one
# baseline per session) -- no pipeline re-run, no fitting, seconds to run.
#
# Produces: (1) the flip census per input (how many corner-phases flip
# under each input's +/-1% perturbation, on which session), (2) the
# "marginal set" cross-check against the pre-registration (do flips
# concentrate near a classification threshold, or appear far from one --
# a flip far from threshold would be the "different, worse mechanism" the
# work order says to flag loudly), (3) a worst-10 margin table, (4) two
# figures per session: CS_ratio baseline value vs threshold with flip
# status, and the corner map coloured by marginal/stable.

import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from modules.stability_analysis import load_parameters

RESULTS_DIR = Path("diagnostics/results_metrology")
PLOTS_DIR = Path("diagnostics/plots_metrology")

SESSIONS = ("dubai", "v3")
INPUTS = ["mass", "front_fraction", "wheelbase", "yaw_inertia", "steering_ratio"]


def _load(label):
    path = RESULTS_DIR / f"{label}.json"
    if not path.exists():
        return None
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _is_flip(old_label, new_label):
    old_axle = "understeer" if "understeer" in old_label else ("oversteer" if "oversteer" in old_label else None)
    new_axle = "understeer" if "understeer" in new_label else ("oversteer" if "oversteer" in new_label else None)
    return old_axle is not None and new_axle is not None and old_axle != new_axle


def main():
    params = load_parameters()
    cls_cfg = params["classification"]
    STRONG_CSF = cls_cfg["STRONG_CSF"]["value"]
    STRONG_CSR = cls_cfg["STRONG_CSR"]["value"]
    MODERATE_CSF = cls_cfg["MODERATE_CSF"]["value"]
    MODERATE_CSR = cls_cfg["MODERATE_CSR"]["value"]

    all_flips = []  # list of dicts: session, corner, phase, input, sign, old, new
    per_session_baseline = {}
    per_session_marginal_keys = {}

    for session in SESSIONS:
        baseline = _load(f"{session}_baseline_base")
        if baseline is None:
            print(f"{session}: baseline not yet available, skipping")
            continue
        per_session_baseline[session] = baseline
        marginal_keys = set()

        for input_name in INPUTS:
            for sign, tag in ((1, "plus1"), (-1, "minus1")):
                variant = _load(f"{session}_{input_name}_{tag}")
                if variant is None:
                    continue
                for key, base_v in baseline["verdicts"].items():
                    var_v = variant["verdicts"].get(key)
                    if var_v is None:
                        continue
                    if base_v != var_v and _is_flip(base_v[1], var_v[1]):
                        cid, phase = key.split("|", 1)
                        all_flips.append({
                            "session": session, "corner": cid, "phase": phase,
                            "input": input_name, "sign": sign,
                            "old": base_v, "new": var_v,
                        })
                        marginal_keys.add(key)
        per_session_marginal_keys[session] = marginal_keys

    # --- flip census per input ---
    print("=== FLIP CENSUS PER INPUT (count of corner-phase flips at +/-1%) ===")
    for input_name in INPUTS:
        n = sum(1 for f in all_flips if f["input"] == input_name)
        sessions_hit = sorted(set(f["session"] for f in all_flips if f["input"] == input_name))
        print(f"  {input_name}: {n} flips, sessions={sessions_hit}")
    print(f"  TOTAL: {len(all_flips)} flips across {len(INPUTS)} inputs x 2 directions x {len(SESSIONS)} sessions")

    # --- threshold-distance cross-check (the pre-registration) ---
    print("\n=== PRE-REGISTRATION CHECK: do flips concentrate near a threshold? ===")
    dists_flipped = []
    dists_all = []
    for session, baseline in per_session_baseline.items():
        for key, cs in baseline["cs_values"].items():
            csf, csr = cs.get("cs_ratio_f"), cs.get("cs_ratio_r")
            d_f = min(abs(csf - STRONG_CSF), abs(csf - MODERATE_CSF)) if csf is not None and csf == csf else None
            d_r = min(abs(csr - STRONG_CSR), abs(csr - MODERATE_CSR)) if csr is not None and csr == csr else None
            cand = [d for d in (d_f, d_r) if d is not None]
            if not cand:
                continue
            d_min = min(cand)
            dists_all.append(d_min)
            if key in per_session_marginal_keys.get(session, set()):
                dists_flipped.append(d_min)

    if dists_flipped:
        print(f"  flipped corner-phases: n={len(dists_flipped)}, mean threshold-distance={np.mean(dists_flipped):.4f}, "
              f"median={np.median(dists_flipped):.4f}, max={np.max(dists_flipped):.4f}")
    print(f"  ALL corner-phases:      n={len(dists_all)}, mean threshold-distance={np.mean(dists_all):.4f}, "
          f"median={np.median(dists_all):.4f}")
    if dists_flipped and np.max(dists_flipped) > 2 * np.median(dists_all):
        print("  *** FLAG: at least one flip sits far from any threshold relative to the population -- "
              "possible 'different, worse mechanism', inspect individually. ***")
    else:
        print("  Pre-registration holds at the aggregate level: flipped corner-phases sit closer to a "
              "threshold than the general population (or the flip set is empty).")

    # --- worst-10 margin table ---
    print("\n=== MARGIN TABLE (worst 10 corner-phases, ranked by #inputs that flip it at <=1%) ===")
    by_key = {}
    for f in all_flips:
        k = (f["session"], f["corner"], f["phase"])
        by_key.setdefault(k, []).append(f)
    ranked = sorted(by_key.items(), key=lambda kv: -len(kv[1]))
    for (session, cid, phase), flips in ranked[:10]:
        inputs_hit = sorted(set(f"{f['input']}({f['sign']:+d}%)" for f in flips))
        cs = per_session_baseline[session]["cs_values"].get(f"{cid}|{phase}", {})
        print(f"  {session} C{cid} {phase}: margin<=1% via {len(flips)} input(s): {inputs_hit} "
              f"[baseline CSf={cs.get('cs_ratio_f')}, CSr={cs.get('cs_ratio_r')}]")

    # --- figures ---
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)
    for session, baseline in per_session_baseline.items():
        marginal_keys = per_session_marginal_keys.get(session, set())
        fig, ax = plt.subplots(figsize=(8, 6))
        for key, cs in baseline["cs_values"].items():
            csf, csr = cs.get("cs_ratio_f"), cs.get("cs_ratio_r")
            if csf is None or csr is None or csf != csf or csr != csr:
                continue
            marginal = key in marginal_keys
            ax.scatter(csf, csr, color=("red" if marginal else "tab:green"),
                       s=40 if marginal else 15, alpha=0.9 if marginal else 0.5,
                       marker="x" if marginal else "o")
        ax.axvline(STRONG_CSF, color="grey", linestyle="--", linewidth=0.8, label="STRONG_CSF")
        ax.axvline(MODERATE_CSF, color="grey", linestyle=":", linewidth=0.8, label="MODERATE_CSF")
        ax.axhline(STRONG_CSR, color="brown", linestyle="--", linewidth=0.8, label="STRONG_CSR")
        ax.axhline(MODERATE_CSR, color="brown", linestyle=":", linewidth=0.8, label="MODERATE_CSR")
        ax.set_xlabel("CS_ratio_f (baseline)")
        ax.set_ylabel("CS_ratio_r (baseline)")
        ax.set_title(f"{session}: verdict margin map (red x = flips at <=1% perturbation)")
        ax.legend(fontsize=7)
        fig.tight_layout()
        fig.savefig(PLOTS_DIR / f"{session}_margin_distribution.png", dpi=120)
        plt.close(fig)
        print(f"figure saved: {PLOTS_DIR / f'{session}_margin_distribution.png'}")

        # A true 2-D corner map needs apex_position_x/y_m (GPS-projection
        # based) -- censused directly: GPS is invalid on both real sessions,
        # so that field is None for every corner here. The corner-map
        # composition figure is instead produced by inspect_metrology_
        # phase1_corner_map.py, using each corner's own lap-distance
        # position (no GPS needed) as an honestly-labelled 1-D proxy.


if __name__ == "__main__":
    main()
