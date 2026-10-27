# Cross-lap corner aggregation and the phase / severity vocabulary shared by
# the classifier, the setup engine, the corner map and the limit events.

import numpy as np

# must match summarise_corners' phase keys and the e1..x5 feedback columns
PHASE_KEYS = ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]
PHASE_TO_FEEDBACK_KEY = dict(zip(PHASE_KEYS, ["e1", "e2", "a3", "x4", "x5"]))

# ordinal enum, method-defining
SEVERITY_RANK = {"normal": 0, "moderate": 1, "strong": 2}

# ordinal enum (corner_analysis speed_class)
SPEED_CLASS_ORDER = ["low", "medium", "high"]

PHASE_WORD_SHORT = {
    "entry_1_brake": "brake", "entry_2_turnin": "turn-in", "apex_3": "apex",
    "exit_4": "exit", "exit_5": "exit",
}
PHASE_WORD_LONG = {
    "entry_1_brake": "brake", "entry_2_turnin": "turn-in", "apex_3": "apex",
    "exit_4": "early exit", "exit_5": "late exit",
}


def _nanmedian_or_nan(values):
    valid = [v for v in values if v == v]  # drop NaN (NaN != NaN)
    if not valid:
        return float("nan")
    return float(np.median(valid))


def _nanmin_or_nan(values):
    valid = [v for v in values if v == v]
    if not valid:
        return float("nan")
    return float(np.min(valid))


def _group_by_corner(summaries):
    by_id = {}
    for s in summaries:
        cid = s.get("stable_corner_id")
        if cid is None:
            continue
        by_id.setdefault(cid, []).append(s)
    return by_id


def _aggregate_speed_class(lap_summaries):
    # modal speed_class across laps -> a corner near a class boundary does not
    # flip its speed gate per lap
    return modal_speed_class(s.get("speed_class") for s in lap_summaries)


def modal_speed_class(classes):
    # tie -> closest to medium, then lower class; None entries ignored
    counts = {}
    for sc in classes:
        if sc is None:
            continue
        counts[sc] = counts.get(sc, 0) + 1
    if not counts:
        return None
    max_n = max(counts.values())
    tied = [c for c, n in counts.items() if n == max_n]
    mid = SPEED_CLASS_ORDER.index("medium")
    tied.sort(key=lambda c: (abs(SPEED_CLASS_ORDER.index(c) - mid), SPEED_CLASS_ORDER.index(c)))
    return tied[0]


def aggregate_by_corner(summaries):
    # CS cross-lap combiner from config: "median" keeps what repeats every
    # lap; "worst_lap" = per-phase min across laps, composed with the
    # classifier's min over phases = global min over (lap, phase).
    # Thresholds must be re-derived for whichever population is used.
    # Stability always median.
    from modules.stability_analysis import load_parameters
    cs_aggregation = load_parameters()["classification"].get(
        "cs_cross_lap_aggregation", "median"
    )
    cs_combine = _nanmin_or_nan if cs_aggregation == "worst_lap" else _nanmedian_or_nan

    by_id = _group_by_corner(summaries)

    aggregated = {}
    for cid, corner_summaries in by_id.items():
        phases = {}
        for phase in PHASE_KEYS:
            csf, csr, stab = [], [], []
            for s in corner_summaries:
                p = s["phases"].get(phase)
                if p is None:
                    continue
                csf.append(p["cs_ratio_f"]["median"])
                csr.append(p["cs_ratio_r"]["median"])
                stab.append(p["stability_observed_Nm_per_deg"]["median"])
            phases[phase] = {
                "cs_ratio_f": {"median": cs_combine(csf)},
                "cs_ratio_r": {"median": cs_combine(csr)},
                "stability_observed_Nm_per_deg": {"median": _nanmedian_or_nan(stab)},
            }
        # same combiner for apex_region (the classifier reads it for apex_3);
        # a summary without it is skipped like any other gap
        ar_csf = [s["apex_region"]["cs_ratio_f"]["median"] for s in corner_summaries if s.get("apex_region")]
        ar_csr = [s["apex_region"]["cs_ratio_r"]["median"] for s in corner_summaries if s.get("apex_region")]
        aggregated[cid] = {
            "stable_corner_id": cid,
            "n_laps": len(corner_summaries),
            "speed_class": _aggregate_speed_class(corner_summaries),
            "phases": phases,
            "apex_region": {
                "cs_ratio_f": {"median": cs_combine(ar_csf)},
                "cs_ratio_r": {"median": cs_combine(ar_csr)},
            },
        }
    return aggregated
