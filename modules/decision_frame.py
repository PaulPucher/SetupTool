# Decision frame: evidence -> candidates -> scoring, plus a conflict
# resolver. Production recommendation path; all 39 rules of
# recommendations.json run here as candidate bridges (parity:
# diagnostics/inspect_frame_stage2_parity.py), using the rule table and
# helpers from modules/recommendation.py.

import json

import numpy as np

from modules.stability_analysis import load_parameters
from modules.csv_parser import load_channels_config
from modules.wheel_loads import (
    CORNERS as _WHEEL_CORNERS, AXLE_CORNERS, TRAVEL_CHANNEL,
    _interp_channel, _normalize_travel_to_mm, _channel_is_dead,
)
from modules.recommendation import (
    PHASE_KEYS,
    PHASE_TO_FEEDBACK_KEY,
    SEVERITY_RANK,
    _NON_FIRING_STATUSES,
    _action_key,
    _axle_verdict,
    _current_setup_value,
    _feedback_row,
    _group_by_corner,
    _nanmin_or_nan,
    _phase_verdict,
    _verdict_present,
    aggregate_by_corner,
    load_recommendations_config,
    load_setup_parameters_registry,
)

DECISION_FRAME_CONFIG_PATH = "config/decision_frame.json"


def load_decision_frame_config():
    with open(DECISION_FRAME_CONFIG_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def _axle_cs_severity(cs_median, strong_thresh, moderate_thresh):
    # same strong/moderate boundaries as _classify_corner, but per axle --
    # brake_balance_signature needs front and rear separately
    if cs_median != cs_median:  # NaN: no signal for this axle/phase
        return None
    if cs_median < strong_thresh:
        return "strong"
    if cs_median < moderate_thresh:
        return "moderate"
    return "normal"


def _count_repeating(cid, by_corner_laps, predicate):
    # lap count like _consistency_gate_ok, but feeds a graded confidence
    # instead of a pass/fail gate
    laps = by_corner_laps.get(cid, [])
    if not laps:
        return 0, 0
    return sum(1 for lap in laps if predicate(lap)), len(laps)


def _fraction(numerator, denominator):
    return 0.0 if denominator == 0 else numerator / denominator


def aggregate_ls_by_corner(summaries):
    """Per corner and phase: worst-lap LS_ratio_f/r (min of min), CS's
    cross-lap policy reused -- LS has no policy of its own derived yet.
    Summaries built without ls -> NaN, no signal.
    """
    by_id = _group_by_corner(summaries)
    out = {}
    for cid, laps in by_id.items():
        phases = {}
        for phase in PHASE_KEYS:
            f_vals, r_vals = [], []
            for lap in laps:
                p = lap["phases"].get(phase)
                if p is None:
                    continue
                lf = p.get("ls_ratio_f")
                lr = p.get("ls_ratio_r")
                if lf is not None and lf["median"] == lf["median"]:
                    f_vals.append(lf["median"])
                if lr is not None and lr["median"] == lr["median"]:
                    r_vals.append(lr["median"])
            phases[phase] = {
                "ls_ratio_f": _nanmin_or_nan(f_vals),
                "ls_ratio_r": _nanmin_or_nan(r_vals),
                "n_contributing_laps_f": len(f_vals),
                "n_contributing_laps_r": len(r_vals),
            }
        out[cid] = phases
    return out


def _build_corner_verdict_evidence(aggregated, by_corner_laps, classify_fn, config=None):
    # (a) corner verdicts via classify_fn -- same aggregate and thresholds as
    # the stability grid, can't disagree with it.
    # "[MARGINAL]" verdict (within cs_margin of a threshold) -> confidence
    # capped at 0.8 via min() (same cap as ABS); strong repeats pulled down,
    # weak ones stay below.
    margin_confidence_cap = (config or {}).get("intervention_evidence", {}).get("abs", {}).get("confidence", 0.8)
    evidence = []
    for cid, corner in aggregated.items():
        for phase in PHASE_KEYS:
            if phase not in corner["phases"]:
                continue
            severity, short = _phase_verdict(corner, [phase], classify_fn)
            if severity == "normal":
                continue

            verdicts_here = []
            axle = _axle_verdict(short)
            if axle is not None:
                verdicts_here.append(axle)
            if _verdict_present(short, "unstable_yaw"):
                verdicts_here.append("unstable_yaw")
            # classify_fn gives one severity per phase -> axle and yaw evidence at the
            # same phase share it; no per-verdict severity scale exists
            for verdict in verdicts_here:
                def _lap_matches(lap_summary, phase=phase, verdict=verdict, severity=severity):
                    lap_sev, lap_short = _phase_verdict(lap_summary, [phase], classify_fn)
                    return (_verdict_present(lap_short, verdict)
                            and SEVERITY_RANK[lap_sev] >= SEVERITY_RANK[severity])

                repeat, total = _count_repeating(cid, by_corner_laps, _lap_matches)
                valid_laps = sum(
                    1 for lap in by_corner_laps.get(cid, [])
                    if lap["phases"].get(phase, {}).get("n_samples", 0) > 0
                )
                confidence = round(_fraction(repeat, total) * _fraction(valid_laps, total), 3)
                is_marginal = "[MARGINAL]" in short
                if is_marginal:
                    confidence = min(confidence, margin_confidence_cap)

                evidence.append({
                    "type": "corner_verdict",
                    "corner": cid,
                    "phase": phase,
                    "speed_class": corner.get("speed_class"),
                    "verdict": verdict,
                    "severity": severity,
                    "confidence": confidence,
                    "marginal": is_marginal,
                    "source": f"classify_fn (worst-lap aggregate, anchored thresholds): "
                              f"C{cid} {phase} '{short}' -- repeats on {repeat}/{total} laps, "
                              f"signal present on {valid_laps}/{total} laps"
                              + (f", MARGINAL: capped at {margin_confidence_cap}" if is_marginal else ""),
                })
    return evidence


def _build_ls_disambiguation_evidence(aggregated, aggregated_ls, corner_verdict_evidence):
    # (b) traction- vs cornering-limited, exit oversteer only, valid LS only
    # (method from diagnostics/inspect_ls_cs_disambiguation.py). Split is
    # population-relative (median LS_ratio_r over this session's oversteer
    # corners at that phase) -- no absolute LS threshold at this point.
    exit_phases = ("exit_4", "exit_5")
    candidates = [e for e in corner_verdict_evidence
                  if e["verdict"] == "oversteer" and e["phase"] in exit_phases]
    if not candidates:
        return []

    population = []
    for e in candidates:
        ls_r = aggregated_ls.get(e["corner"], {}).get(e["phase"], {}).get("ls_ratio_r")
        if ls_r is not None and ls_r == ls_r:
            population.append(ls_r)
    if len(population) < 2:
        # relative split needs >= 2 values
        return []
    ls_median = float(np.median(population))

    evidence = []
    for e in candidates:
        phase_ls = aggregated_ls.get(e["corner"], {}).get(e["phase"], {})
        ls_r = phase_ls.get("ls_ratio_r")
        if ls_r is None or ls_r != ls_r:
            continue  # LS invalid here -> no evidence
        n_contrib = phase_ls.get("n_contributing_laps_r", 0)
        # real per-corner lap count, not assumed
        corner_n_laps = aggregated.get(e["corner"], {}).get("n_laps", 0)
        confidence = round(_fraction(n_contrib, corner_n_laps), 3)
        evidence.append({
            "type": "ls_disambiguation",
            "corner": e["corner"],
            "phase": e["phase"],
            "speed_class": e.get("speed_class"),
            "verdict": e["verdict"],
            "severity": e["severity"],
            "confidence": confidence,
            "ls_class": "traction_limited" if ls_r < ls_median else "cornering_limited",
            "source": f"LS_ratio_r={ls_r:.3f} vs session median {ls_median:.3f} "
                      f"(population-relative split, n={len(population)} corners; "
                      f"method: diagnostics/inspect_ls_cs_disambiguation.py, "
                      f"no absolute LS_ratio threshold exists in config -- see PLAN.md STEP 4)",
        })
    return evidence


def _build_ls_threshold_evidence(aggregated_ls, by_corner_laps, config):
    """Absolute LS thresholds (ls_threshold_evidence.STRONG_LSF/LSR) on any
    phase, independent of the relative split. Evidence only, not a verdict
    tier -- never reaches _classify_corner or the UI colours.

    Confidence = repeat fraction x valid fraction (same as corner verdicts).
    exit_4/exit_5 capped via min() at exit_phase_confidence_discount (0.5):
    in the census, phase type decided whether a negative reading
    corroborated. Repeats lift exit readings up to the cap, never above; a
    one-off stays low. Braking/turn-in/apex uncapped (89%/72% corroborated).
    """
    ls_cfg = config.get("ls_threshold_evidence", {})
    if not ls_cfg.get("enabled", False):
        return []
    strong_lsf = ls_cfg["STRONG_LSF"]
    strong_lsr = ls_cfg["STRONG_LSR"]
    exit_phases = tuple(ls_cfg.get("exit_phases", ("exit_4", "exit_5")))
    braking_turnin_phases = tuple(ls_cfg.get("braking_turnin_phases",
                                              ("entry_1_brake", "entry_2_turnin", "apex_3")))
    exit_discount = ls_cfg["exit_phase_confidence_discount"]

    evidence = []
    for cid, phases in aggregated_ls.items():
        for phase in braking_turnin_phases + exit_phases:
            p = phases.get(phase)
            if p is None:
                continue
            for axle, key, thresh, verdict in (
                ("front", "ls_ratio_f", strong_lsf, "brake_limited"),
                ("rear", "ls_ratio_r", strong_lsr, "traction_limited"),
            ):
                val = p.get(key)
                if val is None or val != val or val >= thresh:
                    continue

                def _lap_crosses(lap_summary, phase=phase, key=key, thresh=thresh):
                    lv_entry = lap_summary["phases"].get(phase, {}).get(key)
                    lv = lv_entry.get("median") if isinstance(lv_entry, dict) else None
                    return lv is not None and lv == lv and lv < thresh

                repeat, total = _count_repeating(cid, by_corner_laps, _lap_crosses)
                valid_laps = sum(
                    1 for lap in by_corner_laps.get(cid, [])
                    if lap["phases"].get(phase, {}).get("n_samples", 0) > 0
                )
                confidence = round(_fraction(repeat, total) * _fraction(valid_laps, total), 3)
                is_exit = phase in exit_phases
                if is_exit:
                    confidence = min(confidence, exit_discount)

                evidence.append({
                    "type": "ls_threshold",
                    "corner": cid,
                    "phase": phase,
                    "axle": axle,
                    "verdict": verdict,
                    "confidence": confidence,
                    "phase_scope": "exit" if is_exit else "braking_turnin",
                    "source": f"LS_ratio_{('f' if axle=='front' else 'r')}={val:.3f} < "
                              f"{'STRONG_LSF' if axle=='front' else 'STRONG_LSR'}={thresh:.2f} "
                              f"(LS-evidence census, thesis_notes.md 'LS-evidence work package'); "
                              f"repeats on {repeat}/{total} laps, signal on {valid_laps}/{total} laps"
                              + (f"; exit-phase confidence capped at {exit_discount} "
                                 f"(TC corroboration structurally unavailable this session, "
                                 f"34% corroborated in the census)" if is_exit else ""),
                })
    return evidence


def _build_brake_balance_evidence(aggregated, by_corner_laps, config):
    # (c) front past its limit while rear healthy under braking. Thresholds
    # read from parameters.json classification, never copied -> can't drift.
    settings = config["plausibility_checks"]["brake_balance_signature"]
    phase = settings["phase"]
    front_min = settings["front_beyond_severity_min"]
    rear_max = settings["rear_healthy_severity_max"]
    cls_cfg = load_parameters()["classification"]
    strong_csf = cls_cfg["STRONG_CSF"]["value"]
    moderate_csf = cls_cfg["MODERATE_CSF"]["value"]
    strong_csr = cls_cfg["STRONG_CSR"]["value"]
    moderate_csr = cls_cfg["MODERATE_CSR"]["value"]

    def _axle_severities(summary):
        p = summary["phases"].get(phase)
        if p is None:
            return None, None
        f_sev = _axle_cs_severity(p["cs_ratio_f"]["median"], strong_csf, moderate_csf)
        r_sev = _axle_cs_severity(p["cs_ratio_r"]["median"], strong_csr, moderate_csr)
        return f_sev, r_sev

    def _fires(summary):
        f_sev, r_sev = _axle_severities(summary)
        if f_sev is None or r_sev is None:
            return False
        return (SEVERITY_RANK[f_sev] >= SEVERITY_RANK[front_min]
                and SEVERITY_RANK[r_sev] <= SEVERITY_RANK[rear_max])

    evidence = []
    for cid, corner in aggregated.items():
        if not _fires(corner):
            continue
        f_sev, r_sev = _axle_severities(corner)
        repeat, total = _count_repeating(cid, by_corner_laps, _fires)
        valid_laps = sum(
            1 for lap in by_corner_laps.get(cid, [])
            if lap["phases"].get(phase, {}).get("n_samples", 0) > 0
        )
        confidence = round(_fraction(repeat, total) * _fraction(valid_laps, total), 3)
        evidence.append({
            "type": "plausibility_brake_balance",
            "corner": cid,
            "phase": phase,
            "speed_class": corner.get("speed_class"),
            "verdict": None,
            "severity": f_sev,
            "confidence": confidence,
            "source": f"C{cid} {phase}: front CS severity={f_sev} (>= {front_min}), "
                      f"rear CS severity={r_sev} (<= {rear_max}) -- front beyond limit, "
                      f"rear healthy; repeats on {repeat}/{total} laps",
        })
    return evidence


# Stage 2 (d): matrix-rule verdicts -- same classify_fn/_phase_verdict call
# as the rule engine's data trigger, over the phase groups the matrix uses
# (four single phases + exit_4/exit_5 pair, derived from the rules).
# Separate type "matrix_verdict" so corner_verdict stays unchanged.

def _distinct_phase_groups(config_recs):
    return sorted({tuple(r["phases"]) for r in config_recs["rules"]})


def _build_matrix_verdict_evidence(aggregated, by_corner_laps, classify_fn, config_recs, config=None):
    # same MARGINAL cap as corner verdicts -- this path feeds most candidates
    margin_confidence_cap = (config or {}).get("intervention_evidence", {}).get("abs", {}).get("confidence", 0.8)
    evidence = []
    for phase_group in _distinct_phase_groups(config_recs):
        phases = list(phase_group)
        for cid, corner in aggregated.items():
            if not any(p in corner["phases"] for p in phases):
                continue
            severity, short = _phase_verdict(corner, phases, classify_fn)
            if severity == "normal":
                continue

            verdicts_here = []
            axle = _axle_verdict(short)
            if axle is not None:
                verdicts_here.append(axle)
            if _verdict_present(short, "unstable_yaw"):
                verdicts_here.append("unstable_yaw")

            for verdict in verdicts_here:
                def _lap_matches(lap_summary, phases=phases, verdict=verdict, severity=severity):
                    lap_sev, lap_short = _phase_verdict(lap_summary, phases, classify_fn)
                    return (_verdict_present(lap_short, verdict)
                            and SEVERITY_RANK[lap_sev] >= SEVERITY_RANK[severity])

                repeat, total = _count_repeating(cid, by_corner_laps, _lap_matches)
                valid_laps = sum(
                    1 for lap in by_corner_laps.get(cid, [])
                    if any(lap["phases"].get(p, {}).get("n_samples", 0) > 0 for p in phases)
                )
                confidence = round(_fraction(repeat, total) * _fraction(valid_laps, total), 3)
                is_marginal = "[MARGINAL]" in short
                if is_marginal:
                    confidence = min(confidence, margin_confidence_cap)

                evidence.append({
                    "type": "matrix_verdict",
                    "corner": cid,
                    "phases": phase_group,
                    "speed_class": corner.get("speed_class"),
                    "verdict": verdict,
                    "severity": severity,
                    "confidence": confidence,
                    "marginal": is_marginal,
                    "source": f"classify_fn (worst-lap aggregate, anchored thresholds), phases="
                              f"{'+'.join(phase_group)}: C{cid} '{short}' -- repeats on {repeat}/{total} "
                              f"laps, signal present on {valid_laps}/{total} laps"
                              + (f", MARGINAL: capped at {margin_confidence_cap}" if is_marginal else ""),
                })
    return evidence


# Intervention evidence (ABS/TC), config-gated per source. Corroborates
# existing candidates only, never its own. Needs per-lap phase windows
# (corners/state/channels); any missing -> skipped.

def _phase_window_indices(t, segment):
    # same searchsorted slicing as summarise_corners' private _phase_slice
    if segment is None:
        return None
    start_t, end_t = segment
    if end_t < start_t:
        return None
    lo = int(np.searchsorted(t, start_t, side="left"))
    hi = int(np.searchsorted(t, end_t, side="right"))
    if hi <= lo:
        return None
    return lo, hi


def _log_abs_pos_at(channels, t, idx):
    # trusted ABS position (log_abs_pos, not abs_switch_pos), for the source
    # string only -- positions are categorical, no ordering to route on
    ch = (channels or {}).get("log_abs_pos")
    if ch is None or ch.get("quality") in ("missing", "failed") or ch.get("time") is None or idx is None:
        return None
    lo, hi = idx
    val = np.interp(t, ch["time"], ch["data"])[lo:hi]
    if val.size == 0:
        return None
    return float(np.median(val))


def _build_intervention_abs_evidence(corners, by_corner_laps_ignored, state, channels, aggregated, abs_config):
    abs_ch = (channels or {}).get("abs_active")
    if state is None or abs_ch is None or abs_ch.get("quality") in ("missing", "failed") or abs_ch.get("time") is None:
        return []
    t = state["time"]
    abs_on_ref = np.interp(t, abs_ch["time"], abs_ch["data"]) > 0.5
    confidence_cap = abs_config.get("confidence", 1.0)
    heavy_cfg = abs_config.get("abs_heavy_masks_verdict", {})
    heavy_threshold = heavy_cfg.get("heavy_duty_cycle_threshold", 1.0)

    by_corner = {}
    for c in corners or []:
        cid = c.get("stable_corner_id")
        if cid is not None:
            by_corner.setdefault(cid, []).append(c)

    evidence = []
    for cid, instances in by_corner.items():
        inactive_count, heavy_count, total = 0, 0, 0
        example_idx_inactive, example_idx_heavy = None, None
        for c in instances:
            idx = _phase_window_indices(t, c.get("segments", {}).get("entry_1_brake"))
            if idx is None:
                continue
            lo, hi = idx
            total += 1
            window = abs_on_ref[lo:hi]
            if not window.any():
                inactive_count += 1
                example_idx_inactive = idx
            elif window.mean() >= heavy_threshold:
                heavy_count += 1
                example_idx_heavy = idx

        if total and inactive_count:
            # engineer rule: "instability/locking under braking + ABS not
            # intervening -> ABS map up"; corroborates unstable_yaw on entry_1_brake
            confidence = min(confidence_cap, round(inactive_count / total, 3))
            pos = _log_abs_pos_at(channels, t, example_idx_inactive)
            pos_txt = f", log_abs_pos={pos:.0f}" if pos is not None else ""
            evidence.append({
                "type": "intervention_abs",
                "corner": cid, "phases": ("entry_1_brake",),
                "speed_class": aggregated.get(cid, {}).get("speed_class"),
                "verdict": "unstable_yaw", "severity": None, "confidence": confidence,
                "source": f"abs_active read 0 throughout entry_1_brake on {inactive_count}/{total} analysed "
                          f"laps{pos_txt} -- corroborates braking-phase instability per the user's own rule: "
                          f"'ABS inactive + instability under braking -> more ABS'.",
            })

        if total and heavy_count:
            # engineer rule: "ABS regulating heavily through braking zones -> flag as
            # masking, prefer brake-balance/platform levers". A flag, verdict None --
            # it questions the phase's verdict rather than supporting one.
            confidence = min(confidence_cap, round(heavy_count / total, 3))
            pos = _log_abs_pos_at(channels, t, example_idx_heavy)
            pos_txt = f", log_abs_pos={pos:.0f}" if pos is not None else ""
            evidence.append({
                "type": "intervention_abs_masking",
                "corner": cid, "phases": ("entry_1_brake",),
                "speed_class": aggregated.get(cid, {}).get("speed_class"),
                "verdict": None, "severity": None, "confidence": confidence,
                "masked_by_heavy_abs": True,
                "source": f"abs_active duty cycle >= {heavy_threshold:.0%} of entry_1_brake on {heavy_count}/"
                          f"{total} analysed laps{pos_txt} -- per the user's own rule, this corner/phase's own "
                          f"braking-phase verdict may reflect ABS regulation rather than raw mechanical "
                          f"balance; prefer brake-balance/platform levers over a literal reading.",
            })
    return evidence


def _build_intervention_tc_evidence(corners, state, channels, aggregated):
    tc_ch = (channels or {}).get("ecu_B_tc_act")
    if state is None or tc_ch is None or tc_ch.get("quality") in ("missing", "failed") or tc_ch.get("time") is None:
        return []
    t = state["time"]
    tc_on_ref = np.interp(t, tc_ch["time"], tc_ch["data"]) > 0.5

    by_corner = {}
    for c in corners or []:
        cid = c.get("stable_corner_id")
        if cid is not None:
            by_corner.setdefault(cid, []).append(c)

    evidence = []
    for cid, instances in by_corner.items():
        active_count, total = 0, 0
        for c in instances:
            segs = c.get("segments", {})
            windows = [w for p in EXIT_PHASES for w in [_phase_window_indices(t, segs.get(p))] if w is not None]
            if not windows:
                continue
            total += 1
            if any(tc_on_ref[lo:hi].any() for lo, hi in windows):
                active_count += 1
        if total == 0 or active_count == 0:
            continue
        confidence = round(active_count / total, 3)
        evidence.append({
            "type": "intervention_tc",
            "corner": cid, "phases": EXIT_PHASES,
            "speed_class": aggregated.get(cid, {}).get("speed_class"),
            "verdict": "oversteer", "severity": None, "confidence": confidence,
            "source": f"ecu_B_tc_act read 1 during exit_4/exit_5 on {active_count}/{total} analysed laps "
                      f"(USABLE-NOW boolean, Frame-Stage-2 Phase 2) -- corroborates traction-limited per "
                      f"the user's own rule: 'TC cutting hard + exit oversteer -> corroborates "
                      f"traction-limited'.",
        })
    return evidence


# Kerb-strike blowoff evidence ("curb strikes / hard bumps -> more
# blowoff"). Trigger = strike SEVERITY repeating across laps (peak
# |log_acc_z| inside the existing kerb_mask), never kerb_fraction alone --
# kerb contact itself is normal.
# Threshold = pooled p75 (diagnostics/inspect_kerb_severity_census.py):
# lowest candidate where firing is a minority on both sessions (Dubai
# 1/14, v3 4/17); p50 fires on ~half. Repeat: >= 2 laps (_count_repeating).

def _corner_overall_window(corner, t):
    segments = corner.get("segments", {})
    starts = [s for s, e in segments.values() if e >= s]
    ends = [e for s, e in segments.values() if e >= s]
    if not starts:
        return None
    return _phase_window_indices(t, (min(starts), max(ends)))


def _kerb_axle_attribution(instances_lo_hi, t, channels, wl_cfg):
    # peak |travel rate| INSIDE the kerb windows -- opposite of damper_motion,
    # which excludes them; here the hit is the signal
    dead_std_max = wl_cfg["dead_channel_std_max_travel_mm"]
    peak_rate_by_wheel = {}
    for wheel in _WHEEL_CORNERS:
        ch = channels.get(TRAVEL_CHANNEL[wheel])
        raw_native = _interp_channel(channels, TRAVEL_CHANNEL[wheel], t)
        if ch is None or raw_native is None or ch.get("quality") != "valid":
            continue
        raw_mm = _normalize_travel_to_mm(raw_native, ch.get("unit_raw"))
        if _channel_is_dead(raw_mm, dead_std_max):
            continue
        peaks = []
        for lo, hi in instances_lo_hi:
            seg_t, seg_travel = t[lo:hi], raw_mm[lo:hi]
            valid = np.isfinite(seg_travel)
            if valid.sum() < 2:
                continue
            idx = np.where(valid)[0]
            rates = np.gradient(seg_travel[idx], seg_t[idx])
            peaks.append(float(np.max(np.abs(rates))))
        if peaks:
            peak_rate_by_wheel[wheel] = max(peaks)

    axle_score = {}
    for axle, wheels in AXLE_CORNERS.items():
        scores = [peak_rate_by_wheel[w] for w in wheels if w in peak_rate_by_wheel]
        if len(scores) == len(wheels):  # both wheels evaluable
            axle_score[axle] = max(scores)

    if len(axle_score) < 2:
        return None  # not attributable -> both axles
    return max(axle_score, key=axle_score.get)


def _build_kerb_blowoff_evidence(corners, state, channels, kb_cfg, wl_cfg):
    if state is None or channels is None or not corners:
        return []
    t = state["time"]
    az_g = state.get("az_g")
    kerb_mask = state.get("kerb_mask")
    if az_g is None or kerb_mask is None:
        return []

    threshold = kb_cfg["severity_threshold_g"]
    repeat_min = kb_cfg.get("repeat_min_laps", 2)

    by_corner = {}
    for c in corners:
        cid = c.get("stable_corner_id")
        if cid is not None:
            by_corner.setdefault(cid, []).append(c)

    evidence = []
    for cid, instances in by_corner.items():
        peaks = []  # (lo, hi, peak_g)
        for c in instances:
            w = _corner_overall_window(c, t)
            if w is None:
                continue
            lo, hi = w
            window_kerb = kerb_mask[lo:hi]
            peak = float(np.max(np.abs(az_g[lo:hi][window_kerb]))) if window_kerb.any() else 0.0
            peaks.append((lo, hi, peak))
        if not peaks:
            continue
        total = len(peaks)
        firing = [(lo, hi) for lo, hi, p in peaks if p >= threshold]
        if len(firing) < repeat_min:
            continue  # not enough laps repeat

        axle = _kerb_axle_attribution(firing, t, channels, wl_cfg)
        max_severity = max(p for _, _, p in peaks if p >= threshold)
        evidence.append({
            "type": "kerb_blowoff",
            "corner": cid, "phase": None,
            "axle": axle,  # None -> both axles proposed
            "severity": None, "confidence": round(len(firing) / total, 3),
            "peak_severity_g": round(max_severity, 3),
            "source": f"peak |log_acc_z| inside kerb_mask exceeded {threshold:.4f}g on "
                      f"{len(firing)}/{total} laps at this corner -- author-elicited 2026-09-24: "
                      f"curb strikes / hard bumps -> more blowoff. Axle attribution: "
                      f"{axle or 'not attributable (car-wide kerb evidence)'}.",
        })
    return evidence


def _kerb_blowoff_candidates(evidence, registry):
    """Advisory click-class candidates, one exploratory step per axle
    (damper_blowoff_* adjustment_step.exploratory). Rear proposable like
    front. Axle not attributable -> both axles, labelled.
    """
    candidates = []
    for e in evidence:
        if e["type"] != "kerb_blowoff":
            continue
        attributable = e["axle"] is not None
        axles = [e["axle"]] if attributable else ["front", "rear"]
        for axle in axles:
            params = [f"damper_blowoff_{w}" for w in AXLE_CORNERS[axle]]
            step = registry[params[0]].get("adjustment_step", {}).get("exploratory", 1)
            actions = [{"parameter": p, "direction": "increase", "delta": step} for p in params]
            axle_note = (f"{axle} axle" if attributable else
                         "car-wide kerb evidence, axle not attributable -- both axles proposed")
            candidates.append({
                "id": f"kerb_blowoff_increase_{axle}:C{e['corner']}",
                "scenario": "kerb_blowoff",
                "corner": e["corner"], "phase": None,
                "lever_family": "damper_blowoff",
                "actions": actions,
                "effort_class": _effort_class_for_actions(params, registry),
                "effect_class": "secondary",
                "grade": "proposed",
                "cell_id": None,
                "evidence_refs": [e],
                "rationale": f"Repeated hard kerb contact at this corner (peak "
                             f"{e['peak_severity_g']}g, {axle_note}) -- author-elicited 2026-09-24: "
                             f"curb strikes / hard bumps call for more blowoff relief to protect the "
                             f"contact patch over the worst hits.",
            })
    return candidates


# Driver-feedback evidence. Never its own candidate -- corroborates an
# existing one at the same corner/phase/verdict (_attach_feedback_evidence).

def _feedback_confidence(magnitude, feedback_cfg):
    """Band-shaped, not linear: |1| slight, |3| strong, |5| undrivable are
    different regimes. Bands from config (values provisional). First band
    whose max_abs isn't exceeded wins; beyond all bands -> last band's value
    (<= 1.0).
    """
    bands = feedback_cfg["confidence_bands"]
    for band in bands:
        if magnitude <= band["max_abs"]:
            return band["value"]
    return bands[-1]["value"]


def _build_driver_feedback_evidence(feedback_data, aggregated, feedback_cfg):
    if not feedback_data:
        return []

    evidence = []
    for cid, corner in aggregated.items():
        fb_row = _feedback_row(feedback_data, cid)
        if not fb_row:
            continue
        for phase in PHASE_KEYS:
            key = PHASE_TO_FEEDBACK_KEY.get(phase)
            if key is None:
                continue
            raw = fb_row.get(key, 0)
            if not raw:
                continue
            magnitude = abs(raw)
            confidence = _feedback_confidence(magnitude, feedback_cfg)
            evidence.append({
                "type": "driver_feedback",
                "corner": cid, "phase": phase,
                "speed_class": corner.get("speed_class"),
                "verdict": "oversteer" if raw > 0 else "understeer",
                "severity": None, "confidence": confidence, "raw_feedback": raw,
                "source": f"driver feedback {raw:+g} at {phase} (magnitude {magnitude:g}; "
                          f"band-shaped confidence {confidence} -- Phase D ITEM 2(c), 2026-09-23, "
                          f"band values provisional pending elicitation)",
            })
    return evidence


def _attach_feedback_evidence(candidates, evidence):
    feedback_by_key = {}
    for e in evidence:
        if e["type"] == "driver_feedback":
            feedback_by_key.setdefault((e["corner"], e["phase"]), []).append(e)
    if not feedback_by_key:
        return candidates

    for c in candidates:
        own_verdicts_by_phase = {}
        for ref in c["evidence_refs"]:
            verdict = ref.get("verdict")
            if verdict is None:
                continue
            phases = ref["phases"] if "phases" in ref else (ref.get("phase"),)
            for p in phases:
                if p is not None:
                    own_verdicts_by_phase.setdefault(p, set()).add(verdict)
        own_ids = {id(r) for r in c["evidence_refs"]}
        for phase, verdicts in own_verdicts_by_phase.items():
            for fb in feedback_by_key.get((c["corner"], phase), []):
                if fb["verdict"] in verdicts and id(fb) not in own_ids:
                    c["evidence_refs"].append(fb)
                    own_ids.add(id(fb))
                    # data agreeing with feedback = strongest trigger class; feedback-only
                    # candidates have no data verdict to agree with
                    if c.get("trigger_provenance") == TRIGGER_DATA_ONLY:
                        c["trigger_provenance"] = TRIGGER_BOTH_AGREEING
    return candidates


# Driver vs data disagreement. Only oversteer/understeer have an opposite;
# unstable_yaw never gets conflicting_feedback.
_OPPOSITE_VERDICT = {"oversteer": "understeer", "understeer": "oversteer"}


def _attach_conflicting_feedback(candidates, evidence):
    """Driver contradicts data -> both shown, engineer decides; never suppresses.
    Adds opposite-verdict feedback to conflicting_feedback; status,
    confidence and evidence_refs untouched. An item can't land in both lists.
    """
    feedback_by_key = {}
    for e in evidence:
        if e["type"] == "driver_feedback":
            feedback_by_key.setdefault((e["corner"], e["phase"]), []).append(e)

    for c in candidates:
        if not feedback_by_key:
            c["conflicting_feedback"] = []
            continue
        own_verdicts_by_phase = {}
        for ref in c["evidence_refs"]:
            verdict = ref.get("verdict")
            if verdict is None:
                continue
            phases = ref["phases"] if "phases" in ref else (ref.get("phase"),)
            for p in phases:
                if p is not None:
                    own_verdicts_by_phase.setdefault(p, set()).add(verdict)
        conflicting = []
        seen_ids = set()
        for phase, verdicts in own_verdicts_by_phase.items():
            opposites = {_OPPOSITE_VERDICT[v] for v in verdicts if v in _OPPOSITE_VERDICT}
            if not opposites:
                continue
            for fb in feedback_by_key.get((c["corner"], phase), []):
                if fb["verdict"] in opposites and id(fb) not in seen_ids:
                    conflicting.append(fb)
                    seen_ids.add(id(fb))
        c["conflicting_feedback"] = conflicting
    return candidates


def build_evidence(summaries, ls_stats, config, classify_fn, corners=None, state=None, channels=None,
                    feedback_data=None):
    """Stability summaries -> flat evidence list {type, corner, phase, verdict,
    severity, confidence, source, ...}. Confidence = plain fraction of real
    counts (repeating laps x signal validity).

    (a) corner_verdict via classify_fn -- can't disagree with the grid
    (b) ls_disambiguation: exit oversteer, traction vs cornering limited
    (c) plausibility_brake_balance
    (d) matrix_verdict: classify_fn over the matrix phase groups
    (e)/(f) ABS / ABS-masking / TC intervention, gated per source (ABS on,
        TC off); needs corners/state/channels, else skipped
    (g) ls_threshold (config-gated), absolute LS thresholds
    (h) damper_motion (config-gated), entry/exit only
    + driver_feedback when feedback_data given; kerb blowoff when corners given.

    ls_stats = aggregate_ls_by_corner(summaries), passed in so callers can
    reuse it. classify_fn = the UI's classifier, passed in -- modules/ can't
    import ui/.
    """
    aggregated = aggregate_by_corner(summaries)
    by_corner_laps = _group_by_corner(summaries)
    config_recs = load_recommendations_config()

    evidence = []
    evidence += _build_corner_verdict_evidence(aggregated, by_corner_laps, classify_fn, config)
    evidence += _build_ls_disambiguation_evidence(
        aggregated, ls_stats, [e for e in evidence if e["type"] == "corner_verdict"]
    )
    evidence += _build_brake_balance_evidence(aggregated, by_corner_laps, config)
    evidence += _build_matrix_verdict_evidence(aggregated, by_corner_laps, classify_fn, config_recs, config)
    evidence += _build_ls_threshold_evidence(ls_stats, by_corner_laps, config)

    # ABS on by default (trusted position channel); TC off until its channel
    # mapping is done
    intervention_cfg = config.get("intervention_evidence", {})
    if corners is not None:
        abs_cfg = intervention_cfg.get("abs", {})
        if abs_cfg.get("enabled", False):
            evidence += _build_intervention_abs_evidence(corners, by_corner_laps, state, channels, aggregated, abs_cfg)
        tc_cfg = intervention_cfg.get("tc", {})
        if tc_cfg.get("enabled", False):
            evidence += _build_intervention_tc_evidence(corners, state, channels, aggregated)

        # damper motion needs per-lap phase windows. Lazy import: damper_motion
        # imports from this module.
        dm_cfg = config.get("damper_motion", {})
        if dm_cfg.get("enabled", False):
            from modules.damper_motion import build_damper_motion_evidence
            wl_cfg = load_parameters()["wheel_loads"]
            dm_evidence, _dm_summary = build_damper_motion_evidence(
                corners, state, channels, aggregated, dm_cfg, wl_cfg)
            evidence += dm_evidence

        # kerb blowoff, needs per-instance time windows
        kb_cfg = config.get("kerb_blowoff_evidence", {})
        if kb_cfg.get("enabled", False):
            wl_cfg = load_parameters()["wheel_loads"]
            evidence += _build_kerb_blowoff_evidence(corners, state, channels, kb_cfg, wl_cfg)

    if feedback_data:
        evidence += _build_driver_feedback_evidence(
            feedback_data, aggregated, config.get("driver_feedback_weighting", {}))

    return evidence


# Candidate layer. Exit oversteer, both LS branches (cornering-limited ->
# ARB/springs; traction-limited -> diff/TC), plus brake-balance.
# Grade: 'derived-from-matrix' if a real cell_id backs this exact
# parameter+direction+scenario, else 'proposed' (advisory-capped).

EXIT_PHASES = ("exit_4", "exit_5")

# entry/exit = transient phases: dampers only act with shaft velocity,
# not at steady apex (Segers ch. 11). Derived from PHASE_KEYS.
TRANSIENT_PHASES = tuple(p for p in PHASE_KEYS if p != "apex_3")

# ordinal change_effort enum, method-defining (half_hour = camber class)
EFFORT_RANK = {"seconds": 0, "minutes": 1, "half_hour": 2, "garage_hours": 3}

# Every reachable lever resolves to exactly one status. Per candidate
# (corner + phase), never merged across corners; STATUS_NO_TRIGGER is a
# placeholder for a lever with no candidate anywhere.
STATUS_PROPOSED = "proposed"
STATUS_NO_TRIGGER = "no_trigger"
STATUS_BLOCKED_AT_EDGE = "blocked_at_edge"
STATUS_CONTRADICTED = "contradicted"
STATUS_NOT_ASSESSABLE = "not_assessable"
LEVER_STATUSES = (STATUS_PROPOSED, STATUS_NO_TRIGGER, STATUS_BLOCKED_AT_EDGE,
                   STATUS_CONTRADICTED, STATUS_NOT_ASSESSABLE)

# same action-eligible provenances as the rule engine
_MATRIX_ELIGIBLE_PROVENANCES = frozenset({"engineer-verbatim", "project-lead-reviewed"})


def _effort_class_for_actions(param_keys, registry):
    efforts = [registry[p]["change_effort"] for p in param_keys
               if p in registry and registry[p].get("change_effort")]
    if not efforts:
        return None
    return max(efforts, key=lambda e: EFFORT_RANK.get(e, 0))


def _matrix_cell(config_recs, cell_id):
    if cell_id is None:
        return None
    for rule in config_recs["rules"]:
        if rule.get("cell_id") == cell_id:
            return rule
    return None


def _grade_for_provenance(provenance):
    return "derived-from-matrix" if provenance in _MATRIX_ELIGIBLE_PROVENANCES else "proposed"


def _exit_oversteer_candidates(corner_verdicts_by_key, ls_by_key, registry, config_recs,
                                intervention_tc_by_corner=None):
    intervention_tc_by_corner = intervention_tc_by_corner or {}
    candidates = []
    for (cid, phase), items in corner_verdicts_by_key.items():
        if phase not in EXIT_PHASES:
            continue
        cv = next((e for e in items if e["verdict"] == "oversteer"), None)
        if cv is None:
            continue
        speed_class = cv.get("speed_class")
        ls_evidence = ls_by_key.get((cid, phase))
        ls_class = ls_evidence["ls_class"] if ls_evidence else None
        evidence_refs = [cv] + ([ls_evidence] if ls_evidence else [])
        # TC evidence corroborates the traction-limited branch only
        tc_evidence_refs = evidence_refs + (
            [intervention_tc_by_corner[cid]] if cid in intervention_tc_by_corner else []
        )

        if ls_class in (None, "cornering_limited"):
            # ARB/springs. Rear ARB soften is matrix-exact only for OS-EXIT-med;
            # elsewhere the same lever at 'proposed' (LS routing has no speed class).
            cell = _matrix_cell(config_recs, "OS-EXIT-med") if speed_class == "medium" else None
            arb_params = ["arb_rl", "arb_rr"]
            candidates.append({
                "id": f"arb_soften:C{cid}:{phase}",
                "scenario": "exit_oversteer",
                "corner": cid, "phase": phase,
                "lever_family": "arb_spring",
                "actions": [{"parameter": p, "direction": "soften", "delta": -1} for p in arb_params],
                "effort_class": _effort_class_for_actions(arb_params, registry),
                "effect_class": "primary",
                "grade": _grade_for_provenance(cell["elicitation_provenance"]) if cell else "proposed",
                "cell_id": "OS-EXIT-med" if cell else None,
                "evidence_refs": evidence_refs,
                "rationale": (cell["rationale"] if cell else
                              "Cornering-limited (LS disambiguation): softening the rear ARB frees up "
                              "rear grip under lateral load, addressing exit oversteer at the "
                              "roll-stiffness-distribution level. Matrix-exact only at medium-speed "
                              "exit (cell OS-EXIT-med); generalised here across speed classes."),
            })
            candidates.append({
                "id": f"springs_rear_soften:C{cid}:{phase}",
                "scenario": "exit_oversteer",
                "corner": cid, "phase": phase,
                "lever_family": "arb_spring",
                "actions": [{"parameter": "springs_rear", "direction": "soften", "delta": -1}],
                "effort_class": _effort_class_for_actions(["springs_rear"], registry),
                "effect_class": "secondary",
                "grade": "proposed",
                "cell_id": None,
                "evidence_refs": evidence_refs,
                "rationale": "Same rear-grip goal as the ARB candidate above, at the garage-effort "
                             "tier: softer rear springs reduce rear vertical stiffness generally "
                             "(registry mechanism: sets rear-axle ride stiffness independent of roll "
                             "stiffness). Not itself a matrix cell for this scenario -- proposed "
                             "grade, advisory-capped, a heavier-effort alternative to the ARB lever, "
                             "not a substitute recommendation on equal footing. Companion note "
                             "(author-elicited 2026-09-24, WP-ELICIT Phase B1): softer rear springs "
                             "and stiffer rear dampers are compensatory, same axle -- if the springs "
                             "move, the rear dampers may need a matching stiffen to hold the platform "
                             "where the springs alone would let it move. Informational only, not a "
                             "competing candidate; no separate damper action is generated from this "
                             "note.",
            })

        if ls_class in (None, "traction_limited"):
            # diff/TC. TC LON matrix-exact only at OS-EXIT-low; diff has no exit
            # oversteer cell -> 'proposed', advisory-capped.
            cell = _matrix_cell(config_recs, "OS-EXIT-low") if speed_class == "low" else None
            candidates.append({
                "id": f"tc_lon_increase:C{cid}:{phase}",
                "scenario": "exit_oversteer",
                "corner": cid, "phase": phase,
                "lever_family": "diff_tc",
                "actions": [{"parameter": "tc_lon", "direction": "increase", "delta": 1}],
                "effort_class": _effort_class_for_actions(["tc_lon"], registry),
                "effect_class": "primary",
                "grade": _grade_for_provenance(cell["elicitation_provenance"]) if cell else "proposed",
                "cell_id": "OS-EXIT-low" if cell else None,
                "evidence_refs": tc_evidence_refs,
                "rationale": (cell["rationale"] if cell else
                              "Traction-limited (LS disambiguation): raising TC LON cuts wheel spin "
                              "under power, addressing power-on rear-axle oversteer directly. "
                              "Matrix-exact only at low-speed exit (cell OS-EXIT-low); generalised "
                              "here across speed classes for the same LS-routing reason as the ARB "
                              "candidate above."),
            })
            candidates.append({
                "id": f"diff_position_increase:C{cid}:{phase}",
                "scenario": "exit_oversteer",
                "corner": cid, "phase": phase,
                "lever_family": "diff_tc",
                "actions": [{"parameter": "diff_position", "direction": "increase", "delta": 1}],
                "effort_class": _effort_class_for_actions(["diff_position"], registry),
                "effect_class": "secondary",
                "grade": "proposed",
                "cell_id": None,
                "evidence_refs": tc_evidence_refs,
                "rationale": "More locking torque resists axle-speed difference under power, "
                             "stabilising the rear on corner exit -- the same mechanism the matrix "
                             "already uses for turn-in/apex oversteer (cells OS-TIN-med, OS-APX-med), "
                             "generalised here to exit. No matrix cell exists for diff_position at "
                             "exit specifically -- PROPOSED grade, advisory-capped, per the work "
                             "order's own instruction.",
            })
    return candidates


def _brake_balance_candidates(evidence, registry, config_recs):
    # reuses the US-BRK-{speed_class} cell's suggestion/rationale -- a cheaper
    # detection path for the same problem, not a new lever
    speed_class_to_cell = {"low": "US-BRK-low", "medium": "US-BRK-med", "high": "US-BRK-high"}
    candidates = []
    for e in evidence:
        if e["type"] != "plausibility_brake_balance":
            continue
        cell_id = speed_class_to_cell.get(e.get("speed_class"))
        cell = _matrix_cell(config_recs, cell_id)
        if cell is None:
            candidates.append({
                "id": f"brake_balance_unrouted:C{e['corner']}:{e['phase']}",
                "scenario": "plausibility_brake_balance",
                "corner": e["corner"], "phase": e["phase"],
                "lever_family": None, "actions": [],
                "effort_class": None, "effect_class": None,
                "grade": "proposed", "cell_id": None,
                "evidence_refs": [e],
                "rationale": f"Front-beyond/rear-healthy brake-balance signature fired at "
                             f"C{e['corner']} {e['phase']}, but speed_class "
                             f"({e.get('speed_class')!r}) does not map to a known US-BRK-* cell -- "
                             f"no candidate action generated, engineer attention needed.",
            })
            continue
        actions = cell["suggestion"] if isinstance(cell["suggestion"], list) else [cell["suggestion"]]
        param_keys = [a["parameter"] for a in actions]
        candidates.append({
            "id": f"brake_balance:{cell_id}:C{e['corner']}:{e['phase']}",
            "scenario": "plausibility_brake_balance",
            "corner": e["corner"], "phase": e["phase"],
            "lever_family": "brake_balance",
            "actions": actions,
            "effort_class": _effort_class_for_actions(param_keys, registry),
            "effect_class": "primary",
            "grade": _grade_for_provenance(cell["elicitation_provenance"]),
            "cell_id": cell_id,
            "evidence_refs": [e],
            "rationale": cell["rationale"] + " (Reached here via the brake_balance_signature "
                         "plausibility check, not the full CS-verdict classify path -- same "
                         f"underlying matrix cell and lever as the existing {cell_id} rule in "
                         "config/recommendations.json.)",
        })
    return candidates


# brake_bias bridge (Segers ch. 5): too much front bias uses up front grip
# needed for turn-in -> entry/mid understeer -> more_rear. Too much rear
# bias -> rear steps out on trail braking, entry only -> more_front.
# Bias moves away from the limiting axle.
# Output = direction word + click count, never a channel delta (the
# channel's sign/scale isn't elicited yet).
# Clicks = SEVERITY_RANK (moderate 1, strong 2); 3 is a cap, not a target.

def _brake_bias_candidates(evidence):
    direction_by_verdict = {
        "understeer": ("more_rear", [("entry_1_brake",), ("entry_2_turnin",)]),
        "oversteer": ("more_front", [("entry_1_brake",)]),
    }
    candidates = []
    for e in evidence:
        if e["type"] != "matrix_verdict":
            continue
        spec = direction_by_verdict.get(e["verdict"])
        if spec is None:
            continue
        direction, allowed_phase_groups = spec
        if e["phases"] not in allowed_phase_groups:
            continue
        if SEVERITY_RANK[e["severity"]] < SEVERITY_RANK["moderate"]:
            continue
        clicks = min(3, SEVERITY_RANK[e["severity"]])
        if direction == "more_rear":
            mechanism = ("Too much front bias uses up front-tyre grip capacity under "
                         "straight-line braking, unavailable for turn-in cornering force "
                         "(understeer) -- move REARWARD to free up front grip.")
        else:
            mechanism = ("Too much rear bias risks the rear stepping out under "
                         "trail-braking (oversteer) -- move FORWARD to stabilise the rear.")
        candidates.append({
            "id": f"brake_bias:{direction}:C{e['corner']}:{'+'.join(e['phases'])}",
            "scenario": f"brake_bias:{direction}",
            "corner": e["corner"], "phase": e["phases"][-1], "phases": e["phases"],
            "lever_family": "brake_bias",
            "actions": [{"parameter": "brake_bias", "direction": direction, "delta": clicks}],
            "effort_class": "seconds",
            "effect_class": "secondary",
            "grade": "proposed",
            "cell_id": None,
            "evidence_refs": [e],
            "derived_from": "Segers ch.5 p.107 Eq.5.3 (docs/segers_bridge_review.md C5-1), "
                             "reviewer-confirmed direction convention 2026-09-22",
            "rationale": mechanism + f" Severity-scaled {clicks} click(s) (max 3). Direction is "
                         "an output word only -- the channel's own sign/scale and current-state "
                         "window are elicitation item 4, not resolved here.",
        })
    return candidates


# Stage 2: every recommendations.json rule as a candidate bridge --
# matrix_verdict evidence -> the rule's suggestion; grading via
# _grade_for_provenance. Status -> behaviour, same as the rule engine:
#   elicited/reviewed: primary candidate on matching evidence
#   held: never primary; emitted as secondary alongside its base cell's
#     candidate, always 'proposed' (not automated, no history)
#   dropped/retired: accounted for, no candidate
#   trigger != "data": out of scope (all such rules retired), defensive

def rule_bridge_status(rule):
    """'primary' | 'secondary(held)' | 'inactive(dropped)' | 'inactive(retired)'
    | 'inactive(other-status)' | 'non-matrix(trigger)' -- also used by the
    migration-completeness test.
    """
    status = rule.get("status")
    if status == "retired":
        return "inactive(retired)"
    if status == "dropped":
        return "inactive(dropped)"
    if status == "held":
        return "secondary(held)"
    if rule["condition"].get("trigger") != "data" or rule.get("suggestion") is None:
        return "non-matrix(trigger)"
    if status in _NON_FIRING_STATUSES:
        return "inactive(other-status)"
    return "primary"


def _bridge_candidates_for_matrix_rules(evidence, registry, config_recs, intervention_abs_by_corner=None,
                                         intervention_abs_masking_by_corner=None):
    intervention_abs_by_corner = intervention_abs_by_corner or {}
    intervention_abs_masking_by_corner = intervention_abs_masking_by_corner or {}
    matrix_by_group = {}
    for e in evidence:
        if e["type"] == "matrix_verdict":
            matrix_by_group.setdefault(e["phases"], {}).setdefault(e["corner"], []).append(e)

    held_by_base = {r["escalation_of"]: r for r in config_recs["rules"]
                     if r.get("status") == "held" and r.get("escalation_of")}

    def _make_candidate(rule, cid, evidence_refs, effect_class, grade, phase_group):
        actions = rule["suggestion"] if isinstance(rule["suggestion"], list) else [rule["suggestion"]]
        param_keys = [a["parameter"] for a in actions]
        return {
            "id": f"{rule['id']}:C{cid}:{'+'.join(phase_group)}",
            "scenario": rule.get("cell_id") or rule["id"],
            "corner": cid, "phase": phase_group[-1], "phases": phase_group,
            "lever_family": f"matrix:{rule.get('cell_id') or rule['id']}",
            "actions": actions,
            "effort_class": _effort_class_for_actions(param_keys, registry),
            "effect_class": effect_class,
            "grade": grade,
            "cell_id": rule.get("cell_id"),
            "evidence_refs": evidence_refs,
            "rationale": rule["rationale"],
            "practice_note": rule.get("practice_note"),
            "rule_id": rule["id"], "rule_status": rule.get("status"),
        }

    candidates = []
    for rule in config_recs["rules"]:
        if rule_bridge_status(rule) != "primary":
            continue
        condition = rule["condition"]
        phase_group = tuple(rule["phases"])
        verdict = condition["verdict"]
        min_sev = condition.get("min_severity", "normal")
        req_speed_class = condition.get("speed_class")

        for cid, items in matrix_by_group.get(phase_group, {}).items():
            for ev in items:
                if ev["verdict"] != verdict:
                    continue
                if SEVERITY_RANK[ev["severity"]] < SEVERITY_RANK[min_sev]:
                    continue
                if req_speed_class is not None and ev.get("speed_class") != req_speed_class:
                    continue

                evidence_refs = [ev]
                if verdict == "unstable_yaw" and phase_group == ("entry_1_brake",) and cid in intervention_abs_by_corner:
                    evidence_refs = evidence_refs + [intervention_abs_by_corner[cid]]
                # ABS heavily regulating -> masking flag on any braking-phase verdict.
                # Added to evidence_refs so min() confidence pulls this candidate down and
                # a brake-balance candidate at the same corner can outrank it. Partial
                # realisation of "prefer" -- no hard reordering.
                if phase_group == ("entry_1_brake",) and cid in intervention_abs_masking_by_corner:
                    evidence_refs = evidence_refs + [intervention_abs_masking_by_corner[cid]]

                candidates.append(_make_candidate(
                    rule, cid, evidence_refs, "primary",
                    _grade_for_provenance(rule.get("elicitation_provenance")), phase_group,
                ))

                held = held_by_base.get(rule.get("cell_id"))
                if held is not None and held.get("suggestion") is not None:
                    candidates.append(_make_candidate(
                        held, cid, evidence_refs, "secondary", "proposed", phase_group,
                    ))
    return candidates


# Per-bridge condition evaluator. Schema: decision_frame.json
# "conditions". No repeat-count condition -- every verdict item on both
# sessions rests on one repeating lap; repeatability stays a confidence
# discount.

def _eval_evidence_corroboration(cond, corner, phase, evidence_items):
    """PASS/FAIL/not-evaluable, same corner and phase. Source never built this
    run (gated off, inputs missing) -> not-evaluable; source ran but has
    nothing here -> real FAIL.
    """
    evidence_type = cond["evidence_type"]
    presence = cond["presence"]
    if not any(e["type"] == evidence_type for e in evidence_items):
        return "not_evaluable", f"no {evidence_type} evidence available this run"

    def _matches(e):
        if e["type"] != evidence_type or e.get("corner") != corner:
            return False
        return phase == e.get("phase") or phase in (e.get("phases") or ())

    found = any(_matches(e) for e in evidence_items)
    if presence == "present":
        return (True, None) if found else (False, f"no {evidence_type} evidence at C{corner} {phase}")
    return (True, None) if not found else (False, f"{evidence_type} evidence present at C{corner} {phase} (expected absent)")


def _eval_setup_state(cond, setup_data, registry):
    """PASS/FAIL/not-evaluable. Nominal/span from decision_frame parameter_windows
    (same windows as _settings_window_component) -- not re-derived from the
    registry's mixed window formats. registry only resolves maps_to.
    """
    param = cond["parameter"]
    check = cond["check"]
    entry = registry.get(param)
    if entry is None:
        return "not_evaluable", f"no registry entry: {param}"

    window = load_decision_frame_config()["parameter_windows"].get(param, {})
    nominal, span = window.get("nominal"), window.get("span")
    if nominal is None or span is None or not span:
        return "not_evaluable", f"no settings window: {param}"

    current = _current_setup_value(setup_data, entry)
    if current is None:
        return "not_evaluable", f"setup sheet unfilled: {param}"
    try:
        current = float(current)
    except (TypeError, ValueError):
        return "not_evaluable", f"setup sheet value non-numeric (likely an enum label): {param}"

    if check == "within_window":
        ok = abs(current - nominal) <= span
        return (True, None) if ok else (False, f"{param} outside window (current={current}, nominal={nominal}, span={span})")
    if check == "at_window_edge":
        ok = abs(current - nominal) >= span
        return (True, None) if ok else (False, f"{param} not at window edge (current={current}, nominal={nominal}, span={span})")
    if check == "moved_from_nominal":
        direction = cond.get("direction")
        if direction == "increase":
            ok = current > nominal
        elif direction == "decrease":
            ok = current < nominal
        else:
            return "not_evaluable", f"moved_from_nominal requires a direction: {param}"
        return (True, None) if ok else (False, f"{param} not moved {direction} from nominal (current={current}, nominal={nominal})")
    return "not_evaluable", f"unknown setup_state check: {check!r}"


def _eval_phase_transient(phase):
    ok = phase in TRANSIENT_PHASES
    return (True, None) if ok else (False, f"{phase} is not a transient phase (apex/steady-state)")


def evaluate_conditions(conditions, corner, phase, evidence_items, setup_data, registry,
                         contradiction_sources=None):
    """Returns (verdict, reasons):
      PASS: all evaluable and passing
      CONTRADICTED: a required evidence_corroboration condition fails and
        its type is in contradiction_sources -- data contradicts data
      SUPPRESS: any other required condition fails (not emitted)
      CAP_ADVISORY: a non-required condition fails or anything is
        not-evaluable; reasons = human-readable gaps
    An evidence gap caps, never suppresses: "can't corroborate" !=
    "contradicted". No conditions -> PASS, []. contradiction_sources None ->
    treated as empty.
    """
    if not conditions:
        return "PASS", []
    contradiction_sources = contradiction_sources or ()

    reasons = []
    degraded = False
    for cond in conditions:
        ctype = cond["type"]
        required = cond.get("required", False)

        if ctype == "evidence_corroboration":
            result, reason = _eval_evidence_corroboration(cond, corner, phase, evidence_items)
        elif ctype == "setup_state":
            result, reason = _eval_setup_state(cond, setup_data, registry)
        elif ctype == "phase_transient":
            result, reason = _eval_phase_transient(phase)
        else:
            result, reason = "not_evaluable", f"unknown condition type: {ctype!r}"

        if result == "not_evaluable":
            degraded = True
            reasons.append(reason)
            continue
        if result is False:
            if required:
                if ctype == "evidence_corroboration" and cond.get("evidence_type") in contradiction_sources:
                    return "CONTRADICTED", [reason]
                return "SUPPRESS", [reason]
            degraded = True
            reasons.append(reason)

    return ("CAP_ADVISORY", reasons) if degraded else ("PASS", [])


# Generic lever bridges (decision_frame.json lever_bridges): one row per
# (lever, direction), fires over its own list of phase groups. Same
# verdict/min_severity gate as the matrix bridge.

def _bridge_candidates_for_levers(evidence, registry, decision_config, existing_candidates, setup_data=None):
    """Plumbing only -- the physics (Segers ch. 9/10) sits in interaction_table.
    Dedupe: a key already emitted by the exit-oversteer or matrix bridge
    wins (earlier, richer evidence); skipped here, never re-scored.
    Optional per-bridge "conditions" via evaluate_conditions: PASS -> emit,
    SUPPRESS -> skip, CAP_ADVISORY -> emit with a confidence-capping ref +
    condition_reasons. setup_data feeds setup_state conditions.
    """
    cap = decision_config.get("conditions", {}).get("not_evaluable_confidence_cap", 1.0)
    bridges = decision_config.get("lever_bridges", [])
    if not bridges:
        return []

    existing_keys = {
        (action["parameter"], action["direction"], c["corner"])
        for c in existing_candidates for action in c["actions"]
    }

    matrix_by_group = {}
    for e in evidence:
        if e["type"] == "matrix_verdict":
            matrix_by_group.setdefault(e["phases"], {}).setdefault(e["corner"], []).append(e)

    candidates = []
    for bridge in bridges:
        param = bridge["lever"]
        direction = bridge["direction"]
        condition = bridge["condition"]
        verdict = condition["verdict"]
        min_sev = condition.get("min_severity", "moderate")
        # optional speed_class filter (splitter bridges, like wing_position's
        # high-speed cells); absent = no filter
        required_speed_class = condition.get("speed_class")
        # +1/-1 = one step in the lever's direction_semantics, a sign not a
        # magnitude (_DIRECTION_SIGN covers stiffen/soften and increase/decrease)
        delta = _DIRECTION_SIGN.get(direction, 1)

        for phase_group in bridge["phase_groups"]:
            phase_group = tuple(phase_group)
            for cid, items in matrix_by_group.get(phase_group, {}).items():
                key = (param, direction, cid)
                if key in existing_keys:
                    continue
                for ev in items:
                    if ev["verdict"] != verdict:
                        continue
                    if SEVERITY_RANK[ev["severity"]] < SEVERITY_RANK[min_sev]:
                        continue
                    if required_speed_class is not None and ev.get("speed_class") != required_speed_class:
                        continue

                    contradiction_sources = decision_config.get("conditions", {}).get(
                        "contradiction_sources", [])
                    verdict_result, reasons = evaluate_conditions(
                        bridge.get("conditions", []), cid, phase_group[-1], evidence, setup_data, registry,
                        contradiction_sources)
                    if verdict_result == "SUPPRESS":
                        continue

                    evidence_refs = [ev]
                    status = STATUS_PROPOSED
                    if verdict_result == "CAP_ADVISORY":
                        evidence_refs = evidence_refs + [{
                            "type": "condition_gap", "corner": cid, "phase": phase_group[-1],
                            "verdict": None, "severity": None, "confidence": cap,
                            "source": f"condition gap, capped at {cap}: " + "; ".join(reasons),
                        }]
                        # unfilled sheet / missing source -> not assessable, visible in the inventory
                        status = STATUS_NOT_ASSESSABLE
                    elif verdict_result == "CONTRADICTED":
                        # data contradicts data -> out of the shortlist, kept in the tail with
                        # reason; confidence left as is
                        status = STATUS_CONTRADICTED
                        reasons = [f"contradicted by {reasons[0]}"] if reasons else ["contradicted"]

                    candidates.append({
                        "id": f"lever_bridge:{param}:{direction}:C{cid}:{'+'.join(phase_group)}",
                        "scenario": f"lever_bridge:{param}:{direction}",
                        "corner": cid, "phase": phase_group[-1], "phases": phase_group,
                        "lever_family": bridge.get("lever_family", param),
                        "actions": [{"parameter": param, "direction": direction, "delta": delta}],
                        "effort_class": _effort_class_for_actions([param], registry),
                        "effect_class": bridge.get("effect_class", "secondary"),
                        "grade": "proposed",  # structural cap
                        "cell_id": None,
                        "evidence_refs": evidence_refs,
                        "rationale": bridge["rationale"],
                        "derived_from": bridge["derived_from"],
                        "condition_reasons": reasons,
                        "status": status,
                    })
    return candidates


# Feedback-only trigger: |feedback| >= 2 -> candidate with no data verdict.
# Routed via interaction_table's signed (parameter, direction, axis)
# entries, click-class levers only. Relaxing matrix severity floors was
# rejected -- they're part of what the engineer set. |1| = note only.
# trigger_provenance: other generators data_only, this one feedback_only,
# data_only + matching feedback -> both_agreeing (strongest).
TRIGGER_DATA_ONLY = "data_only"
TRIGGER_FEEDBACK_ONLY = "driver_reported"
TRIGGER_BOTH_AGREEING = "both_agreeing"

# symbolic one-step sign, same as the lever bridges; matches the rule
# table's sign convention for these direction words
_DIRECTION_SIGN = {
    "stiffen": 1, "soften": -1,
    "increase": 1, "decrease": -1,
    "more_negative": -1, "less_negative": 1,
    "more_positive": 1, "less_positive": -1,
}


def _phase_compatible(phase, phase_affinity):
    # None = global lever, any phase. wing_position's affinity is a speed-class
    # tag, never a phase -> never matches feedback here.
    return phase_affinity is None or phase in phase_affinity


_AXLE_PAIR_SUFFIXES = {"_fl": "_fr", "_fr": "_fl", "_rl": "_rr", "_rr": "_rl"}


_AXLE_PAIRED_PREFIXES = ("damper_", "arb_")


def _axle_partner_param(param):
    # Dampers and ARBs axle-paired: fl+fr (rl+rr) is one move, never a
    # choice. A real front-vs-rear cross-family tie is handled below.
    if not param.startswith(_AXLE_PAIRED_PREFIXES):
        return None
    for suffix, partner_suffix in _AXLE_PAIR_SUFFIXES.items():
        if param.endswith(suffix):
            return param[: -len(suffix)] + partner_suffix
    return None


def _group_axle_pairs(pool):
    # same-direction axle pairs -> one grouped item, so families compete, not
    # fl vs fr. Partner filtered out earlier -> singleton.
    by_key = {(p, d): (p, d, r, e) for p, d, r, e in pool}
    grouped, seen = [], set()
    for param, direction, reg_entry, entry in pool:
        if (param, direction) in seen:
            continue
        partner = _axle_partner_param(param)
        partner_item = by_key.get((partner, direction)) if partner else None
        if partner_item is not None:
            seen.add((param, direction))
            seen.add((partner, direction))
            grouped.append({
                "params": tuple(sorted((param, partner))),
                "direction": direction,
                "reg_entries": {param: reg_entry, partner: partner_item[2]},
                "entry": entry,
            })
        else:
            seen.add((param, direction))
            grouped.append({
                "params": (param,), "direction": direction,
                "reg_entries": {param: reg_entry}, "entry": entry,
            })
    return grouped


def _feedback_only_candidates(evidence, existing_candidates, registry, config, current_setup=None):
    feedback_items = [e for e in evidence if e["type"] == "driver_feedback" and abs(e["raw_feedback"]) >= 2]
    if not feedback_items:
        return []

    click_class = set(config.get("eligibility_classes", {}).get("click_class", []))
    table = config["interaction_table"]
    existing_keys = {
        (action["parameter"], action.get("direction"), c["corner"])
        for c in existing_candidates for action in c["actions"]
    }

    candidates = []
    for fb in feedback_items:
        axis = "oversteer_tendency" if fb["verdict"] == "oversteer" else "understeer_tendency"
        pool = []
        for entry in table:
            if entry["performance_axis"] != axis or entry["sign"] != 1:
                continue
            param = entry["parameter"]
            if param not in click_class:
                continue
            reg_entry = registry.get(param)
            if reg_entry is None:
                continue
            if (param, entry["direction"], fb["corner"]) in existing_keys:
                continue
            if not _phase_compatible(fb["phase"], reg_entry.get("phase_affinity")):
                continue
            pool.append((param, entry["direction"], reg_entry, entry))
        if not pool:
            continue  # honest gap, no fallback

        grouped_pool = _group_axle_pairs(pool)

        def _effort_rank(item):
            efforts = [item["reg_entries"][p].get("change_effort") for p in item["params"]]
            return max(EFFORT_RANK.get(e, len(EFFORT_RANK)) for e in efforts)

        min_rank = min(_effort_rank(item) for item in grouped_pool)
        cheapest = [item for item in grouped_pool if _effort_rank(item) == min_rank]

        if len(cheapest) > 1:
            # tie-break 1: window headroom (_settings_window_component score) before
            # interaction penalty -- only if every tied item is computable, never
            # half-trusted on a partly filled sheet
            def _headroom_score(item):
                probe_actions = [{"parameter": p, "direction": item["direction"],
                                   "delta": _DIRECTION_SIGN.get(item["direction"], 1)}
                                  for p in item["params"]]
                score, flags = _settings_window_component(
                    {"actions": probe_actions}, current_setup, registry, config, 1.0)
                return score, len(flags) < len(probe_actions)
            headroom = [_headroom_score(item) for item in cheapest]
            if all(computable for _, computable in headroom):
                max_headroom = max(score for score, _ in headroom)
                cheapest = [item for item, (score, _) in zip(cheapest, headroom) if score == max_headroom]

        if len(cheapest) > 1:
            def _penalty_magnitude(item):
                probe = {"corner": fb["corner"], "evidence_refs": [fb],
                         "actions": [{"parameter": p, "direction": item["direction"]} for p in item["params"]]}
                penalty, _notes = _interaction_penalty(probe, evidence, config,
                                                        config["cost_function"]["interaction"])
                return abs(penalty)
            max_pen = max(_penalty_magnitude(item) for item in cheapest)
            cheapest = [item for item in cheapest if _penalty_magnitude(item) == max_pen]

        # Remaining cross-family tie (equal cost, headroom, penalty) is normal for
        # isolated feedback (soften one axle / stiffen the other) -> emit both as
        # ordinary alternatives. Not arbitrary, not an error.
        tie_labels = ["+".join(item["params"]) + " " + item["direction"] for item in cheapest]

        for chosen, own_label in zip(cheapest, tie_labels):
            params, direction, entry = chosen["params"], chosen["direction"], chosen["entry"]
            delta = _DIRECTION_SIGN.get(direction, 1)
            param_label = "+".join(params)
            paired_note = " (axle-paired action, Stage 1 design principle)" if len(params) > 1 else ""
            tie_note = ""
            if len(cheapest) > 1:
                others = [label for label in tie_labels if label != own_label]
                tie_note = (f" ALTERNATIVE: genuinely tied with {', '.join(others)} -- cost, window "
                            f"headroom and interaction penalty all agree; no evidence here breaks it, "
                            f"engineer's call.")
            candidates.append({
                "id": f"feedback_only:{param_label}:{direction}:C{fb['corner']}:{fb['phase']}",
                "scenario": f"feedback_only:{param_label}:{direction}",
                "corner": fb["corner"], "phase": fb["phase"],
                "lever_family": param_label,
                "actions": [{"parameter": p, "direction": direction, "delta": delta} for p in params],
                "effort_class": _effort_class_for_actions(list(params), registry),
                "effect_class": "secondary",
                "grade": entry["grade"],
                "cell_id": None,
                "evidence_refs": [fb],
                "rationale": f"Driver reported {fb['verdict']} at {fb['phase']} "
                             f"(feedback {fb['raw_feedback']:+g}) -- {param_label} {direction} routed via "
                             f"interaction_table's own {axis} entry (grade={entry['grade']}), "
                             f"cheapest click-class lever available{paired_note}.{tie_note}",
                "status": STATUS_PROPOSED,
                "trigger_provenance": TRIGGER_FEEDBACK_ONLY,
            })
    return candidates


# Eligibility gate: mild / single-corner problems -> click-class levers
# only; springs/camber/toe need strong + multi-corner, or |feedback| >= 4
# (never for camber). Stricter than the matrix floor (moderate) on purpose.
# "Same axle" matters only for camber (per-corner keys); springs and toe
# are axle-level already.
_CAMBER_AXLE_FAMILY = {
    "camber_fl": "camber_front", "camber_fr": "camber_front",
    "camber_rl": "camber_rear", "camber_rr": "camber_rear",
}


def _axle_family(parameter):
    return _CAMBER_AXLE_FAMILY.get(parameter, parameter)


def _apply_eligibility_gate(candidates, evidence, config, driving_level=None):
    heavy = set(config.get("eligibility_classes", {}).get("heavy_correctors", []))
    if not heavy:
        return candidates

    # driving_level (1-10 or None, from the UI) -> trust arbitration on the
    # multi-corner data path, not a new unlock route
    driver_level_threshold = config.get("eligibility_classes", {}).get("driver_level_threshold", 5)
    # unknown level = neutral (>= reaches it) -> never unlocks the heavier move
    effective_driving_level = driving_level if driving_level is not None else driver_level_threshold
    feedback_cfg = config.get("driver_feedback_weighting", {})
    _confidence_bands = feedback_cfg.get("confidence_bands", [])
    # "moderate |fb|" = driver_feedback_weighting's mid band, not a second 2/3
    _moderate_lo = _confidence_bands[0]["max_abs"] if _confidence_bands else 1
    _moderate_hi = _confidence_bands[1]["max_abs"] if len(_confidence_bands) > 1 else 3

    # |feedback| >= 4 path matches per (corner, verdict), not phase -- drivers
    # don't split complaints by phase. The data path stays phase-scoped.
    feedback_magnitude_by_corner_verdict = {}
    for e in evidence:
        if e["type"] == "driver_feedback":
            key = (e["corner"], e["verdict"])
            feedback_magnitude_by_corner_verdict[key] = max(
                feedback_magnitude_by_corner_verdict.get(key, 0), abs(e["raw_feedback"])
            )

    def _is_heavy(c):
        return any(a["parameter"] in heavy for a in c["actions"])

    heavy_candidates = [c for c in candidates if _is_heavy(c)]
    if not heavy_candidates:
        return candidates
    other_candidates = [c for c in candidates if not _is_heavy(c)]

    # (axle_family, direction) -> corners with STRONG severity (read like score())
    strong_corners_by_group = {}
    for c in heavy_candidates:
        primary = c["evidence_refs"][0] if c["evidence_refs"] else None
        severity = primary.get("severity") if primary else None
        if severity != "strong":
            continue
        for a in c["actions"]:
            if a["parameter"] not in heavy:
                continue
            group = (_axle_family(a["parameter"]), a["direction"])
            strong_corners_by_group.setdefault(group, set()).add(c["corner"])

    kept = []
    for c in heavy_candidates:
        eligible = False
        primary = c["evidence_refs"][0] if c["evidence_refs"] else None
        candidate_verdict = primary.get("verdict") if primary else None
        fb_mag = feedback_magnitude_by_corner_verdict.get((c["corner"], candidate_verdict), 0)
        moderate_feedback = _moderate_lo < fb_mag <= _moderate_hi
        for a in c["actions"]:
            if a["parameter"] not in heavy:
                continue
            group = (_axle_family(a["parameter"]), a["direction"])
            multi_corner_ok = len(strong_corners_by_group.get(group, set())) >= 2
            # veto: moderate rating from an at/above-neutral driver at this
            # corner/verdict = "manageable" -> hold back the multi-corner unlock.
            # Tightening only; camber's multi-corner floor still applies.
            if multi_corner_ok and moderate_feedback and effective_driving_level >= driver_level_threshold:
                multi_corner_ok = False
            # camber: no feedback bypass, always multi-corner
            feedback_ok = fb_mag >= 4 and not a["parameter"].startswith("camber_")
            if multi_corner_ok or feedback_ok:
                eligible = True
                break
        if eligible:
            kept.append(c)
    return other_candidates + kept


# Breadth: a global lever helping 1 of N corners is penalised ("helps CX,
# risks others"); dampers exempt (per-corner capable via LS/HS split).
# N = every corner assessed, normal ones included -- a normal corner is
# evidence of a working state. Not in the evidence list (normal verdicts
# are skipped) -> assessed_corner_ids parameter.

def _attach_breadth(candidates, assessed_corner_ids):
    by_action = {}
    for c in candidates:
        for a in c["actions"]:
            by_action.setdefault((a["parameter"], a.get("direction")), set()).add(c["corner"])

    if not assessed_corner_ids:
        # no census -> breadth fields null, no invented N
        for c in candidates:
            c["corners_helped"] = None
            c["corners_touched"] = None
            c["breadth_note"] = None
        return candidates

    assessed = set(assessed_corner_ids)
    for c in candidates:
        exempt = any(a["parameter"].startswith("damper_") for a in c["actions"])
        helped = set()
        opposed = set()
        for a in c["actions"]:
            param, direction = a["parameter"], a.get("direction")
            helped |= by_action.get((param, direction), set())
            for (other_param, other_direction), corners in by_action.items():
                if other_param == param and other_direction != direction:
                    opposed |= corners
        c["corners_helped"] = sorted(helped)
        if exempt:
            c["corners_touched"] = sorted(helped)
            c["breadth_note"] = None
            continue

        c["corners_touched"] = sorted(assessed)
        helped_in_assessed = helped & assessed
        rebalanced = sorted(assessed - helped_in_assessed)
        notes = []
        if rebalanced:
            helped_str = "/".join(str(x) for x in sorted(helped_in_assessed)) or "none assessed"
            notes.append(
                f"helps C{helped_str} -- rebalances {len(rebalanced)} corner(s) "
                f"currently assessed good (C{'/'.join(str(x) for x in rebalanced)})"
            )
        opposed_only = sorted(opposed - helped)
        if opposed_only:
            notes.append(f"directly opposed at C{'/'.join(str(x) for x in opposed_only)}")
        c["breadth_note"] = "; ".join(notes) if notes else None
    return candidates


def _attach_tc_safety_note(candidates, config):
    # lower TC intervention trades safety margin for rotation -> display-only
    # caution, never scored or suppressed. Text and pairs from config
    # tc_safety_note.
    tc_cfg = config.get("tc_safety_note")
    if not tc_cfg:
        for c in candidates:
            c["tc_safety_note"] = None
        return candidates
    flagged = {(p, tc_cfg["direction"]) for p in tc_cfg["parameters"]}
    for c in candidates:
        c["tc_safety_note"] = tc_cfg["text"] if any(
            (a["parameter"], a.get("direction")) in flagged for a in c["actions"]
        ) else None
    return candidates


# Window edge -> blocked_at_edge: shown at its rank with reason, never
# suppressed, alternatives rank up on merit. Only when setup_data given.
# HARD edge = registry value_space min/max (physical); SOFT = parameter
# window nominal +/- span ("engineer may exceed").
# Directional: blocks only if the delta pushes further past the edge.
# Missing value for this lever -> not assessable (per lever).

def _window_edge_check(action, setup_data, registry, config):
    param = action["parameter"]
    delta = action.get("delta")
    if delta is None or delta == 0:
        return None  # target action, no push
    entry = registry.get(param)
    if entry is None:
        return None
    current = _current_setup_value(setup_data, entry)
    if current is None:
        return ("not_assessable", f"setup sheet unfilled: {param}")
    try:
        current = float(current)
    except (TypeError, ValueError):
        return None  # enum label, not numeric

    push_sign = 1 if delta > 0 else -1

    value_space = entry.get("value_space") or {}
    vmin, vmax = value_space.get("min"), value_space.get("max")
    if push_sign > 0 and vmax is not None and current >= vmax:
        return ("hard", f"{param} already at its hard maximum ({current} >= {vmax})")
    if push_sign < 0 and vmin is not None and current <= vmin:
        return ("hard", f"{param} already at its hard minimum ({current} <= {vmin})")

    window = config["parameter_windows"].get(param, {})
    nominal, span = window.get("nominal"), window.get("span")
    if nominal is None or span is None or not span:
        return None  # no window data -> skip, as in scoring
    distance = current - nominal
    if (push_sign > 0 and distance >= span) or (push_sign < 0 and distance <= -span):
        return ("soft", f"{param} already at its typical-window edge "
                         f"(current={current}, nominal={nominal}, span={span})")
    return None


def _edge_normal_state_label(action, config):
    # edge_normal_state_params: an edge that is the lever's normal resting
    # state is annotated, not blocked
    for entry in config.get("edge_normal_state_params", []):
        if entry["parameter"] == action["parameter"] and entry["direction"] == action.get("direction"):
            return entry["label"]
    return None


def _apply_window_edge_status(candidates, setup_data, registry, config):
    if setup_data is None:
        return candidates
    for c in candidates:
        if c.get("status") != STATUS_PROPOSED:
            continue  # not_assessable takes precedence
        for action in c["actions"]:
            result = _window_edge_check(action, setup_data, registry, config)
            if result is None:
                continue
            kind, reason = result
            normal_label = _edge_normal_state_label(action, config) if kind in ("hard", "soft") else None
            if normal_label is not None:
                # exempt: annotate, keep checking the other actions
                c["edge_normal_state_note"] = f"{action['parameter']}: {normal_label} ({reason})"
                continue
            if kind == "not_assessable":
                c["status"] = STATUS_NOT_ASSESSABLE
                c["edge_reason"] = reason
            else:
                c["status"] = STATUS_BLOCKED_AT_EDGE
                c["edge_kind"] = kind
                c["edge_reason"] = reason
                c["edge_label"] = "engineer may exceed" if kind == "soft" else "hard limit"
            break  # first blocking action decides
    return candidates


def generate_candidates(evidence, registry, config, setup_data=None, assessed_corner_ids=None,
                         driving_level=None):
    """Candidate layer: exit-oversteer bridge, matrix-rule bridges, lever
    bridges, brake_bias, kerb blowoff; then eligibility gate, feedback-only
    candidates, feedback attach, conflicting feedback, window edges,
    breadth, TC caution.
    registry = load_setup_parameters_registry(); config =
    load_decision_frame_config() (lever_bridges live there).
    setup_data: setup sheet, for conditions and window edges.
    assessed_corner_ids: every assessed corner, for breadth (None -> null).
    driving_level: 1-10 or None, for the eligibility veto.
    """
    config_recs = load_recommendations_config()

    corner_verdicts_by_key = {}
    ls_by_key = {}
    intervention_abs_by_corner = {}
    intervention_abs_masking_by_corner = {}
    intervention_tc_by_corner = {}
    for e in evidence:
        if e["type"] == "corner_verdict":
            corner_verdicts_by_key.setdefault((e["corner"], e["phase"]), []).append(e)
        elif e["type"] == "ls_disambiguation":
            ls_by_key[(e["corner"], e["phase"])] = e
        elif e["type"] == "intervention_abs":
            intervention_abs_by_corner[e["corner"]] = e
        elif e["type"] == "intervention_tc":
            intervention_tc_by_corner[e["corner"]] = e
        elif e["type"] == "intervention_abs_masking":
            intervention_abs_masking_by_corner[e["corner"]] = e

    candidates = []
    candidates += _exit_oversteer_candidates(corner_verdicts_by_key, ls_by_key, registry, config_recs,
                                              intervention_tc_by_corner)
    candidates += _brake_balance_candidates(evidence, registry, config_recs)
    # brake_bias bridge (Segers ch. 5)
    candidates += _brake_bias_candidates(evidence)
    candidates += _bridge_candidates_for_matrix_rules(evidence, registry, config_recs, intervention_abs_by_corner,
                                                        intervention_abs_masking_by_corner)
    candidates += _bridge_candidates_for_levers(evidence, registry, config, candidates, setup_data)
    # kerb blowoff -- click-class, not affected by the gate below
    candidates += _kerb_blowoff_candidates(evidence, registry)
    # heavy correctors (springs/camber/toe) gated before the feedback-only
    # generator, so a gated-out one doesn't block a click-class alternative
    candidates = _apply_eligibility_gate(candidates, evidence, config, driving_level=driving_level)
    # default status "proposed" and trigger data_only where no generator set one
    for c in candidates:
        c.setdefault("status", STATUS_PROPOSED)
        c.setdefault("trigger_provenance", TRIGGER_DATA_ONLY)
    # feedback-only candidates, deduped against everything so far
    candidates += _feedback_only_candidates(evidence, candidates, registry, config, current_setup=setup_data)
    # feedback corroborates matching candidates (min confidence); data_only
    # + matching feedback -> both_agreeing
    candidates = _attach_feedback_evidence(candidates, evidence)
    # driver vs data disagreement: display only
    candidates = _attach_conflicting_feedback(candidates, evidence)
    # window edges (skipped without setup_data)
    candidates = _apply_window_edge_status(candidates, setup_data, registry, config)
    # breadth (null without assessed_corner_ids)
    candidates = _attach_breadth(candidates, assessed_corner_ids)
    # TC caution last, covers every candidate
    candidates = _attach_tc_safety_note(candidates, config)
    return candidates


# Scoring: six terms -- problem weight (severity x phase_importance x
# confidence), change_time, breadth, headroom, interaction, effect_class.
# Weights in cost_function (placeholders, except the elicited
# phase_importance/effect_class orderings). Deterministic.

# performance_axis -> verdict for the interaction lookup;
# braking/traction_performance have no verdict analogue -> never matched
_AXIS_TO_VERDICT = {
    "understeer_tendency": "understeer",
    "oversteer_tendency": "oversteer",
    "yaw_stability": "unstable_yaw",
}


def _candidate_confidence(candidate):
    # MIN, not mean -- one shaky ref drags the candidate down
    confs = [e["confidence"] for e in candidate["evidence_refs"] if e.get("confidence") is not None]
    return min(confs) if confs else 0.0


def _settings_window_component(candidate, current_setup, registry, decision_config, weight):
    windows = decision_config["parameter_windows"]
    distances = []
    flags = []
    for action in candidate["actions"]:
        if "target" in action:
            continue  # absolute target, no delta
        param = action["parameter"]
        window = windows.get(param, {})
        nominal, span = window.get("nominal"), window.get("span")
        entry = registry.get(param)
        current = _current_setup_value(current_setup, entry) if entry else None
        if nominal is None or span is None or not span or current is None:
            flags.append(f"{param}: settings-window distance not computable "
                         f"(nominal={nominal}, span={span}, current={current}) -- neutral, contributes 0")
            continue
        # enum parameters stored as labels ("P9") -> float() fails -> neutral,
        # same as a missing value
        try:
            new_value = float(current) + action["delta"]
        except (TypeError, ValueError):
            flags.append(f"{param}: settings-window distance not computable "
                         f"(current={current!r} is not numeric, likely an enum label) -- neutral, contributes 0")
            continue
        distances.append(min(1.0, abs(new_value - nominal) / span))
    if not distances:
        return 0.0, flags
    # worst (max) distance over a package, like _rank_key
    return weight * (1.0 - max(distances)), flags


def _interaction_penalty(candidate, evidence, decision_config, weight):
    # Known limitation: corner_verdict and matrix_verdict can describe the
    # same event; the other type's duplicate could count as an "other
    # problem". Inert today (no interaction entry targets oversteer_tendency).
    # Fix would need evidence de-duplication.
    table = decision_config["interaction_table"]
    corner = candidate["corner"]
    own_refs = {id(e) for e in candidate["evidence_refs"]}
    other_active = [e for e in evidence
                    if e["corner"] == corner and id(e) not in own_refs
                    and e.get("severity") not in (None, "normal")]
    other_verdicts = {e["verdict"] for e in other_active if e.get("verdict")}

    param_directions = {(a["parameter"], a.get("direction")) for a in candidate["actions"]}
    penalty = 0.0
    notes = []
    for entry in table:
        if (entry["parameter"], entry["direction"]) not in param_directions:
            continue
        target_verdict = _AXIS_TO_VERDICT.get(entry["performance_axis"])
        if target_verdict is None or target_verdict not in other_verdicts:
            continue  # no other problem on this axis
        penalty += weight * entry["sign"]
        notes.append(f"{entry['parameter']} {entry['direction']} -> {entry['performance_axis']} "
                     f"(sign={entry['sign']:+d}, grade={entry['grade']})")
    return penalty, notes


def _breadth_penalty(candidate, weight):
    """-(1 - helped/touched): 0 if it helps every assessed corner (or is exempt),
    0 + flag if breadth was never computed.
    """
    helped = candidate.get("corners_helped")
    touched = candidate.get("corners_touched")
    if helped is None or touched is None:
        return 0.0, ["breadth not computable (no assessed_corner_ids supplied to "
                      "generate_candidates) -- neutral, contributes 0"]
    if not touched:
        return 0.0, []
    return weight * -(1.0 - len(helped) / len(touched)), []


def score(candidate, evidence, current_setup, config):
    """Six terms weighted by config['cost_function'], summed; breakdown kept for
    the UI. (1) problem weight = severity x phase_importance x confidence,
    then (2) change_time, (3) breadth, (4) headroom, (5) interaction,
    (6) effect_class. effect_class stays out of the problem weight -- the
    same problem gets different effect classes from different levers.
    evidence = full build_evidence list (interaction term needs the corner's
    other problems). current_setup = setup sheet.
    """
    weights = config["cost_function"]
    registry = load_setup_parameters_registry()
    flags = []

    primary = candidate["evidence_refs"][0] if candidate["evidence_refs"] else None
    sev_rank = SEVERITY_RANK.get(primary.get("severity") if primary else None, 0)
    phase_importance = weights["phase_importance"].get(candidate["phase"], 1.0)
    confidence = _candidate_confidence(candidate)
    c_problem_weight = weights["severity"] * sev_rank * phase_importance * confidence

    if candidate["effort_class"] is None:
        c_change_time = 0.0
        flags.append("effort_class unset (unrouted candidate)")
    else:
        c_change_time = weights["change_time"] / (EFFORT_RANK.get(candidate["effort_class"], 0) + 1)

    c_breadth, breadth_flags = _breadth_penalty(candidate, weights["breadth"])
    flags += breadth_flags

    c_window, window_flags = _settings_window_component(
        candidate, current_setup, registry, config, weights["headroom"])
    flags += window_flags

    c_interaction, interaction_notes = _interaction_penalty(
        candidate, evidence, config, weights["interaction"])

    if candidate["effect_class"] is None:
        c_effect_class = 0.0
        flags.append("effect_class unset (unrouted candidate)")
    else:
        c_effect_class = weights["effect_class"].get(candidate["effect_class"], 0.0)

    total = c_problem_weight + c_change_time + c_breadth + c_window + c_interaction + c_effect_class

    return {
        "total": round(total, 4),
        "components": {
            "problem_weight": round(c_problem_weight, 4),
            "change_time": round(c_change_time, 4),
            "breadth": round(c_breadth, 4),
            "headroom": round(c_window, 4),
            "interaction": round(c_interaction, 4),
            "effect_class": round(c_effect_class, 4),
        },
        "interaction_notes": interaction_notes,
        "flags": flags,
    }


def _score_and_sort(candidates, evidence, current_setup, config):
    # score descending, tie-break on candidate id -> deterministic
    scored = []
    for c in candidates:
        result = score(c, evidence, current_setup, config)
        scored.append({
            **c,
            "score": result["total"],
            "score_components": result["components"],
            "score_interaction_notes": result["interaction_notes"],
            "score_flags": result["flags"],
        })
    scored.sort(key=lambda c: (-c["score"], c["id"]))
    return scored


def generate_shortlist(candidates, evidence, current_setup, config):
    """Ranked status == "proposed" candidates only; other statuses go to the
    inventory tail.
    """
    scored = _score_and_sort(candidates, evidence, current_setup, config)
    return [c for c in scored if c.get("status", STATUS_PROPOSED) == STATUS_PROPOSED]


def reachable_lever_keys(registry):
    """Every recommendation_target registry key -- session-wide, not per corner."""
    return {k for k, v in registry.items() if isinstance(v, dict) and v.get("recommendation_target")}


def generate_lever_inventory(candidates, evidence, current_setup, config, registry):
    """Every reachable lever gets exactly one status. Order: proposed (as in
    the shortlist), then real non-proposed by score, then no_trigger rows
    unranked.
    """
    scored = _score_and_sort(candidates, evidence, current_setup, config)
    proposed = [c for c in scored if c.get("status", STATUS_PROPOSED) == STATUS_PROPOSED]
    tail_real = [c for c in scored if c.get("status", STATUS_PROPOSED) != STATUS_PROPOSED]
    touched = {action["parameter"] for c in candidates for action in c["actions"]}
    no_trigger = [
        {
            "id": f"no_trigger:{lever}",
            "status": STATUS_NO_TRIGGER,
            "lever": lever,
            "corner": None,
            "phase": None,
            "actions": [],
            "evidence_refs": [],
            "rationale": "No evidence pointed at this lever this session.",
        }
        for lever in sorted(reachable_lever_keys(registry) - touched)
    ]
    return proposed + tail_real + no_trigger


def generate_display_split(candidates, evidence, current_setup, config, registry):
    """(shortlist, tail). Shortlist = every proposed candidate; the visible
    cutoff is apply_display_top_n, after conflict resolution and grouping.
    Tail = every other inventory entry with its status. Any length.
    """
    inventory = generate_lever_inventory(candidates, evidence, current_setup, config, registry)
    visible = [c for c in inventory if c.get("status") == STATUS_PROPOSED]
    visible_ids = {id(c) for c in visible}
    tail = [c for c in inventory if id(c) not in visible_ids]
    return {"shortlist": visible, "tail": tail}


def apply_display_top_n(shortlist, tail, config):
    """Top N distinct proposals visible, rest to the tail -- never dropped.
    Call after group_display_rows: "distinct" = distinct display rows;
    before grouping the list would under-fill.
    Returns (visible, combined_tail, tail_note or None).
    """
    top_n = config["display_top_n"]["value"]
    visible = shortlist[:top_n]
    overflow = shortlist[top_n:]
    combined_tail = overflow + tail
    tail_note = None
    if overflow:
        tail_note = f"{len(overflow)} further alternative{'s' if len(overflow) != 1 else ''}"
    return visible, combined_tail, tail_note


# Top line shows a magnitude only with a real delta AND a linear registry
# unit (int/float with unit). Enum levers (springs, wing, arb mount) carry
# a routing sign, not a step -> direction only, by design.
# Wording from registry vocabulary: symmetric words -> plain opposite;
# lever-specific words from direction_semantics (TC intervention, diff
# locking, wing position, brake bias forward/rearward).

_SYMMETRIC_PHRASES = {
    "soften": "softer", "stiffen": "stiffer",
    "more_negative": "more negative", "less_negative": "less negative",
    "more_positive": "more positive", "less_positive": "less positive",
}

_LEVER_DIRECTION_PHRASES = {
    ("brake_bias", "more_rear"): "rearward",
    ("brake_bias", "more_front"): "forward",
    ("abs_position", "more_fa_stability"): "more front-axle stability",
    ("abs_position", "more_ra_stability"): "more rear-axle stability",
    ("tc_lat", "increase"): "more intervention",
    ("tc_lat", "decrease"): "less intervention",
    ("tc_lon", "increase"): "more intervention",
    ("tc_lon", "decrease"): "less intervention",
    ("diff_position", "increase"): "more locking",
    ("diff_position", "decrease"): "less locking",
    ("wing_position", "increase"): "higher position",
    ("wing_position", "decrease"): "lower position",
    ("ride_height_front", "decrease"): "lower to the ground",
    ("ride_height_front", "increase"): "higher off the ground",
}


def _direction_phrase(parameter, direction):
    return (_LEVER_DIRECTION_PHRASES.get((parameter, direction))
            or _SYMMETRIC_PHRASES.get(direction)
            or direction.replace("_", " "))


def render_action_line(action, registry):
    """One action's fragment -- magnitude rule above."""
    param = action["parameter"]
    entry = registry.get(param, {})
    label = entry.get("label", param)
    delta = action.get("delta")
    value_space = entry.get("value_space") or {}
    unit = value_space.get("unit")
    is_linear = value_space.get("type") in ("int", "float")

    if delta and is_linear and unit:
        sign = "+" if delta > 0 else "-"
        line = f"{label} {sign}{abs(delta)} {unit}"
        if param == "brake_bias":
            line += f" {_direction_phrase(param, action['direction'])}"
        return line
    return f"{label}: {_direction_phrase(param, action['direction'])}"


def render_top_line(candidate, registry):
    """The change only -- no corner, verdict or provenance. Unrouted candidates
    don't name the corner here either.
    """
    actions = candidate.get("actions")
    if not actions:
        return "Engineer attention: no routed action"
    return " + ".join(render_action_line(a, registry) for a in actions)


# row colour: SEVERITY_RANK words -> the UI's existing strong/moderate/
# normal map; None -> NEUTRAL

def candidate_severity(candidate):
    """First severity in evidence_refs, or None (intervention/feedback only)."""
    for e in candidate.get("evidence_refs", []):
        sev = e.get("severity")
        if sev is not None:
            return sev
    return None


# tail rows

def render_tail_line(entry, registry):
    """Tail line: no_trigger -> lever name; real candidate -> render_top_line
    text + status reason.
    """
    status = entry.get("status")
    if status == STATUS_NO_TRIGGER:
        lever = entry["lever"]
        label = registry.get(lever, {}).get("label", lever)
        return f"{label}: no trigger this session"

    line = render_top_line(entry, registry)
    if status == STATUS_BLOCKED_AT_EDGE:
        return f"{line} -- BLOCKED ({entry.get('edge_reason', 'at edge')})"
    if status == STATUS_CONTRADICTED:
        reasons = entry.get("condition_reasons") or ["contradicted"]
        return f"{line} -- {reasons[0]}"
    if status == STATUS_NOT_ASSESSABLE:
        reasons = entry.get("condition_reasons")
        reason = reasons[0] if reasons else entry.get("edge_reason", "not assessable")
        return f"{line} -- not assessable ({reason})"
    if status == STATUS_PROPOSED:
        return f"{line} -- below display threshold (score {entry.get('score', 0):.2f})"
    return line


# Tyre pressure: check-only flags beside the shortlist, never in it.

_TPMS_CORNERING_PHASES = ("entry_2_turnin", "apex_3", "exit_4", "exit_5")
# cornering phases only -- straight-line pressure drop is physics, not a
# flag (diagnostics/inspect_tpms_pressure_cornering_phase.py)

_TPMS_WHEEL_LABELS = {"fl": "FL", "fr": "FR", "rl": "RL", "rr": "RR"}


def _tpms_cornering_median(channels, corners, state, wheel):
    # Returns (median_bar, glitch_count) or (None, 0). Samples outside the
    # channel's channels.json range are dropped and counted (census: none on
    # either session).
    ch = (channels or {}).get(f"tpms_press_{wheel}")
    if ch is None or ch.get("quality") in ("missing", "failed") or ch.get("time") is None or state is None:
        return None, 0
    t = state["time"]
    interp = np.interp(t, ch["time"], ch["data"])
    samples = []
    for c in corners or []:
        segments = c.get("segments", {})
        for phase in _TPMS_CORNERING_PHASES:
            idx = _phase_window_indices(t, segments.get(phase))
            if idx is None:
                continue
            lo, hi = idx
            samples.append(interp[lo:hi])
    if not samples:
        return None, 0
    vals = np.concatenate(samples)
    vals = vals[np.isfinite(vals)]
    if vals.size == 0:
        return None, 0
    lo_range, hi_range = load_channels_config()["channels"][f"tpms_press_{wheel}"]["range"]
    in_range = (vals >= lo_range) & (vals <= hi_range)
    glitch_count = int((~in_range).sum())
    sane = vals[in_range]
    if sane.size == 0:
        return None, glitch_count
    return float(np.median(sane)), glitch_count


def tyre_pressure_flags(config, channels=None, corners=None, state=None):
    """Check-only, displayed beside the shortlist -- pressures are never
    recommended. Per wheel: median over cornering-phase samples vs the
    tyre_pressure_target band.
    Null target -> silent. Filled target, dead channel -> "pressure not
    evaluable".
    """
    targets = config.get("tyre_pressure_target", {})
    flags = []
    for wheel in ("fl", "fr", "rl", "rr"):
        target = targets.get(wheel)
        if not isinstance(target, dict) or target.get("min_bar") is None or target.get("max_bar") is None:
            continue
        label = _TPMS_WHEEL_LABELS[wheel]
        median, glitch_count = _tpms_cornering_median(channels, corners, state, wheel)
        if median is None:
            flags.append(f"{label}: pressure not evaluable (channel dead/missing)")
            continue
        lo, hi = target["min_bar"], target["max_bar"]
        if lo <= median <= hi:
            continue
        direction = "under" if median < lo else "over"
        line = f"{label} {median:.2f} — {direction} target {lo:.2f}-{hi:.2f}"
        compound_note = targets.get("compound_note")
        if compound_note:
            line += f" [compound: {compound_note}]"
        if glitch_count:
            line += f" ({glitch_count} glitch sample{'s' if glitch_count != 1 else ''} excluded)"
        flags.append(line)
    return flags


# Display grouping: rows with the same render_top_line string collapse into
# one row; group score = max member score (breadth already covers
# multi-corner help). Display only, after scoring and conflict resolution.
# Never grouped: no_trigger rows (all render the same fallback text) and
# unrouted candidates (no parameter set; each corner stays its own flag).

def _group_key(candidate, registry):
    if candidate.get("status") == STATUS_NO_TRIGGER:
        return ("no_trigger", candidate.get("lever"))
    if not candidate.get("actions"):
        return ("unrouted", id(candidate))
    return render_top_line(candidate, registry)


def group_display_rows(rows, registry):
    """Rows with equal _group_key -> one row with "group_members"; fields copied
    from the highest-scoring member. Order kept; single-member groups
    unchanged (no group_members key).
    """
    grouped, order = {}, []
    for c in rows:
        key = _group_key(c, registry)
        if key not in grouped:
            grouped[key] = []
            order.append(key)
        grouped[key].append(c)

    result = []
    for key in order:
        members = grouped[key]
        if len(members) == 1:
            result.append(members[0])
            continue
        best = max(members, key=lambda m: m.get("score", float("-inf")))
        merged = dict(best)
        merged["group_members"] = members
        result.append(merged)
    return result


# conflict resolver

def resolve_conflicts(shortlist):
    """Corner with candidates in different directions/targets on the same
    parameter:
    1. platform calming: a candidate at this corner whose evidence covers
       every verdict involved wins -> 'platform_calming_available', the rest
       'superseded_by_platform_calming'.
    2. else time loss: higher phase_importance wins, ties by score ->
       'wins_time_loss' / 'superseded_by_time_loss'.
    Adds conflict_status, conflict_with, conflict_resolution_note; never
    removes a candidate.
    """
    weights = load_decision_frame_config()["cost_function"]["phase_importance"]

    for c in shortlist:
        c["conflict_status"] = None
        c["conflict_with"] = []

    by_corner = {}
    for c in shortlist:
        by_corner.setdefault(c["corner"], []).append(c)

    for cid, group in by_corner.items():
        param_to_entries = {}
        for c in group:
            for a in c["actions"]:
                param_to_entries.setdefault(a["parameter"], []).append((c, _action_key(a)))

        conflicting_ids = set()
        for param, entries in param_to_entries.items():
            if len({key for _, key in entries}) > 1:
                conflicting_ids.update(c["id"] for c, _ in entries)
        if not conflicting_ids:
            continue

        conflicting_candidates = [c for c in group if c["id"] in conflicting_ids]
        for c in conflicting_candidates:
            c["conflict_with"] = sorted({o["id"] for o in conflicting_candidates if o["id"] != c["id"]})

        conflicting_verdicts = {
            e["verdict"] for c in conflicting_candidates for e in c["evidence_refs"] if e.get("verdict")
        }

        platform_calming = None
        if len(conflicting_verdicts) > 1:
            for c in group:
                own_verdicts = {e["verdict"] for e in c["evidence_refs"] if e.get("verdict")}
                if conflicting_verdicts <= own_verdicts:
                    platform_calming = c
                    break

        if platform_calming is not None:
            for c in conflicting_candidates:
                if c["id"] != platform_calming["id"]:
                    c["conflict_status"] = "superseded_by_platform_calming"
                    c["conflict_resolution_note"] = (
                        f"Candidate '{platform_calming['id']}' addresses both conflicting "
                        f"problems at C{cid} at once -- preferred over a single-purpose lever."
                    )
            platform_calming["conflict_status"] = "platform_calming_available"
            continue

        def _time_loss(c):
            phases = c.get("phases") or (c["phase"],)
            return max(weights.get(p, 1.0) for p in phases)

        ranked = sorted(conflicting_candidates, key=lambda c: (-_time_loss(c), -c["score"]))
        winner = ranked[0]
        winner["conflict_status"] = "wins_time_loss"
        for c in ranked[1:]:
            c["conflict_status"] = "superseded_by_time_loss"
            c["conflict_resolution_note"] = (
                f"Superseded by '{winner['id']}' at C{cid} (higher phase-importance weight "
                f"{_time_loss(winner):.2f} vs {_time_loss(c):.2f})."
            )

    return shortlist
