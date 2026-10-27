# Per-corner verdict from the aggregated (or per-lap) corner summary: which
# axle is limiting, in which phase, how severe. One classifier for the setup
# engine and the UI; the UI maps the returned state to colour.
#
# Verdict rule (D2): the physical boundary decides, the percentile tiers
# only grade depth:
# - an axle is LIMITING when its cornering-stiffness ratio is below zero,
#   i.e. past the peak of its Fy / Fz curve; the test is CS < -margin with
#   a margin per axle (classification.cs_boundary_margin_front / _rear, p90
#   of the combined observer + vehicle-input uncertainty) so estimator noise
#   around zero does not toggle the verdict;
# - front limiting -> understeer symptom (US), moderate; strong when the
#   front value also lies in the strong tier (below STRONG_CSF);
# - rear limiting -> oversteer symptom (OS), always strong: a rear axle past
#   its peak removes the restoring yaw moment, i.e. a loss of stability
#   (Hoffman 2008, sec. 2);
# - rear between the boundary and its moderate tier -> information only, no
#   symptom (the rear still has positive slope);
# - open-loop unstable yaw (dMz/dbeta sign criterion, Werner 2021 sec.
#   2.2.3) -> symptom INST in whichever phase it occurs, strong;
# - a high-speed corner whose only symptom is front limiting is flagged
#   low_priority: mild high-speed understeer is the deliberate safe balance
#   of a race car (Milliken 1995, setup checklist items 21 and 1F).

from modules.corner_aggregation import PHASE_KEYS, PHASE_WORD_SHORT, PHASE_WORD_LONG, SEVERITY_RANK

NO_CS_SHORT = "no CS data"
NO_CS_LONG = "No CS data in any phase"

SYMPTOM_WORD = {"US": "understeer", "OS": "oversteer", "INST": "open-loop unstable"}


def verdict_cs_stat(summary, phase, axle):
    # the CS statistic the verdict reads for this phase -- one accessor for
    # the classifier and the per-phase table, so the two cannot disagree:
    # apex_3 from the apex region (distance window), not the fixed slice
    apex_region = summary.get("apex_region")
    if phase == "apex_3" and apex_region is not None:
        return apex_region[f"cs_ratio_{axle}"]
    return summary["phases"][phase][f"cs_ratio_{axle}"]


def _thresholds():
    from modules.stability_analysis import load_parameters
    cl = load_parameters()["classification"]
    return {
        "strong_f": cl["STRONG_CSF"]["value"], "moderate_r": cl["MODERATE_CSR"]["value"],
        "stab": cl["stab_neg_thresh_Nm_per_deg"]["value"],
        "boundary": {"f": cl["cs_boundary_margin_front"]["value"], "r": cl["cs_boundary_margin_rear"]["value"]},
        "marginal": cl["verdict_stability_margin"]["cs_margin"],
    }


def cs_decision_lines(thr=None):
    """The CS levels the verdict rule above reads, as values to draw: the
    per-axle boundaries (-margin), the front strong tier and the rear
    information tier. Every trace view and thesis figure takes its lines
    from here, so a drawn line can never be one the classifier ignores."""
    thr = thr or _thresholds()
    return {"boundary_f": -thr["boundary"]["f"], "strong_f": thr["strong_f"],
            "boundary_r": -thr["boundary"]["r"], "info_r": thr["moderate_r"]}


def phase_symptoms(summary, thr=None):
    """Symptoms per phase of one corner summary (one lap or the aggregate).

    Returns {"symptoms": [...], "info": [...], "any_cs": bool}; a symptom is
    {"symptom", "phase", "axle", "value", "severity"}. Phases without data
    produce nothing."""
    thr = thr or _thresholds()
    symptoms, info = [], []
    any_cs = False
    for phase in PHASE_KEYS:
        p = summary["phases"].get(phase)
        if p is None:
            continue
        csf = verdict_cs_stat(summary, phase, "f")["median"]
        csr = verdict_cs_stat(summary, phase, "r")["median"]
        sob = p["stability_observed_Nm_per_deg"]["median"]
        any_cs = any_cs or csf == csf or csr == csr
        if csf == csf and csf < -thr["boundary"]["f"]:
            sev = "strong" if csf < thr["strong_f"] else "moderate"
            symptoms.append({"symptom": "US", "phase": phase, "axle": "f", "value": csf, "severity": sev})
        if csr == csr:
            if csr < -thr["boundary"]["r"]:
                symptoms.append({"symptom": "OS", "phase": phase, "axle": "r", "value": csr, "severity": "strong"})
            elif csr < thr["moderate_r"]:
                info.append({"phase": phase, "axle": "r", "value": csr,
                             "note": "rear near its peak (moderate tier), no symptom"})
        if sob == sob and sob < thr["stab"]:
            symptoms.append({"symptom": "INST", "phase": phase, "axle": None, "value": sob, "severity": "strong"})
    return {"symptoms": symptoms, "info": info, "any_cs": any_cs}


def cs_value_state(value, axle, thr=None):
    # the class one phase value would set, for the per-phase colour: the
    # colour and the verdict read the same rule
    thr = thr or _thresholds()
    if value is None or value != value:
        return "no_data"
    if value < -thr["boundary"][axle]:
        if axle == "r":
            return "strong"
        return "strong" if value < thr["strong_f"] else "moderate"
    if axle == "r" and value < thr["moderate_r"]:
        return "info"
    return "normal"


def is_low_priority(symptoms, speed_class):
    return speed_class == "high" and bool(symptoms) and all(s["symptom"] == "US" for s in symptoms)


def _worst(symptoms, kind):
    cand = [s for s in symptoms if s["symptom"] == kind]
    return min(cand, key=lambda s: s["value"]) if cand else None


def _primary(symptoms):
    # primary axle verdict: rear limiting first (stability loss), then front;
    # each at its worst phase
    return _worst(symptoms, "OS") or _worst(symptoms, "US")


def verdict_phases(summary):
    """Phases the short verdict names -- the primary axle symptom's phase and
    the open-loop-unstable phase -- in phase order; empty for ok / no data.
    Same selection as classify_corner, so a view that marks these phases
    marks exactly what the verdict text says."""
    symptoms = phase_symptoms(summary)["symptoms"]
    named = {s["phase"] for s in (_primary(symptoms), _worst(symptoms, "INST")) if s is not None}
    return [p for p in PHASE_KEYS if p in named]


def classify_corner(summary):
    """Return (severity, short_verdict, long_verdict, state).

    state is the severity word, or "no_data" when no phase carries a CS
    value; the UI turns it into a colour. The SHORT string is also a machine
    interface (verdict words, bracket markers); LONG is the card text."""
    from modules.stability_analysis import load_parameters, thresholds_calibrated
    params = load_parameters()
    cls_cfg = params["classification"]
    thr = _thresholds()
    # thresholds valid only for the estimator path they were derived on
    uncalibrated_marker = "" if thresholds_calibrated(params) else " [UNCAL]"
    active_sideslip_source = params["stability_estimation"].get("sideslip_source", "kinematic")
    stab_calibrated_for = cls_cfg.get("stab_thresh_calibrated_for_sideslip_source", "kinematic")
    stab_legacy_marker = ("" if active_sideslip_source == stab_calibrated_for
                          else " [stab thresh: kinematic-era, not re-derived]")

    res = phase_symptoms(summary, thr)
    symptoms = res["symptoms"]
    severity = max((s["severity"] for s in symptoms), key=SEVERITY_RANK.get, default="normal")

    primary = _primary(symptoms)
    short_parts, long_parts = [], []
    if primary is not None:
        # MARGINAL: the deciding value sits within cs_margin of a boundary
        # that decides its class (the limiting boundary, and for the front
        # the strong tier)
        bounds = [-thr["boundary"][primary["axle"]]] + ([thr["strong_f"]] if primary["axle"] == "f" else [])
        marginal = " [MARGINAL]" if min(abs(primary["value"] - b) for b in bounds) <= thr["marginal"] else ""
        word = SYMPTOM_WORD[primary["symptom"]]
        short_parts.append(f"{word} @ {PHASE_WORD_SHORT[primary['phase']]}{marginal}")
        axle_word = "front" if primary["axle"] == "f" else "rear"
        long_parts.append(f"{word.capitalize()}, {PHASE_WORD_LONG[primary['phase']]}, "
                          f"{axle_word} CS {primary['value']:.2f}, {primary['severity']}")
    inst = _worst(symptoms, "INST")
    if inst is not None:
        short_parts.append(f"open-loop unstable @ {PHASE_WORD_SHORT[inst['phase']]}{stab_legacy_marker}")
        long_parts.append(f"Open-loop unstable, {PHASE_WORD_LONG[inst['phase']]}, {inst['value']:.0f} Nm/deg")
    if is_low_priority(symptoms, summary.get("speed_class")):
        long_parts.append("high-speed understeer only: low priority")

    state = severity
    if not short_parts and not res["any_cs"]:
        short_parts.append(NO_CS_SHORT)
        long_parts.append(NO_CS_LONG)
        state = "no_data"
    if not short_parts:
        short_parts.append("ok")
    if not long_parts:
        long_parts.append("Within normal range")
    return severity, " | ".join(short_parts) + uncalibrated_marker, " | ".join(long_parts), state
