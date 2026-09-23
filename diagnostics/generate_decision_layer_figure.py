# DIAGNOSTIC (read-only, [keep-reproduces]): regenerates the two thesis/
# supervisor figures for the decision layer (WP-DL Phase E, 2026-09-23) --
# a six-stage flow diagram and a lever coverage table -- straight from live
# config (config/decision_frame.json, config/setup_parameters.json,
# config/recommendations.json) plus the production classifier
# rule_bridge_status (modules/decision_frame.py), so neither figure can
# drift from what the pipeline actually does. No production write; run
# manually from the project root:
#     python -m diagnostics.generate_decision_layer_figure
#
# Two things this script CANNOT read from config, because they are Python
# control flow rather than config-driven mechanisms, and are therefore
# named as small cited constants below instead of invented or omitted:
# _exit_oversteer_candidates' fixed parameter surface (arb_rl/rr,
# springs_rear, tc_lon, diff_position) and _brake_bias_candidates' single
# dedicated lever (brake_bias). Both are modules/decision_frame.py
# functions, not config blocks -- re-check these two sets if either
# function's own parameter list ever changes.

import os
import re
import textwrap

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

from modules.decision_frame import load_decision_frame_config, rule_bridge_status
from modules.recommendation import load_recommendations_config, load_setup_parameters_registry

OUT_DIR = os.path.join("diagnostics", "plots_decision_layer")

# --- dedicated (non-config-driven) candidate functions, cited by name ----
_EXIT_OVERSTEER_BRIDGE_PARAMS = {"arb_rl", "arb_rr", "springs_rear", "tc_lon", "diff_position"}
_EXIT_OVERSTEER_BRIDGE_NOTE = "+ dedicated exit-oversteer bridge (_exit_oversteer_candidates, LS-routed)"
_BRAKE_BIAS_DEDICATED_PARAMS = {"brake_bias"}
_BRAKE_BIAS_DEDICATED_NOTE = "dedicated bridge only (_brake_bias_candidates, Segers C5-1/2); no lever_bridges/matrix entry"

# --- axle-pairing display rule (PLAN.md DECISION LAYER SPEC Stage 1) -----
# "camber ... axle-pairs"; "Dampers as 10 levers ... AXLE PAIRS ONLY, never
# asymmetric". arb_fl/fr/rl/rr deliberately excluded -- the registry's own
# notes (config/setup_parameters.json arb_fl/arb_rl) state "Four fully
# independent per-corner targets, no pairing concept". This is a display
# grouping rule (which registry keys share one coverage row), not a config
# value -- every field shown per row is still read live from the registry.
_AXLE_PAIRED_FAMILIES = {
    "camber", "damper_bump_ls", "damper_bump_hs", "damper_blowoff",
    "damper_rebound_ls", "damper_rebound_hs",
}
_CORNER_SUFFIX_RE = re.compile(r"^(?P<base>.+)_(?P<corner>fl|fr|rl|rr)$")


def _group_rows(reachable_keys):
    """One display row per Stage-1 lever family. Returns an ordered dict
    row_key -> [member registry keys]."""
    rows = {}
    handled = set()
    for key in sorted(reachable_keys):
        if key in handled:
            continue
        m = _CORNER_SUFFIX_RE.match(key)
        if m and m.group("base") in _AXLE_PAIRED_FAMILIES:
            base = m.group("base")
            front = [f"{base}_fl", f"{base}_fr"]
            rear = [f"{base}_rl", f"{base}_rr"]
            if all(k in reachable_keys for k in front):
                rows[f"{base}_front"] = front
                handled.update(front)
            if all(k in reachable_keys for k in rear):
                rows[f"{base}_rear"] = rear
                handled.update(rear)
        else:
            rows[key] = [key]
            handled.add(key)
    return rows


def _row_label(member_keys, registry):
    if len(member_keys) == 1:
        return registry[member_keys[0]]["label"]
    sample = registry[member_keys[0]]["label"]
    stripped = re.sub(r"^(FL|FR|RL|RR)\s+", "", sample)
    side = "Front" if member_keys[0].endswith("_fl") else "Rear"
    return f"{side} {stripped} (axle pair)"


def _collect_data():
    registry = load_setup_parameters_registry()
    dfc = load_decision_frame_config()
    recs = load_recommendations_config()

    reachable = {k: v for k, v in registry.items() if v.get("recommendation_target")}
    excluded = {k: v for k, v in registry.items() if not v.get("recommendation_target")}

    eligibility = dfc["eligibility_classes"]
    heavy = set(eligibility["heavy_correctors"])
    click = set(eligibility["click_class"])

    interaction_table = dfc["interaction_table"]
    lever_bridges = dfc["lever_bridges"]
    parameter_windows = dfc["parameter_windows"]

    primary_rules = [r for r in recs["rules"] if rule_bridge_status(r) == "primary"]
    matrix_params = set()
    for r in primary_rules:
        sugg = r["suggestion"] if isinstance(r["suggestion"], list) else [r["suggestion"]]
        for a in sugg:
            matrix_params.add(a["parameter"])

    # Mirrors modules/decision_frame.py _feedback_only_candidates' own gate
    # exactly: axis = "oversteer_tendency"/"understeer_tendency" (per the
    # feedback verdict), sign==1, parameter in click_class. Found during
    # Phase F's completeness check (2026-09-23): an earlier version of this
    # set omitted the axis filter and over-counted arb_rl/arb_rr (their
    # sign=+1 entries are on yaw_stability, an axis B2 never checks) as
    # feedback-eligible when they are not.
    _B2_AXES = {"understeer_tendency", "oversteer_tendency"}
    feedback_eligible = {e["parameter"] for e in interaction_table
                          if e["sign"] == 1 and e["parameter"] in click
                          and e["performance_axis"] in _B2_AXES}
    lever_bridge_params = {e["lever"] for e in lever_bridges}

    return {
        "registry": registry, "dfc": dfc, "recs": recs,
        "reachable": reachable, "excluded": excluded,
        "heavy": heavy, "click": click,
        "interaction_table": interaction_table, "lever_bridges": lever_bridges,
        "parameter_windows": parameter_windows,
        "primary_rules": primary_rules, "matrix_params": matrix_params,
        "feedback_eligible": feedback_eligible, "lever_bridge_params": lever_bridge_params,
    }


# ---------------------------------------------------------------------
# FIGURE 1 -- six-stage flow diagram
# ---------------------------------------------------------------------

def _box(ax, xy, w, h, title, body, title_size=9.5, body_size=6.8):
    x, y = xy
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.012",
                                 linewidth=1.1, edgecolor="black", facecolor="white"))
    ax.text(x + w / 2, y + h - 0.018, title, ha="center", va="top",
            fontsize=title_size, fontweight="bold")
    ax.text(x + 0.015, y + h - 0.05, body, ha="left", va="top",
            fontsize=body_size, linespacing=1.35, family="monospace")


def _arrow(ax, p0, p1):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle="-|>", mutation_scale=14,
                                  linewidth=1.1, color="black"))


def render_flow_diagram(data, out_path_base):
    dfc = data["dfc"]
    reachable = data["reachable"]
    excluded = data["excluded"]
    cf = dfc["cost_function"]
    threshold = dfc["display_score_threshold"]["value"]
    cond = dfc["conditions"]

    n_reachable = len(reachable)
    n_excluded = len(excluded)
    n_matrix = len(data["primary_rules"])
    n_bridges = len(data["lever_bridges"])
    n_fb_eligible = len(data["feedback_eligible"])
    data_only_levers = data["matrix_params"] | data["lever_bridge_params"] \
        | _EXIT_OVERSTEER_BRIDGE_PARAMS | _BRAKE_BIAS_DEDICATED_PARAMS
    n_data_only_levers = len(data_only_levers & reachable.keys())

    n_soft_filled = sum(
        1 for k in reachable
        if (data["parameter_windows"].get(k) or {}).get("nominal") is not None
    )
    n_interaction = len(data["interaction_table"])

    fig = plt.figure(figsize=(11.69, 8.27), dpi=200)  # landscape A4
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    fig.text(0.5, 0.975, "Decision Layer: Six-Stage Flow", ha="center",
              fontsize=14, fontweight="bold")
    fig.text(0.5, 0.955,
              "PLAN.md DECISION LAYER SPEC -- generated from live config, "
              "values printed as-is (config/decision_frame.json, config/setup_parameters.json)",
              ha="center", fontsize=7.5, style="italic", color="0.25")

    w, h = 0.30, 0.34
    gap_x, gap_y = 0.02, 0.06
    row1_y = 0.55
    row2_y = row1_y - h - gap_y
    xs = [0.02, 0.02 + w + gap_x, 0.02 + 2 * (w + gap_x)]

    _box(ax, (xs[0], row1_y), w, h, "1. Lever inventory",
         f"{n_reachable} reachable levers\n"
         f"(recommendation_target=true,\n"
         f" config/setup_parameters.json)\n"
         f"+ 1 check-only item (tyre pressure)\n"
         f"+ {n_excluded} written exclusions\n\n"
         f"Every lever resolves to exactly\n"
         f"one status: proposed / no-trigger /\n"
         f"blocked-at-edge / contradicted /\n"
         f"not-assessable. No silent gaps.")

    _box(ax, (xs[1], row1_y), w, h, "2. Triggers",
         f"Three labelled provenances:\n\n"
         f"data-only: {n_matrix}, primary matrix rules\n"
         f" + {n_bridges} lever_bridges entries\n"
         f" -> reach {n_data_only_levers} levers today\n\n"
         f"feedback-only (|fb|>=2): {n_fb_eligible} click-class\n"
         f" levers carry a sign=+1 tendency-axis\n"
         f" interaction_table entry" +
         (" -- OPEN gap\n (elicitation item 10): 0 today"
          if n_fb_eligible == 0 else "") + "\n\n"
         f"both-agreeing: data-only candidate\n"
         f" + matching feedback -> upgraded")

    _box(ax, (xs[2], row1_y), w, h, "3. Windows / current state",
         f"HARD edge = registry value_space\n"
         f" min/max (physical/legal bound)\n"
         f"SOFT edge = parameter_windows\n"
         f" nominal +/- span\n\n"
         f"{n_soft_filled}/{n_reachable} reachable levers carry\n"
         f"a filled soft window\n\n"
         f"AT EDGE -> BLOCKED, shown at its\n"
         f"earned rank -- never suppressed")

    _box(ax, (xs[0], row2_y), w, h, "4. Interference",
         f"{n_interaction} interaction_table entries\n"
         f"(signed, per performance_axis)\n\n"
         f"Shortlist semantics: PICK ONE --\n"
         f"one change per iteration, for\n"
         f"attribution\n\n"
         f"Same-axis overlaps: annotated\n"
         f"'overlapping effect', ranked\n"
         f"normally, never suppressed")

    _box(ax, (xs[1], row2_y), w, h, "5. Corroboration / contradiction",
         f"contradiction_sources:\n"
         f" {', '.join(cond['contradiction_sources'])}\n\n"
         f"data contradicts data ->\n"
         f" SUPPRESSED to tail\n"
         f" ('contradicted by X')\n\n"
         f"driver contradicts data -> shown\n"
         f" side by side, never suppresses\n\n"
         f"not-evaluable condition -> confidence\n"
         f" capped at {cond['not_evaluable_confidence_cap']}")

    _box(ax, (xs[2], row2_y), w, h, "6. Cost function + output",
         f"6 governed terms (cost_function,\n"
         f"live values):\n"
         f" severity={cf['severity']}  change_time={cf['change_time']}\n"
         f" breadth={cf['breadth']}  headroom={cf['headroom']}\n"
         f" interaction={cf['interaction']}\n"
         f" effect_class: primary={cf['effect_class']['primary']}"
         f" secondary={cf['effect_class']['secondary']}\n\n"
         f"display_score_threshold={threshold}\n"
         f"OUTPUT: shortlist (score>=threshold)\n"
         f" | assessed-not-proposed tail (rest)")

    # arrows: 1->2->3 (row 1, left to right), then an elbow from 3's bottom
    # over to 4's top (row 2 starts again at the left column), then 4->5->6
    # (row 2, left to right too) -- boxes stay in reading order throughout,
    # only the 3->4 transition needs a routed elbow to cross rows.
    _arrow(ax, (xs[0] + w, row1_y + h / 2), (xs[1], row1_y + h / 2))
    _arrow(ax, (xs[1] + w, row1_y + h / 2), (xs[2], row1_y + h / 2))
    mid_gap_y = row1_y - gap_y / 2
    ax.plot([xs[2] + w / 2, xs[2] + w / 2], [row1_y, mid_gap_y], color="black", linewidth=1.1)
    ax.plot([xs[2] + w / 2, xs[0] + w / 2], [mid_gap_y, mid_gap_y], color="black", linewidth=1.1)
    _arrow(ax, (xs[0] + w / 2, mid_gap_y), (xs[0] + w / 2, row2_y + h + 0.005))
    _arrow(ax, (xs[0] + w, row2_y + h / 2), (xs[1], row2_y + h / 2))
    _arrow(ax, (xs[1] + w, row2_y + h / 2), (xs[2], row2_y + h / 2))

    for ext in ("png", "pdf"):
        path = f"{out_path_base}.{ext}"
        fig.savefig(path)
    plt.close(fig)
    return f"{out_path_base}.png"


# ---------------------------------------------------------------------
# FIGURE 2 -- lever coverage table
# ---------------------------------------------------------------------

def _trigger_provenance(member_keys, data):
    tags = []
    if any(k in data["matrix_params"] for k in member_keys):
        tags.append("matrix (data-only)")
    if any(k in _EXIT_OVERSTEER_BRIDGE_PARAMS for k in member_keys):
        tags.append("exit-bridge (data-only)")
    if any(k in data["lever_bridge_params"] for k in member_keys):
        tags.append("lever_bridge (data-only)")
    if any(k in _BRAKE_BIAS_DEDICATED_PARAMS for k in member_keys):
        tags.append("dedicated fn (data-only)")
    if any(k in data["feedback_eligible"] for k in member_keys):
        tags.append("feedback-eligible")
    return "\n".join(tags) if tags else "none (inert)"


def _eligibility_class(member_keys, data):
    if any(k in data["heavy"] for k in member_keys):
        return "heavy\n(gated)"
    if any(k in data["click"] for k in member_keys):
        return "click"
    return "?"


def _window_filled(member_keys, data):
    rep = member_keys[0]
    pw = (data["parameter_windows"].get(rep) or {})
    return "yes" if pw.get("nominal") is not None else "no"


def _notes(member_keys, data):
    parts = []
    if any(k in _EXIT_OVERSTEER_BRIDGE_PARAMS for k in member_keys):
        parts.append(_EXIT_OVERSTEER_BRIDGE_NOTE)
    if any(k in _BRAKE_BIAS_DEDICATED_PARAMS for k in member_keys):
        parts.append(_BRAKE_BIAS_DEDICATED_NOTE)
    prov = _trigger_provenance(member_keys, data)
    if prov == "none (inert)":
        parts.append("inert: no click-class routing reaches this lever today")
    rep = member_keys[0]
    pw = (data["parameter_windows"].get(rep) or {})
    if pw.get("nominal") is None:
        # Show the registry's own note text directly (single source of
        # truth) rather than a synthesized paraphrase -- a paraphrase here
        # is exactly what went stale after the 2026-09-22 brake_bias/
        # splitter_offset promotion (Phase F close-out, 2026-09-23).
        note_text = pw.get("note", "")
        if note_text:
            parts.append(note_text)
    combined = "; ".join(parts) if parts else ""
    # Bounded regardless of how many parts contributed -- a row combining
    # two long notes (e.g. Brake Bias: dedicated-fn citation + the window
    # note) must still fit the table's fixed row height after wrapping.
    budget = 135
    return combined[:budget] + ("..." if len(combined) > budget else "")


def build_coverage_rows(data):
    rows = []
    grouped = _group_rows(data["reachable"].keys())
    for row_key, member_keys in grouped.items():
        registry = data["registry"]
        rows.append([
            _row_label(member_keys, registry),
            _trigger_provenance(member_keys, data),
            _eligibility_class(member_keys, data),
            _window_filled(member_keys, data),
            registry[member_keys[0]]["change_effort"],
            _notes(member_keys, data),
        ])

    # check-only row: tyre pressures (not a setup_parameters.json key at all)
    tpt = data["dfc"]["tyre_pressure_target"]
    rows.append([
        "Tyre pressures (per corner)",
        "plausibility check\n(cornering-phase only)",
        "check-only",
        "no",
        "n/a",
        "target window null -- " + tpt["derived_from"][:100] + "...",
    ])

    # written exclusions
    for key, entry in sorted(data["excluded"].items()):
        rows.append([
            entry["label"], "n/a -- written exclusion", "excluded", "n/a",
            entry.get("change_effort") or "n/a",
            (entry.get("mechanism") or "")[:110],
        ])
    return rows


_NOTE_ABBREVIATIONS = {
    _EXIT_OVERSTEER_BRIDGE_NOTE: "+EB (exit-oversteer bridge, _exit_oversteer_candidates)",
    _BRAKE_BIAS_DEDICATED_NOTE: "dedicated fn only (_brake_bias_candidates); no lever_bridges/matrix entry",
}


def _short_notes(note_text, wrap_width=46):
    # Narrow half-page columns can't carry full prose on one line --
    # abbreviate the two recurring structural citations, then wrap (matplotlib
    # table cells never wrap on their own; unwrapped text clips at the cell
    # edge instead of breaking, which is a legibility bug, not a style choice).
    for full, short in _NOTE_ABBREVIATIONS.items():
        note_text = note_text.replace(full, short)
    if not note_text:
        return ""
    return "\n".join(textwrap.wrap(note_text, width=wrap_width))


def render_coverage_table(data, out_path_base):
    rows = build_coverage_rows(data)
    n_reachable = len(data["reachable"])
    columns = ["Lever", "Trigger provenance(s)", "Elig.", "Win.",
               "Effort", "Notes"]
    col_widths = [0.20, 0.19, 0.10, 0.08, 0.11, 0.32]

    split = (len(rows) + 1) // 2
    halves = [rows[:split], rows[split:]]

    fig = plt.figure(figsize=(16.5, 8.27), dpi=200)  # landscape, wide enough for 2-up
    fig.text(0.5, 0.975, "Decision Layer: Lever Coverage", ha="center",
              fontsize=15, fontweight="bold")
    fig.text(0.5, 0.955,
              f"{len(rows)} rows cover all {n_reachable} recommendation_target=true registry keys "
              f"(axle-paired per Stage 1: camber, dampers) + 1 check-only item + "
              f"{len(data['excluded'])} written exclusions -- generated from live config "
              f"(config/setup_parameters.json + config/decision_frame.json + config/recommendations.json)",
              ha="center", fontsize=7.5, style="italic", color="0.25")
    fig.text(0.5, 0.028,
              "EB = dedicated exit-oversteer bridge (_exit_oversteer_candidates, LS-routed). "
              "'inert' = no data-only or feedback-only routing currently reaches this lever.",
              ha="center", fontsize=6.5, style="italic", color="0.35")

    panel_bounds = [(0.01, 0.47), (0.52, 0.98)]
    for half_rows, (x0, x1) in zip(halves, panel_bounds):
        ax = fig.add_axes([x0, 0.06, x1 - x0, 0.87])
        ax.axis("off")
        cell_text = [[_short_notes(c) if j == 5 else c for j, c in enumerate(row)]
                     for row in half_rows]
        table = ax.table(cellText=cell_text, colLabels=columns, colWidths=col_widths,
                          loc="upper left", cellLoc="left", bbox=[0, 0, 1, 1])
        table.auto_set_font_size(False)
        table.set_fontsize(5.4)
        for (r, c), cell in table.get_celld().items():
            cell.set_edgecolor("0.6")
            cell.PAD = 0.02
            if r == 0:
                cell.set_text_props(fontweight="bold", ha="center")
                cell.set_facecolor("0.85")
            else:
                cell.set_facecolor("white" if r % 2 else "0.96")

    for ext in ("png", "pdf"):
        path = f"{out_path_base}.{ext}"
        fig.savefig(path)
    plt.close(fig)
    return f"{out_path_base}.png"


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    data = _collect_data()
    flow_path = render_flow_diagram(data, os.path.join(OUT_DIR, "six_stage_flow"))
    table_path = render_coverage_table(data, os.path.join(OUT_DIR, "lever_coverage_table"))
    print(f"wrote {flow_path} (+ .pdf)")
    print(f"wrote {table_path} (+ .pdf)")
    print(f"coverage rows: {len(build_coverage_rows(data))}")


if __name__ == "__main__":
    main()
