# Metrology extension Phase 2g: verify the frame's traction-limited
# evidence (_build_ls_disambiguation_evidence) still fires correctly on
# Dubai's own known traction-limited corner (C3, thesis_notes.md
# "...C3's own established finding that C3 is traction-limited on all 4
# laps") now that LS_ratio consumes only VALID (post-repair) values.
# Read-only, no config/production change -- reuses inspect_deepening_
# phase4e_ls_thresholds.py's own run_pipeline unchanged.

from modules.stability_analysis import load_parameters
from modules.decision_frame import (
    aggregate_ls_by_corner, build_evidence, load_decision_frame_config,
)
from inspect_deepening_phase4e_ls_thresholds import run_pipeline, DUBAI_FILE


def classify_fn(summary):
    from ui.views.outing_form import OutingForm
    return OutingForm._classify_corner(None, summary)


def main():
    print("Running Dubai (production default, config Level-1 weighing)...")
    summaries = run_pipeline(DUBAI_FILE, setup_data=None)
    ls_stats = aggregate_ls_by_corner(summaries)
    config = load_decision_frame_config()

    evidence = build_evidence(summaries, ls_stats, config, classify_fn)
    ls_disambig = [e for e in evidence if e["type"] == "ls_disambiguation"]
    print(f"\ntotal ls_disambiguation evidence items: {len(ls_disambig)}")
    for e in sorted(ls_disambig, key=lambda e: (e["corner"], e["phase"])):
        flag = "  <=== C3" if e["corner"] == 3 else ""
        print(f"  C{e['corner']} {e['phase']}: verdict={e['verdict']} ls_class={e['ls_class']} "
              f"confidence={e['confidence']} source={e['source']}{flag}")

    c3_items = [e for e in ls_disambig if e["corner"] == 3]
    print(f"\nC3-specific ls_disambiguation items: {len(c3_items)}")
    if not c3_items:
        print("  NONE -- C3 either has no exit-phase oversteer corner_verdict this run, "
              "or its own LS_ratio is invalid at that phase. Reported, not assumed.")
    else:
        for e in c3_items:
            verdict_match = "TRACTION_LIMITED (matches the known finding)" if e["ls_class"] == "traction_limited" \
                else "CORNERING_LIMITED (does NOT match the known finding)"
            print(f"  {e['phase']}: {verdict_match} (LS_ratio_r vs session median, source: {e['source']})")


if __name__ == "__main__":
    main()
