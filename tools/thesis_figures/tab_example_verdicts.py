# tab:example_verdicts -- one session, one row per corner: corner ID (of
# that session), canonical speed class, apex speed (median over the analysis
# laps of the stored apex_speed, the speed at the apex instant -- not a
# corner minimum, which is not stored), worst CS_f and CS_r with their phase
# (worst lap per phase as the classifier aggregates, then the minimum over
# the phases), dMz/dbeta (cross-lap median per phase, the minimum over the
# phases -- the value the open-loop-unstable rule reads) with its phase, and
# the verdict as the classifier states it, with the thesis phase words.
# Stored, identity-valid payload only.
# Run: python -m tools.thesis_figures.tab_example_verdicts --outing 12 [--tag r2] [--allow-status]
#      [--allow-excluded] [--out DIR]

import argparse

import numpy as np

from tools.thesis_figures import common

SCRIPT = "tab_example_verdicts"


def _worst(agg, getter):
    from modules.corner_aggregation import PHASE_KEYS
    best = (float("nan"), "-")
    for ph in PHASE_KEYS:
        v = getter(agg, ph)
        if v == v and (best[0] != best[0] or v < best[0]):
            best = (float(v), common.PHASE_LABELS[ph])
    return best


def build(outing, tag="dev", allow_status=False, allow_excluded=False, out=None):
    from modules.classifier import verdict_cs_stat
    from modules.corner_aggregation import aggregate_by_corner
    session = common.load_session(outing)
    payload, why = common.stored_payload(session)
    if payload is None:
        raise common.Refused(f"outing {outing}: {why}")
    marks = common.gate(outing, payload["fit_manifest"], payload["fallback_used"], allow_status, allow_excluded)
    label = common.session_label(session["row"])
    common.write_label_map(tag, [outing])
    summaries = payload["summaries"]
    cols = {k: [] for k in ("corner_id", "speed_class", "apex_speed_kmh", "cs_f_worst", "cs_f_phase", "cs_r_worst",
                            "cs_r_phase", "dmz_dbeta_Nm_per_deg", "dmz_phase", "verdict", "verdict_code")}
    for cid, agg in sorted(aggregate_by_corner(summaries).items(), key=lambda kv: kv[0]):
        laps = [s for s in summaries if s["stable_corner_id"] == cid]
        csf = _worst(agg, lambda a, ph: verdict_cs_stat(a, ph, "f")["median"])
        csr = _worst(agg, lambda a, ph: verdict_cs_stat(a, ph, "r")["median"])
        dmz = _worst(agg, lambda a, ph: a["phases"][ph]["stability_observed_Nm_per_deg"]["median"])
        text, code, _state = common.verdict(agg)
        for k, v in (("corner_id", cid), ("speed_class", agg.get("speed_class")),
                     ("apex_speed_kmh", float(np.median([s["apex_speed"] for s in laps]))),
                     ("cs_f_worst", csf[0]), ("cs_f_phase", csf[1]), ("cs_r_worst", csr[0]), ("cs_r_phase", csr[1]),
                     ("dmz_dbeta_Nm_per_deg", dmz[0]), ("dmz_phase", dmz[1]), ("verdict", text.replace(",", ";")),
                     ("verdict_code", code)):
            cols[k].append(v)
    data_dir, plots_dir, _prefix = common.out_dirs(out)
    common.write_csv(f"{data_dir}/{SCRIPT}.csv", cols,
                     common.meta(SCRIPT, [outing], tag, marks, session_label=label,
                                 apex_speed="apex speed: median over the analysis laps of the speed at the apex instant",
                                 dmz="cross-lap median per phase; minimum over the phases"))
    # value and phase in one cell keeps the table inside \textwidth
    header = ["C", "Speed", "$v_\\mathrm{apex}$ in km/h", f"worst {common.CS_F}", f"worst {common.CS_R}",
              "$\\mathrm{d}M_z/\\mathrm{d}\\beta$ in N\\,m/deg", "Verdict"]

    def cell(v, fmt, phase):
        # value over its phase, so the verdict column keeps the width
        return f"{format(v, fmt)}\\newline {common.tex_escape(phase)}" if v == v else "--"
    body = [[str(c), common.tex_escape(cols["speed_class"][i] or "-"),
             format(cols["apex_speed_kmh"][i], ".0f") if cols["apex_speed_kmh"][i] == cols["apex_speed_kmh"][i] else "--",
             cell(cols["cs_f_worst"][i], ".2f", cols["cs_f_phase"][i]),
             cell(cols["cs_r_worst"][i], ".2f", cols["cs_r_phase"][i]),
             cell(cols["dmz_dbeta_Nm_per_deg"][i], ".0f", cols["dmz_phase"][i]),
             common.tex_escape(cols["verdict"][i])]
            for i, c in enumerate(cols["corner_id"])]
    narrow = ">{\\raggedright\\arraybackslash}p{0.1\\textwidth}"
    common.write_table(f"{plots_dir}/{SCRIPT}.tex", SCRIPT, header, body,
                       f"r l r {narrow} {narrow} {narrow} >{{\\raggedright\\arraybackslash}}X", tabcolsep="4pt")
    return cols


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--outing", type=int, required=True)
    ap.add_argument("--tag", default="dev")
    ap.add_argument("--allow-status", action="store_true")
    ap.add_argument("--allow-excluded", action="store_true")
    ap.add_argument("--out")
    a = ap.parse_args(argv)
    build(a.outing, a.tag, a.allow_status, a.allow_excluded, a.out)


if __name__ == "__main__":
    main()
