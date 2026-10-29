# Tire mileage vs the fitted axle parameters of the evaluation sessions
# (author + review order 2026-10-27, read-only, descriptive). Mileage is the
# outing field tyre_age in km (outing form, 0 shown as "New" -- the app
# cannot tell a new tire from a missing entry). mu and K = B C mu come from
# the stored fit manifest of each outing. Per track only (mu differs between
# tracks); Spearman rho with n, no p-values, no trend line. Rho is given
# with and without the 0 km entries, since those may be missing values.
# Run: python diagnostics/inspect_tire_mileage.py

import json
import os
import sqlite3
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from scipy.stats import spearmanr  # noqa: E402

from tools.thesis_figures import common  # noqa: E402  (labels, read-only DB path)

FINAL_WEEKENDS = (8, 9)


def main():
    con = sqlite3.connect(f"file:{common.DB}?mode=ro", uri=True)
    rows = con.execute("SELECT id, race_weekend_id, tyre_age, analysis_data FROM outings "
                       "WHERE race_weekend_id IN (?, ?) ORDER BY race_weekend_id, number, id", FINAL_WEEKENDS).fetchall()
    con.close()
    by_track = {}
    for oid, _wk, km, payload in rows:
        label = common.session_label(common.db_outing(oid))
        ax = json.loads(payload)["fit_manifest"]["axles"]
        k = {a: ax[a]["B"] * ax[a]["C"] * ax[a]["mu"] for a in ("front", "rear")}
        by_track.setdefault(label.split()[0], []).append(
            (label, km, ax["front"]["mu"], ax["rear"]["mu"], k["front"], k["rear"]))
    for track, recs in by_track.items():
        print(f"{track}:")
        for label, km, muf, mur, kf, kr in recs:
            note = "  (0 km: new tire or no entry)" if not km else ""
            print(f"  {label}: mileage {km} km; mu_f {muf:.3f}, mu_r {mur:.3f}, K_f {kf:.2f}, K_r {kr:.2f} 1/rad{note}")
        for name, i in (("mu_f", 2), ("mu_r", 3), ("K_f", 4), ("K_r", 5)):
            parts = []
            for tag, sel in (("all", recs), ("without 0 km", [r for r in recs if r[1]])):
                rho = spearmanr([r[1] for r in sel], [r[i] for r in sel]).statistic if len(sel) > 2 else float("nan")
                parts.append(f"{tag}: rho {rho:+.2f} (n {len(sel)})")
            print(f"  mileage vs {name}: " + "; ".join(parts))


if __name__ == "__main__":
    main()
