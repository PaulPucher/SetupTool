# Part B pre-step: what does v3's CURRENT corner detection actually
# produce? Read-only, no config/production changes. Lists every stable
# corner (canonical bracket_start_m/end_m, from modules.corner_analysis'
# own median-across-laps realization) in track order with the gap to its
# neighbour, so the C17-C20 / C4-C5 / C8-C11 references in the work order
# can be checked against real output instead of assumed from memory.

from modules.csv_parser import parse_csv

RAW_FILE = "GT3_PRC_MLA-v3.txt"


def main():
    data = parse_csv(RAW_FILE)
    corners = data.get("corners", [])
    laps = data.get("laps", [])
    print(f"laps: {len(laps)} total, "
          f"{sum(1 for l in laps if l.get('is_valid_for_analysis'))} valid_for_analysis")

    by_id = {}
    for c in corners:
        cid = c.get("stable_corner_id")
        if cid is None:
            continue
        by_id.setdefault(cid, []).append(c)

    canon = []
    for cid, insts in by_id.items():
        bs = sorted(i["bracket_start_m"] for i in insts)
        be = sorted(i["bracket_end_m"] for i in insts)
        mid_s = bs[len(bs) // 2]
        mid_e = be[len(be) // 2]
        canon.append((cid, mid_s, mid_e, len(insts)))
    canon.sort(key=lambda x: x[1])

    print(f"\n{len(canon)} stable corners detected on v3\n")
    print(f"{'corner':>8} {'bracket_start_m':>16} {'bracket_end_m':>14} {'length_m':>10} "
          f"{'gap_to_next_m':>14} {'n_lap_instances':>16}")
    for i, (cid, bs, be, n) in enumerate(canon):
        length = be - bs
        gap = canon[i + 1][1] - be if i + 1 < len(canon) else float("nan")
        print(f"{'C' + str(cid):>8} {bs:>16.1f} {be:>14.1f} {length:>10.1f} {gap:>14.1f} {n:>16}")


if __name__ == "__main__":
    main()
