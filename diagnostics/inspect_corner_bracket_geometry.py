# Follow-up to the cs_max_window_m derivation: checks whether 53.0 m
# genuinely stays "within one corner" -- reports each of the 14 physical
# corners' own bracket length (bracket_end_m - bracket_start_m, a pure
# geometric fact, independent of any CS floor) and the gap to its
# neighbours, so a real answer exists to "does 53 m risk bridging into
# an adjacent corner or straight."

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"


def main():
    params = load_parameters()
    data = parse_csv(RAW_FILE)
    prepare_vehicle_state(data["channels"], params)  # unused, just to confirm no crash

    by_id = {}
    for c in data.get("corners", []):
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
        canon.append((cid, mid_s, mid_e))
    canon.sort(key=lambda x: x[1])

    print(f"{'corner':>8} {'bracket_start_m':>16} {'bracket_end_m':>14} {'length_m':>10} {'gap_to_next_m':>14}")
    for i, (cid, bs, be) in enumerate(canon):
        length = be - bs
        if i + 1 < len(canon):
            gap = canon[i + 1][1] - be
        else:
            gap = float("nan")
        print(f"{'C' + str(cid):>8} {bs:>16.1f} {be:>14.1f} {length:>10.1f} {gap:>14.1f}")


if __name__ == "__main__":
    main()
