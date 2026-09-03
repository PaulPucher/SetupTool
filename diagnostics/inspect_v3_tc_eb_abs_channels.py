# Damper package, Phase 4: channel survey for traction control, engine
# braking, ABS activity-vs-position, and brake bias candidates in
# GT3_PRC_MLA-v3.txt. Read-only, single streaming pass (same technique as
# diagnostics/inspect_prc_v3_sample_rates.py, the prior v3 census this
# extends -- that census already covered "tc_lat"/"tc_lon" (zero matches)
# and "abs"/"brake_bias" as raw substrings (171 hits, named a short
# plausible-switch candidate list, no rate/range detail). This script:
# (a) adds "tract"/"asr" (traction control) and "eb"/"ebrake"/
# "engine_brake"/"map" (engine braking) as NEW search terms, (b) reports
# rate + value range + "changes during session" for every match, not just
# a name list, (c) re-confirms abs/brake_bias with the same detail.
#
# TOKEN MATCHING, not raw substring, for the short 2-3 letter terms ("tc",
# "eb", "map") -- a raw substring search on those would false-positive on
# nearly every channel containing those letters anywhere (e.g. "map" is a
# substring of dozens of unrelated ECU map/table channel names, "eb" of
# many more). Channel names are split on non-alphanumeric characters into
# tokens; a term matches if it equals a whole token OR is itself a
# multi-word compound ("engine_brake") checked as a literal substring
# (specific enough not to need token-splitting). "tract"/"asr"/"ebrake"/
# "abs"/"brake_bias" are also checked as substrings (longer, low false-
# positive risk, matches the original census's own convention for those).
# IDENTIFICATION EVIDENCE ONLY -- no mapping conclusion drawn here.

RAW_FILE = "GT3_PRC_MLA-v3.txt"

TOKEN_TERMS = {"tc", "eb", "map", "asr", "tract"}
SUBSTRING_TERMS = ("tract", "asr", "ebrake", "engine_brake", "abs", "brake_bias")


def _matches(channel_lower):
    tokens = set(_tokenize(channel_lower))
    if tokens & TOKEN_TERMS:
        return True
    return any(term in channel_lower for term in SUBSTRING_TERMS)


def _tokenize(name_lower):
    out, cur = [], []
    for ch in name_lower:
        if ch.isalnum():
            cur.append(ch)
        else:
            if cur:
                out.append("".join(cur))
                cur = []
    if cur:
        out.append("".join(cur))
    return out


def _rate_from_times(times):
    if len(times) < 2:
        return float("nan")
    return (len(times) - 1) / (times[-1] - times[0])


def main():
    total_blocks = 0
    detailed = {}

    with open(RAW_FILE, "r", encoding="latin-1") as f:
        line = f.readline()
        while line:
            if line.strip() == "{ChannelBlock}":
                total_blocks += 1
                header = f.readline()
                parts = header.rstrip("\n").split("\t")
                raw_name = parts[1] if len(parts) > 1 else ""
                channel = raw_name.split("[")[0]
                unit = raw_name[len(channel):].strip("[]") if "[" in raw_name else ""

                if _matches(channel.lower()):
                    times, values = [], []
                    row = f.readline()
                    while row and row != "\n":
                        cols = row.rstrip("\n").split("\t")
                        try:
                            t = float(cols[0])
                        except (ValueError, IndexError):
                            row = f.readline()
                            continue
                        times.append(t)
                        if len(cols) > 1 and cols[1] != "":
                            try:
                                values.append(float(cols[1]))
                            except ValueError:
                                pass
                        row = f.readline()
                    rate = _rate_from_times(times)
                    duration = (times[-1] - times[0]) if len(times) >= 2 else float("nan")
                    vmin = min(values) if values else float("nan")
                    vmax = max(values) if values else float("nan")
                    n_distinct = len(set(values)) if len(values) <= 200000 else None
                    detailed[channel] = {
                        "unit": unit, "n": len(times), "duration_s": duration, "rate_hz": rate,
                        "vmin": vmin, "vmax": vmax,
                        "changes": (vmax != vmin) if values else None,
                        "n_distinct": n_distinct,
                    }
                else:
                    row = f.readline()
                    while row and row != "\n":
                        row = f.readline()
            line = f.readline()

    print(f"total ChannelBlock sections: {total_blocks}")
    print(f"candidate channels matched (tc/tract/asr/eb/ebrake/engine_brake/map/abs/brake_bias): {len(detailed)}\n")
    for name in sorted(detailed):
        d = detailed[name]
        print(f"  {name} [{d['unit']}]: n={d['n']} duration={d['duration_s']:.1f}s rate~{d['rate_hz']:.2f}Hz "
              f"range=[{d['vmin']:.4g}, {d['vmax']:.4g}] changes_during_session={d['changes']} "
              f"n_distinct={d['n_distinct']}")


if __name__ == "__main__":
    main()
