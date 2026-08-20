# Rolling-radial/combined-slip follow-up, QUEUED ITEM 2 + ITEM 4: full
# raw-channel-inventory sweep for anything plausibly slip- or
# traction-control-related, by name (no description field exists in
# the Pi Toolbox {ChannelBlock} format beyond the header itself -- Time
# \t ChannelName[unit] -- so "by name" is the whole search space).
# Read-only, Tier B, nothing whitelisted, no config written.
#
# cp1252 decoding (not utf-8/errors="replace", which the WP2b-1 full
# channel census found mangles degree-sign units -- thesis_notes.md
# "Full channel census + targeted verification").
#
# Two passes: (1) cheap name-only scan of the ENTIRE file (~2600+
# channels) to find keyword matches without parsing any data; (2) for
# matched channels only, parse their data to report unit, sample count,
# and value range over the whole session.

import re

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"

KEYWORDS = ["slip", "slp", "kappa", "lambda", "spin", "trac", "tc", "asr",
            "wheelslip", "ref", "target", "delta", "diff", "err"]
KEYWORD_RE = re.compile("|".join(re.escape(k) for k in KEYWORDS), re.IGNORECASE)


def scan_names(file_path):
    with open(file_path, "r", encoding="cp1252", errors="replace") as f:
        lines = f.readlines()
    channels = []
    i, n = 0, len(lines)
    while i < n:
        if lines[i].strip() == "{ChannelBlock}":
            i += 1
            if i < n:
                header_parts = lines[i].strip().split("\t")
                if len(header_parts) == 2:
                    raw_name = header_parts[1].strip()
                    channel_name = raw_name[:raw_name.index('[')].strip() if '[' in raw_name else raw_name
                    i += 1
                    n_samples = 0
                    while i < n and not lines[i].strip().startswith("{"):
                        if lines[i].strip():
                            n_samples += 1
                        i += 1
                    channels.append((channel_name, raw_name, n_samples))
                    continue
        i += 1
    return channels


print("=" * 78)
print("PASS 1 -- full channel-name inventory")
print("=" * 78)
channels = scan_names(RAW_FILE)
print(f"Total channels found: {len(channels)}")

matches = [(name, raw, n_s) for name, raw, n_s in channels if KEYWORD_RE.search(name)]
matches.sort(key=lambda c: c[0].lower())
print(f"Keyword matches ({'/'.join(KEYWORDS)}): {len(matches)}")
print()
for name, raw, n_s in matches:
    print(f"  {name:<28} header={raw:<40} n_samples={n_s}")
print()

print("=" * 78)
print("PASS 2 -- value range for each matched channel (whole session, no masking)")
print("=" * 78)


def read_many(file_path, wanted_names):
    wanted = set(wanted_names)
    out = {}
    with open(file_path, "r", encoding="cp1252", errors="replace") as f:
        lines = f.readlines()
    i, n = 0, len(lines)
    while i < n:
        if lines[i].strip() == "{ChannelBlock}":
            i += 1
            if i < n:
                header_parts = lines[i].strip().split("\t")
                if len(header_parts) == 2:
                    raw_name = header_parts[1].strip()
                    channel_name = raw_name[:raw_name.index('[')].strip() if '[' in raw_name else raw_name
                    i += 1
                    if channel_name in wanted:
                        values = []
                        while i < n and not lines[i].strip().startswith("{"):
                            raw_line = lines[i].strip()
                            if raw_line:
                                parts = raw_line.split("\t")
                                if len(parts) == 2:
                                    try:
                                        values.append(float(parts[1].replace(",", ".")))
                                    except ValueError:
                                        pass
                            i += 1
                        out[channel_name] = values
                        continue
                    else:
                        while i < n and not lines[i].strip().startswith("{"):
                            i += 1
                        continue
        i += 1
    return out


matched_names = [name for name, raw, n_s in matches]
values_by_name = read_many(RAW_FILE, matched_names)
for name, raw, n_s in matches:
    vals = values_by_name.get(name, [])
    if not vals:
        print(f"  {name:<28} header={raw:<40} NO NUMERIC DATA (ASCII/text channel or empty)")
        continue
    vmin, vmax = min(vals), max(vals)
    print(f"  {name:<28} header={raw:<40} n={len(vals):6d}  range=[{vmin:.4f}, {vmax:.4f}]")

print()
print("=" * 78)
print("ITEM 4 -- TC (traction control) specific channels, from the matches above")
print("=" * 78)
tc_related = [c for c in matches if re.search(r"\btc\b|trac|asr", c[0], re.IGNORECASE)]
if tc_related:
    for name, raw, n_s in tc_related:
        print(f"  {name:<28} header={raw}")
else:
    print("  none of the keyword matches above look TC-specific by name; "
          "see full match list in PASS 1 for anything else worth judging.")
