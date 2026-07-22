# Scan a Cosworth Pi Toolbox ASCII file and list every channel it contains,
# regardless of channels.json whitelist. Writes to a file so the full list
# is visible.

import os

src = r"C:\UNI\Bachelorarbeit\Data\Sample\Sample_Dubai.txt"
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "channels_in_file.txt")

if not os.path.exists(src):
    print(f"File not found: {src}")
    raise SystemExit(1)

with open(src, "r", encoding="utf-8", errors="replace") as f:
    lines = f.readlines()

channels = []
i = 0
n = len(lines)

while i < n:
    if lines[i].strip() == "{ChannelBlock}":
        i += 1
        if i < n:
            header_parts = lines[i].strip().split("\t")
            if len(header_parts) == 2:
                raw_name = header_parts[1].strip()
                channel_name = (
                    raw_name[:raw_name.index('[')].strip()
                    if '[' in raw_name else raw_name
                )
                # Count samples in the block (informational)
                i += 1
                n_samples = 0
                while i < n and not lines[i].strip().startswith("{"):
                    if lines[i].strip():
                        n_samples += 1
                    i += 1
                channels.append((channel_name, raw_name, n_samples))
                continue
    i += 1

# Sort alphabetically for easy scanning
channels.sort(key=lambda c: c[0].lower())

with open(out, "w", encoding="utf-8") as f:
    f.write(f"Source: {src}\n")
    f.write(f"Channels found: {len(channels)}\n\n")
    f.write(f"{'channel_name':<40} {'header_label':<60} samples\n")
    f.write(f"{'-'*40} {'-'*60} {'-'*7}\n")
    for name, raw, n_s in channels:
        f.write(f"{name:<40} {raw:<60} {n_s}\n")

print(f"Wrote {len(channels)} channels to {out}")