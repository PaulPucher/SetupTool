# Diagnostics

One-off read-only scripts used during development to produce methodology
evidence for the thesis. Not part of the app; run manually from the project
root (`python diagnostics/<script>.py`) against the sample data.

- **scan_channels.py** — inventories every channel actually present in a
  Cosworth Pi Toolbox file, regardless of the `channels.json` whitelist.
  Used to discover that the file's real GPS position channels are
  `log_gps_lat`/`log_gps_lon`, not the configured-but-absent
  `gpsa_lat`/`gpsa_long`/`VBOX_*` placeholders — evidence for the WP1 GPS
  channel scan and whitelist decision. Writes `channels_in_file.txt` next
  to itself.
- **inspect_kerb_signal.py** — prints `log_acc_z` deviation-from-baseline
  percentiles and the current kerb-mask flag rate. Basis for deriving
  `kerb_z_deviation_threshold_g` (thesis section: "Kerb/jump exclusion").
- **inspect_corner_distribution.py** — prints per-corner worst-phase
  CS_ratio and stability-margin percentiles, and how many corners each
  candidate threshold would flag. Basis for the data-driven severity
  classification thresholds (thesis section: "Two-signal AND-logic for
  severity classification").
