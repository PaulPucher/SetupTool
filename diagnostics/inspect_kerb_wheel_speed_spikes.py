# Kerb-strike wheel-speed spike investigation. Read-only, Tier B
# (signal/data-engineering characterisation, no vehicle-dynamics claim
# of its own). No config change, no production path touched, nothing
# whitelisted beyond what modules/longitudinal_forces.py already reads.
#
# MOTIVATION: log_speed_fl/fr/rl/rr are now the kappa input for
# modules/longitudinal_stiffness.py's LS_ratio estimator (PLAN.md
# STEP 3). Kerb strikes are known (visually, from real data) to
# produce sharp wheel-speed transients -- if those transients leak
# past the existing kerb mask and into an LS regression window, the
# estimator would report a spurious dFx/dkappa slope from a sensor
# artefact, not real tyre behaviour. This script characterises the
# spikes, checks whether the mask catches them, and quantifies the
# consequence if it doesn't -- read-only throughout, no fix applied.
#
# KERB DETECTION MECHANISM (quoted from the actual code/config, not
# from memory): modules/stability_analysis.py _compute_kerb_mask_from_
# az computes raw = |az_g - kerb_baseline_g| > kerb_z_deviation_
# threshold_g, then dilates the raw mask by kerb_dilation_samples on
# each side (a rolling-OR). This is a VERTICAL-ACCELERATION detector
# -- it has no direct knowledge of wheel speed at all, which is
# exactly why this investigation is worth running: the two signals
# are physically related (a kerb strike jolts the car vertically AND
# briefly desyncs a wheel's rotational speed from ground speed) but
# nothing in the pipeline currently checks they agree.

import datetime
import os
import subprocess

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state
from modules.longitudinal_forces import estimate_slip_ratio, WHEEL_NAMES

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RUN_LABEL = datetime.date.today().isoformat() + "_kerb_wheel_speed_spikes"
OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "plots", RUN_LABEL)
os.makedirs(OUTPUT_DIR, exist_ok=True)


def _git_commit_info():
    try:
        commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO_ROOT, text=True).strip()
        dirty = bool(subprocess.check_output(["git", "status", "--porcelain"], cwd=REPO_ROOT, text=True).strip())
        return f"{commit} ({'dirty -- uncommitted changes present' if dirty else 'clean'})"
    except Exception as exc:
        return f"unavailable ({exc})"


with open(os.path.join(OUTPUT_DIR, "run_info.txt"), "w", encoding="utf-8") as f:
    f.write(f"run label: {RUN_LABEL}\n")
    f.write(f"date: {datetime.date.today().isoformat()}\n")
    f.write(f"git commit: {_git_commit_info()}\n")
    f.write("script: diagnostics/inspect_kerb_wheel_speed_spikes.py\n")

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)
se = params["stability_estimation"]
ls_cfg = params["longitudinal_stiffness"]
channels = data["channels"]

t = state["time"]
n = len(t)
sr = state["sample_rate_hz"]
v_ecu_kmh = state["v_mps"] * 3.6
moving = state["moving_mask"]
kerb_mask = state["kerb_mask"]
az_g = state["az_g"]

laps = data.get("laps", [])
valid_lap_numbers = {l["lap_number"] for l in laps if l.get("is_valid_for_analysis")}
racing_mask = np.zeros(n, dtype=bool)
for l in laps:
    if l["lap_number"] in valid_lap_numbers:
        racing_mask |= (t >= l["start_time"]) & (t <= l["end_time"])
# Deliberately NOT excluding kerb_mask here (unlike every other
# script's base_mask this session) -- the whole point is to look
# AT the kerb-flagged population too, not exclude it up front.
population_mask = moving & racing_mask

print("=" * 78)
print("KERB DETECTION MECHANISM (quoted from code + live config)")
print("=" * 78)
print("  modules/stability_analysis.py _compute_kerb_mask_from_az:")
print("    raw = |az_g - kerb_baseline_g| > kerb_z_deviation_threshold_g")
print("    mask = raw dilated by kerb_dilation_samples on each side (rolling-OR)")
print(f"  config/parameters.json stability_estimation: kerb_z_deviation_threshold_g="
      f"{se['kerb_z_deviation_threshold_g']}, kerb_baseline_g={se['kerb_baseline_g']}, "
      f"kerb_dilation_samples={se['kerb_dilation_samples']} ({se['kerb_dilation_samples']/sr*1000:.0f} ms "
      f"each side at {sr:.1f} Hz)")
print(f"  population_mask (moving & valid-lap racing time, kerb NOT excluded): n={int(population_mask.sum())}")
print()


def find_events(mask):
    d = np.diff(mask.astype(int))
    starts = list(np.where(d == 1)[0] + 1)
    ends = list(np.where(d == -1)[0] + 1)
    if mask[0]:
        starts = [0] + starts
    if mask[-1]:
        ends = ends + [len(mask)]
    return list(zip(starts, ends))


kerb_events_all = find_events(kerb_mask)
# Restrict to events that overlap the population (racing, moving) --
# pit-lane/out-of-lap kerb-like az spikes aren't relevant here.
kerb_events = [(s, e) for s, e in kerb_events_all if population_mask[s:e].any()]
durations = np.array([(e - s) / sr for s, e in kerb_events])
print("=" * 78)
print("PART 0 -- kerb event inventory")
print("=" * 78)
print(f"  total kerb-flagged runs in file: {len(kerb_events_all)}; overlapping the racing population: {len(kerb_events)}")
print(f"  duration (s): min={durations.min():.3f} p25={np.percentile(durations,25):.3f} "
      f"median={np.median(durations):.3f} p75={np.percentile(durations,75):.3f} max={durations.max():.3f}")
print(f"  kerb-flagged fraction of the racing population: {kerb_mask[population_mask].mean()*100:.2f}%")
print()

# --- per-wheel raw speeds and kappa -----------------------------------------


def interp(name):
    ch = channels.get(name)
    return np.interp(t, ch["time"], ch["data"])


v_fl, v_fr = interp("log_speed_fl"), interp("log_speed_fr")
v_rl, v_rr = interp("log_speed_rl"), interp("log_speed_rr")

rear_offset = ls_cfg["rear_rolling_radius_offset"]
v_floor_kmh = ls_cfg["min_speed_mps"] * 3.6
speed_ok = v_ecu_kmh >= v_floor_kmh


def wheel_kappa(v_wheel, correction=1.0):
    with np.errstate(divide="ignore", invalid="ignore"):
        return np.where(speed_ok, (v_wheel / correction - v_ecu_kmh) / v_ecu_kmh, np.nan)


kappa_wheel = {
    "fl": wheel_kappa(v_fl),
    "fr": wheel_kappa(v_fr),
    "rl": wheel_kappa(v_rl, 1.0 + rear_offset),
    "rr": wheel_kappa(v_rr, 1.0 + rear_offset),
}
v_wheel = {"fl": v_fl, "fr": v_fr, "rl": v_rl, "rr": v_rr}

# axle-mean kappa -- the ACTUAL input modules/longitudinal_stiffness.py consumes
slip = estimate_slip_ratio(state, channels, params)
kappa_axle = {"f": slip["kappa_f"], "r": slip["kappa_r"]}

print("=" * 78)
print("PART 1 -- spike characterisation, sample of events across the session")
print("=" * 78)

# Stratified sample: shortest, longest, median-duration, plus up to 5
# more spread evenly by time across the session -- diversity over
# convenience, not just "the first few".
order_by_dur = np.argsort(durations)
sample_idx = {order_by_dur[0], order_by_dur[-1], order_by_dur[len(order_by_dur) // 2]}
time_order = sorted(range(len(kerb_events)), key=lambda i: kerb_events[i][0])
for k in np.linspace(0, len(time_order) - 1, 8).astype(int):
    sample_idx.add(time_order[k])
sample_idx = sorted(sample_idx)

PRE_MARGIN_S, POST_MARGIN_S = 1.0, 3.0
SETTLE_BAND = 0.02   # |kappa| below this counts as "settled" (well inside normal traffic, see PART 2)
SETTLE_HOLD_S = 0.1  # must stay below SETTLE_BAND continuously this long to count as settled

event_records = []
for ei in sample_idx:
    s, e = kerb_events[ei]
    win_lo = max(0, s - int(PRE_MARGIN_S * sr))
    win_hi = min(n, e + int(POST_MARGIN_S * sr))
    print(f"  event #{ei} t=[{t[s]:.2f},{t[e-1]:.2f}]s dur={((e-s)/sr):.3f}s "
          f"v_ecu~{v_ecu_kmh[s:e].mean():.0f}km/h")
    row = {"idx": ei, "s": s, "e": e, "win_lo": win_lo, "win_hi": win_hi, "wheels": {}}
    for w in ("fl", "fr", "rl", "rr"):
        k = kappa_wheel[w]
        event_seg = k[s:e]
        finite_event = event_seg[np.isfinite(event_seg)]
        if finite_event.size == 0:
            print(f"    {w}: no finite samples in event window (not moving / no data)")
            row["wheels"][w] = None
            continue
        peak = finite_event[np.argmax(np.abs(finite_event))]
        half = 0.5 * abs(peak)
        above_half = np.abs(event_seg) >= half
        dur_half = np.nansum(above_half) / sr

        tail = k[e:win_hi]
        below = np.abs(tail) < SETTLE_BAND
        hold = int(round(SETTLE_HOLD_S * sr))
        settle_s = None
        if hold > 0 and len(below) >= hold:
            kernel = np.ones(hold, dtype=int)
            run = np.convolve(below.astype(int), kernel, mode="valid")
            settled_at = np.argmax(run == hold) if np.any(run == hold) else None
            if settled_at is not None and run[settled_at] == hold:
                settle_s = settled_at / sr
        print(f"    {w}: peak kappa={peak*100:+.2f}%  half-amplitude duration={dur_half*1000:.0f}ms  "
              f"post-event settle={'>' + str(POST_MARGIN_S) + 's (still ringing)' if settle_s is None else f'{settle_s*1000:.0f}ms'}")
        row["wheels"][w] = {"peak": peak, "dur_half": dur_half, "settle_s": settle_s}
    event_records.append(row)
print()

# Plot each sampled event: wheel kappa through the window, event span shaded.
for row in event_records:
    ei, s, e, win_lo, win_hi = row["idx"], row["s"], row["e"], row["win_lo"], row["win_hi"]
    fig, ax = plt.subplots(figsize=(9, 4))
    t_rel = t[win_lo:win_hi] - t[s]
    for w, color in zip(("fl", "fr", "rl", "rr"), ("#4FC3F7", "#1976D2", "#FFB74D", "#E65100")):
        ax.plot(t_rel, kappa_wheel[w][win_lo:win_hi] * 100, label=w, color=color, linewidth=1.2)
    ax.axvspan(0, (e - s) / sr, color="red", alpha=0.15, label="kerb-flagged window")
    ax.axhline(0, color="#888", linewidth=0.5)
    ax.set_xlabel("time relative to event start (s)")
    ax.set_ylabel("per-wheel kappa (%)")
    ax.set_title(f"kerb event #{ei}, t={t[s]:.1f}s, duration={((e-s)/sr):.3f}s")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fname = os.path.join(OUTPUT_DIR, f"event_{ei:03d}_wheel_kappa.png")
    fig.savefig(fname, dpi=120)
    plt.close(fig)
print(f"  {len(event_records)} event plots written to {OUTPUT_DIR}")
print()

# --- PART 2: anomaly threshold + mask alignment ------------------------------

print("=" * 78)
print("PART 2 -- anomaly threshold derivation and mask alignment")
print("=" * 78)

pooled_wheel_kappa_nonkerb = np.concatenate([
    np.abs(kappa_wheel[w][population_mask & ~kerb_mask]) for w in ("fl", "fr", "rl", "rr")
])
pooled_wheel_kappa_nonkerb = pooled_wheel_kappa_nonkerb[np.isfinite(pooled_wheel_kappa_nonkerb)]
pooled_wheel_kappa_kerb = np.concatenate([
    np.abs(kappa_wheel[w][population_mask & kerb_mask]) for w in ("fl", "fr", "rl", "rr")
])
pooled_wheel_kappa_kerb = pooled_wheel_kappa_kerb[np.isfinite(pooled_wheel_kappa_kerb)]

for label, vals in [("outside kerb mask", pooled_wheel_kappa_nonkerb), ("inside kerb mask", pooled_wheel_kappa_kerb)]:
    print(f"  |kappa_wheel| {label}: n={len(vals)} p50={np.percentile(vals,50)*100:.3f}% "
          f"p99={np.percentile(vals,99)*100:.3f}% p99.9={np.percentile(vals,99.9)*100:.3f}% "
          f"max={vals.max()*100:.3f}%")

# Gap-selected anomaly threshold (Tier B, data-derived, this diagnostic
# only -- NOT written to config): the ceiling of "normal" traffic
# outside the kerb mask (its own p99.9) vs the much larger population
# actually inside kerb-flagged windows. Threshold set at the non-kerb
# p99.9, rounded up to the next whole percent -- everything above it is
# rarer than 1-in-1000 among samples the mask does NOT already think
# are kerb-affected, a defensible "clearly abnormal" floor.
ANOMALY_THRESHOLD = float(np.ceil(np.percentile(pooled_wheel_kappa_nonkerb, 99.9) * 100)) / 100.0
print(f"  ANOMALY_THRESHOLD (gap-selected, non-kerb p99.9 rounded up) = {ANOMALY_THRESHOLD*100:.1f}%")
print()

anomalous_axle = (np.abs(kappa_axle["f"]) > ANOMALY_THRESHOLD) | (np.abs(kappa_axle["r"]) > ANOMALY_THRESHOLD)
anomalous_axle = anomalous_axle & population_mask & np.isfinite(kappa_axle["f"]) & np.isfinite(kappa_axle["r"])
leaked = anomalous_axle & ~kerb_mask
print(f"  anomalous samples (axle-mean |kappa_f| or |kappa_r| > {ANOMALY_THRESHOLD*100:.1f}%, "
      f"racing population): n={int(anomalous_axle.sum())}")
print(f"  of those, OUTSIDE the kerb mask ('leaked'): n={int(leaked.sum())} "
      f"({leaked.sum()/max(anomalous_axle.sum(),1)*100:.1f}% of anomalous samples)")
print()

# Which LS regression windows contain a leaked sample. Reproduces
# modules/longitudinal_stiffness.py _centered_slopes's own half_window
# formula exactly (read fresh from that module's logic, not re-derived
# independently) -- RAW (pre-Butterworth-filter) kappa checked against
# the window, since filtering happens on the whole array and a single
# window's "contamination" is best read off the raw signal that
# actually entered it; the production estimator's Butterworth stage
# then SMEARS a raw spike across neighbouring samples rather than
# removing it, so this likely UNDERSTATES true contamination, noted
# rather than modelled further here.
half_window = max(2, int(round(ls_cfg["regression_window_s"] * sr / 2.0)))
idx = np.arange(n)
w_start = np.maximum(0, idx - half_window)
w_stop = np.minimum(n, idx + half_window + 1)

leaked_idx = np.where(leaked)[0]
contaminated = np.zeros(n, dtype=bool)
for i in range(n):
    if w_stop[i] <= w_start[i]:
        continue
    lo, hi = w_start[i], w_stop[i]
    # any leaked sample index falling inside [lo, hi)?
    j = np.searchsorted(leaked_idx, lo)
    if j < len(leaked_idx) and leaked_idx[j] < hi:
        contaminated[i] = True

n_contaminated = int((contaminated & population_mask).sum())
print(f"  0.45s LS windows containing >=1 leaked sample: n={n_contaminated} "
      f"({n_contaminated/max(int(population_mask.sum()),1)*100:.2f}% of the racing population)")

UTIL_THRESH = 0.05  # kappa>=5%, the same "meaningful longitudinal utilisation" threshold already established
high_kappa_f = (np.abs(kappa_axle["f"]) >= UTIL_THRESH) & population_mask & np.isfinite(kappa_axle["f"])
high_kappa_r = (np.abs(kappa_axle["r"]) >= UTIL_THRESH) & population_mask & np.isfinite(kappa_axle["r"])
high_kappa_any = high_kappa_f | high_kappa_r
print(f"  high-kappa population (|kappa|>={UTIL_THRESH*100:.0f}%, front or rear): n={int(high_kappa_any.sum())}")
print(f"  of those, window contains a leaked sample: n={int((high_kappa_any & contaminated).sum())} "
      f"({(high_kappa_any & contaminated).sum()/max(int(high_kappa_any.sum()),1)*100:.1f}% of the high-kappa population)")
print()

# --- PART 3: impact on a handful of contaminated windows --------------------

print("=" * 78)
print("PART 3 -- impact of leaked samples on the regression slope")
print("=" * 78)

from modules.longitudinal_forces import estimate_longitudinal_forces
long_forces = estimate_longitudinal_forces(state, channels, params)

contaminated_high_kappa_idx = np.where(contaminated & high_kappa_any)[0]
# Spread the sample across the session rather than clustering.
if len(contaminated_high_kappa_idx) > 0:
    pick = contaminated_high_kappa_idx[np.linspace(0, len(contaminated_high_kappa_idx) - 1, min(5, len(contaminated_high_kappa_idx))).astype(int)]
else:
    pick = np.array([], dtype=int)


def ols_slope(x, y):
    if len(x) < 2:
        return np.nan
    xm, ym = np.mean(x), np.mean(y)
    denom = np.sum((x - xm) ** 2)
    if denom < 1e-12:
        return np.nan
    return np.sum((x - xm) * (y - ym)) / denom


for i in pick:
    lo, hi = w_start[i], w_stop[i]
    axle = "f" if abs(kappa_axle["f"][i]) >= abs(kappa_axle["r"][i]) else "r"
    kap = kappa_axle[axle][lo:hi]
    fx = long_forces[f"fx_{axle}_N"][lo:hi]
    fin = np.isfinite(kap) & np.isfinite(fx)
    kap, fx = kap[fin], fx[fin]
    window_leaked_mask = leaked[lo:hi][fin]
    if window_leaked_mask.sum() == 0 or (~window_leaked_mask).sum() < 2:
        continue
    slope_with = ols_slope(kap, fx)
    slope_without = ols_slope(kap[~window_leaked_mask], fx[~window_leaked_mask])
    pct_change = (slope_with - slope_without) / abs(slope_without) * 100 if abs(slope_without) > 1e-6 else float("nan")
    print(f"  index {i} (t={t[i]:.2f}s, axle {axle}): window n={len(kap)}, leaked samples in window={int(window_leaked_mask.sum())}")
    print(f"    slope WITH leaked sample(s):    {slope_with:,.0f} N per unit kappa")
    print(f"    slope WITHOUT leaked sample(s): {slope_without:,.0f} N per unit kappa")
    print(f"    change: {pct_change:+.1f}%")
print()

# --- PART 4: wheel-speed-based kerb detector, reverse comparison ------------

print("=" * 78)
print("PART 4 -- wheel-speed anomaly as an ALTERNATIVE kerb detector")
print("=" * 78)

max_wheel_kappa = np.nanmax(np.stack([np.abs(kappa_wheel[w]) for w in ("fl", "fr", "rl", "rr")]), axis=0)
raw_alt = max_wheel_kappa > ANOMALY_THRESHOLD
raw_alt = np.nan_to_num(raw_alt, nan=False).astype(bool)

dilation = int(se["kerb_dilation_samples"])
alt_mask = raw_alt.copy()
if dilation > 0:
    for shift in range(1, dilation + 1):
        alt_mask[shift:] |= raw_alt[:-shift]
        alt_mask[:-shift] |= raw_alt[shift:]

alt_events_all = find_events(alt_mask)
alt_events = [(s, e) for s, e in alt_events_all if population_mask[s:e].any()]
print(f"  wheel-speed-based events (same {dilation}-sample dilation as the az detector, "
      f"threshold={ANOMALY_THRESHOLD*100:.1f}%): n={len(alt_events)}")
print(f"  az-based (current) events, racing population: n={len(kerb_events)}")


def overlaps_any(ev, other_events):
    s, e = ev
    return any(not (e <= os_ or oe <= s) for os_, oe in other_events)


az_only = [ev for ev in kerb_events if not overlaps_any(ev, alt_events)]
alt_only = [ev for ev in alt_events if not overlaps_any(ev, kerb_events)]
both = [ev for ev in kerb_events if overlaps_any(ev, alt_events)]

print(f"  az events with a wheel-speed counterpart (overlap): n={len(both)} ({len(both)/max(len(kerb_events),1)*100:.1f}% of az events)")
print(f"  az-ONLY (az flags, wheel-speed does not): n={len(az_only)}")
print(f"  wheel-speed-ONLY (wheel-speed flags, az does not): n={len(alt_only)}")
print()

for label, evs in [("az-only", az_only[:3]), ("wheel-speed-only", alt_only[:3])]:
    print(f"  examples, {label}:")
    for s, e in evs:
        print(f"    t=[{t[s]:.2f},{t[min(e,n-1)]:.2f}]s dur={(e-s)/sr:.3f}s "
              f"az_peak={np.max(np.abs(az_g[s:e]-se['kerb_baseline_g'])) if az_g is not None else float('nan'):.3f}g "
              f"max_wheel_kappa_peak={max_wheel_kappa[s:e].max()*100:.2f}%")
    # plot up to 2 examples per category
    for s, e in evs[:2]:
        win_lo, win_hi = max(0, s - int(PRE_MARGIN_S * sr)), min(n, e + int(POST_MARGIN_S * sr))
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
        t_rel = t[win_lo:win_hi] - t[s]
        ax1.plot(t_rel, az_g[win_lo:win_hi], color="#B39DDB")
        ax1.axhline(se["kerb_baseline_g"], color="#888", linewidth=0.5, linestyle="--")
        ax1.set_ylabel("az (g)")
        ax1.set_title(f"{label} disagreement, t={t[s]:.1f}s")
        for w, color in zip(("fl", "fr", "rl", "rr"), ("#4FC3F7", "#1976D2", "#FFB74D", "#E65100")):
            ax2.plot(t_rel, kappa_wheel[w][win_lo:win_hi] * 100, label=w, color=color, linewidth=1.0)
        ax2.axhline(ANOMALY_THRESHOLD * 100, color="red", linewidth=0.5, linestyle="--")
        ax2.axhline(-ANOMALY_THRESHOLD * 100, color="red", linewidth=0.5, linestyle="--")
        ax2.set_ylabel("per-wheel kappa (%)")
        ax2.set_xlabel("time relative to event start (s)")
        ax2.legend(fontsize=8)
        fig.tight_layout()
        fname = os.path.join(OUTPUT_DIR, f"disagreement_{label}_{s:06d}.png")
        fig.savefig(fname, dpi=120)
        plt.close(fig)
print()

# --- Settle-time distribution across ALL racing kerb events (not just
# the Part 1 sample) -- feeds the mask-widening recommendation with a
# real percentile-based number instead of anecdotes from 10 events.
print("=" * 78)
print("PART 1b -- post-event settle time across ALL racing kerb events, per axle")
print("=" * 78)


def settle_time(k, event_end, win_hi):
    tail = k[event_end:win_hi]
    below = np.abs(tail) < SETTLE_BAND
    hold = int(round(SETTLE_HOLD_S * sr))
    if hold <= 0 or len(below) < hold:
        return None
    kernel = np.ones(hold, dtype=int)
    run = np.convolve(below.astype(int), kernel, mode="valid")
    if np.any(run == hold):
        return int(np.argmax(run == hold)) / sr
    return None


settle_front, settle_rear = [], []
for s, e in kerb_events:
    win_hi = min(n, e + int(POST_MARGIN_S * sr))
    for w in ("fl", "fr"):
        st = settle_time(kappa_wheel[w], e, win_hi)
        if st is not None:
            settle_front.append(st)
    for w in ("rl", "rr"):
        st = settle_time(kappa_wheel[w], e, win_hi)
        if st is not None:
            settle_rear.append(st)

for label, vals in [("front (fl+fr)", settle_front), ("rear (rl+rr)", settle_rear)]:
    vals = np.array(vals)
    n_uncapped = len(vals)
    n_never = sum(1 for s, e in kerb_events for w in (("fl", "fr") if "front" in label else ("rl", "rr"))
                   if settle_time(kappa_wheel[w], e, min(n, e + int(POST_MARGIN_S * sr))) is None)
    print(f"  {label}: settled-within-{POST_MARGIN_S}s n={n_uncapped} (never settled within window: {n_never}) -- "
          f"p50={np.median(vals)*1000:.0f}ms p75={np.percentile(vals,75)*1000:.0f}ms "
          f"p90={np.percentile(vals,90)*1000:.0f}ms max={vals.max()*1000:.0f}ms" if n_uncapped else f"  {label}: no data")
print(f"  current kerb_dilation_samples=5 -> {5/sr*1000:.0f}ms each side, for comparison")
print()
print(f"All plots written to {OUTPUT_DIR}")
