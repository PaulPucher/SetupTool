# Characterise the single largest raw-EKF |beta|
# excursion (14.12 deg, lap 4, t=884.224s, inside C2's bracket, not
# C14 -- see the previous turn's plotting report). Read-only, no
# config/production change. Plain per-sample listing, t=883.0-885.5s:
# time, s_m, EKF raw beta, kinematic beta, ay, yaw rate, delta_f, speed,
# per-channel NIS (yaw rate/ay separately), diverged_mask. No
# interpretation forced -- states which of (a) genuine transient
# sideslip or (b) numerical excursion the data supports, or that it is
# inconclusive, after the numbers are printed.

import numpy as np

from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state, estimate_sideslip
from diagnostics.sideslip_ekf_dugoff import estimate_sideslip_ekf_dugoff

RAW_FILE = "C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt"
T_START, T_END = 883.0, 885.5

data = parse_csv(RAW_FILE)
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)

t = state["time"]
s_m = state.get("s_m")
ay = state["ay_mps2"]
yaw_rate_deg_s = np.degrees(state["yaw_rate_radps"])
delta_f_deg = np.degrees(state["delta_f_rad"])
v_kmh = state["v_mps"] * 3.6

beta_kinematic_deg = np.degrees(estimate_sideslip(state, params))
result = estimate_sideslip_ekf_dugoff(state, params)
beta_raw_deg = np.degrees(result["beta"])
diverged = result["diverged_mask"]
innovation = result["innovation"]
S_diag = result["S_diag"]
nis_yaw = innovation[:, 0] ** 2 / S_diag[:, 0]
nis_ay = innovation[:, 1] ** 2 / S_diag[:, 1]

idx = np.where((t >= T_START) & (t <= T_END))[0]

print("=" * 100)
print(f"max-|beta| excursion characterisation, t={T_START}-{T_END}s")
print("=" * 100)
print(f"{'t(s)':>9} {'s_m':>8} {'beta_raw':>9} {'beta_kin':>9} {'ay':>8} {'yaw_rate':>9} "
      f"{'delta_f':>8} {'v(km/h)':>8} {'nis_yaw':>10} {'nis_ay':>10} {'diverged':>9}")
for i in idx:
    print(f"{t[i]:9.3f} {s_m[i]:8.1f} {beta_raw_deg[i]:9.3f} {beta_kinematic_deg[i]:9.3f} "
          f"{ay[i]:8.3f} {yaw_rate_deg_s[i]:9.3f} {delta_f_deg[i]:8.3f} {v_kmh[i]:8.2f} "
          f"{nis_yaw[i]:10.2f} {nis_ay[i]:10.2f} {str(bool(diverged[i])):>9}")
print()

i_peak = idx[np.argmax(np.abs(beta_raw_deg[idx]))]
print(f"peak sample: t={t[i_peak]:.3f}s  beta_raw={beta_raw_deg[i_peak]:.3f} deg  "
      f"beta_kinematic={beta_kinematic_deg[i_peak]:.3f} deg  ay={ay[i_peak]:.3f} m/s^2  "
      f"yaw_rate={yaw_rate_deg_s[i_peak]:.3f} deg/s  delta_f={delta_f_deg[i_peak]:.3f} deg  "
      f"v={v_kmh[i_peak]:.2f} km/h")
