from modules.csv_parser import parse_csv
from modules.stability_analysis import (
    load_parameters,
    prepare_vehicle_state,
    estimate_sideslip,
    estimate_slip_angles,
    estimate_lateral_forces,
    estimate_cornering_stiffness,
    estimate_yaw_moment_stability,
    summarise_corners,
)
import numpy as np

data = parse_csv('C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt')
params = load_parameters()
state = prepare_vehicle_state(data['channels'], params)

if state:
    print(f"Time base: {len(state['time'])} samples at {state['sample_rate_hz']:.1f} Hz")
    print(f"Speed range: {np.min(state['v_mps']):.1f} - {np.max(state['v_mps']):.1f} m/s")
    print(f"Yaw rate range: {np.min(state['yaw_rate_radps']):.3f} - {np.max(state['yaw_rate_radps']):.3f} rad/s")
    print(f"Front wheel angle range: {np.min(state['delta_f_rad'])*180/np.pi:.1f} - {np.max(state['delta_f_rad'])*180/np.pi:.1f} deg")
    print(f"Moving samples: {state['moving_mask'].sum()} / {len(state['moving_mask'])}")
    print(f"Accuracy levels: {state['accuracy_level']}")

    beta = estimate_sideslip(state, params)
    moving = state['moving_mask']

    print(f"\nSideslip angle (moving samples only):")
    print(f"  Min: {np.min(beta[moving])*180/np.pi:.2f} deg")
    print(f"  Max: {np.max(beta[moving])*180/np.pi:.2f} deg")
    print(f"  Mean abs: {np.mean(np.abs(beta[moving]))*180/np.pi:.2f} deg")
    print(f"  Std: {np.std(beta[moving])*180/np.pi:.2f} deg")

    slip = estimate_slip_angles(state, beta, params)
    af = slip["alpha_f_filt"][moving]
    ar = slip["alpha_r_filt"][moving]

    print(f"\nFront slip angle (moving, filtered):")
    print(f"  Min: {np.min(af)*180/np.pi:.2f} deg")
    print(f"  Max: {np.max(af)*180/np.pi:.2f} deg")
    print(f"  Mean abs: {np.mean(np.abs(af))*180/np.pi:.2f} deg")
    print(f"\nRear slip angle (moving, filtered):")
    print(f"  Min: {np.min(ar)*180/np.pi:.2f} deg")
    print(f"  Max: {np.max(ar)*180/np.pi:.2f} deg")
    print(f"  Mean abs: {np.mean(np.abs(ar))*180/np.pi:.2f} deg")

    forces = estimate_lateral_forces(state, params)
    cs = estimate_cornering_stiffness(slip, forces, state, params)

    valid_f = ~np.isnan(cs["CS_ratio_f"]) & moving
    valid_r = ~np.isnan(cs["CS_ratio_r"]) & moving

    print(f"\nFront lateral force (moving):")
    print(f"  Min: {np.min(forces['Fy_f_filt'][moving]):.0f} N")
    print(f"  Max: {np.max(forces['Fy_f_filt'][moving]):.0f} N")

    print(f"\nFront cornering stiffness (valid samples):")
    C_f_valid = cs["C_alpha_f"][valid_f]
    print(f"  Mean: {np.nanmean(C_f_valid):.0f} N/rad")
    print(f"  Min:  {np.nanmin(C_f_valid):.0f} N/rad")
    print(f"  Max:  {np.nanmax(C_f_valid):.0f} N/rad")

    print(f"\nRear cornering stiffness (valid samples):")
    C_r_valid = cs["C_alpha_r"][valid_r]
    print(f"  Mean: {np.nanmean(C_r_valid):.0f} N/rad")
    print(f"  Min:  {np.nanmin(C_r_valid):.0f} N/rad")
    print(f"  Max:  {np.nanmax(C_r_valid):.0f} N/rad")

    print(f"\nCS ratio front (valid samples: {valid_f.sum()}):")
    print(f"  Mean: {np.nanmean(cs['CS_ratio_f'][valid_f]):.3f}")
    print(f"  Min:  {np.nanmin(cs['CS_ratio_f'][valid_f]):.3f}")
    print(f"  Samples below 0.5: {(cs['CS_ratio_f'][valid_f] < 0.5).sum()}")
    print(f"  Samples below 0.0: {(cs['CS_ratio_f'][valid_f] < 0.0).sum()}")

    print(f"\nCS ratio rear (valid samples: {valid_r.sum()}):")
    print(f"  Mean: {np.nanmean(cs['CS_ratio_r'][valid_r]):.3f}")
    print(f"  Min:  {np.nanmin(cs['CS_ratio_r'][valid_r]):.3f}")
    print(f"  Samples below 0.5: {(cs['CS_ratio_r'][valid_r] < 0.5).sum()}")
    print(f"  Samples below 0.0: {(cs['CS_ratio_r'][valid_r] < 0.0).sum()}")

    unclipped_f = valid_f & (cs['CS_ratio_f'] < 1.0) & (cs['CS_ratio_f'] > 0.01)
    unclipped_r = valid_r & (cs['CS_ratio_r'] < 1.0) & (cs['CS_ratio_r'] > 0.01)
    ref_f_implied = np.nanmedian(cs['C_alpha_f'][unclipped_f] / cs['CS_ratio_f'][unclipped_f])
    ref_r_implied = np.nanmedian(cs['C_alpha_r'][unclipped_r] / cs['CS_ratio_r'][unclipped_r])
    print(f"\nImplied C_linear_ref (back-calculated from CS_ratio):")
    print(f"  Front: {ref_f_implied:.0f} N/rad  (expected 80k-180k)")
    print(f"  Rear:  {ref_r_implied:.0f} N/rad  (expected 80k-180k)")

    stab = estimate_yaw_moment_stability(state, beta, params, data.get("laps", []))

    valid = stab["stability_valid"] & moving
    s_obs = stab["stability_observed_Nm_per_deg"][valid]
    mz = stab["mz_inertial_Nm"][moving]
    psi_dd = stab["yaw_accel_filtered_radps2"][moving]

    print(f"\nYaw acceleration filtered (moving):")
    print(f"  Min: {np.min(psi_dd):.3f} rad/s²")
    print(f"  Max: {np.max(psi_dd):.3f} rad/s²")
    print(f"  Mean abs: {np.mean(np.abs(psi_dd)):.3f} rad/s²")

    print(f"\nInertial yaw moment (moving):")
    print(f"  Min: {np.min(mz):.0f} Nm")
    print(f"  Max: {np.max(mz):.0f} Nm")
    print(f"  Mean abs: {np.mean(np.abs(mz)):.0f} Nm")
    print(f"  Iz used: {stab['iz_used_kgm2']} kg·m²")

    print(f"\nStability observed (valid samples: {valid.sum()} / {moving.sum()}):")
    if valid.sum() > 0:
        print(f"  Mean: {np.mean(s_obs):.1f} Nm/deg")
        print(f"  Median: {np.median(s_obs):.1f} Nm/deg")
        print(f"  Min:  {np.min(s_obs):.1f} Nm/deg")
        print(f"  Max:  {np.max(s_obs):.1f} Nm/deg")
        print(f"  Samples positive (stabilising):  {(s_obs > 0).sum()}")
        print(f"  Samples near-zero (|x|<50):      {(np.abs(s_obs) < 50).sum()}")
        print(f"  Samples negative (destabilising): {(s_obs < 0).sum()}")

    corners = data.get("corners", [])
    print(f"\n=== Per-corner summary ===")
    print(f"Detected corners: {len(corners)}")

    print(f"\n=== Stable corner id clustering (per-cluster membership) ===")
    stable_ids = {c["stable_corner_id"] for c in corners if c["stable_corner_id"] is not None}
    print(f"Unique stable corners: {len(stable_ids)}")
    n_valid_laps = len([l for l in data.get("laps", []) if l.get("is_valid_for_analysis")])
    by_id = {}
    for c in corners:
        by_id.setdefault(c["stable_corner_id"], []).append(c)
    n_singleton = 0
    n_full = 0
    n_partial = 0
    for cid in sorted(by_id.keys()):
        members = sorted(by_id[cid], key=lambda c: c["lap_number"])
        laps_present = sorted({m["lap_number"] for m in members})
        if len(members) == 1:
            n_singleton += 1
        elif len(members) == n_valid_laps:
            n_full += 1
        else:
            n_partial += 1
        print(f"C{cid} -- {len(members)} member(s), laps {laps_present}:")
        for m in members:
            tags = ""
            if "compound_corner" in m["warnings"]:
                tags += "  [compound_corner]"
            if "straddles_adjacent_corners" in m["warnings"]:
                tags += "  [straddles_adjacent_corners]"
            print(f"    lap={m['lap_number']}  corner={m['corner_number']}  "
                  f"bracket=[{m['bracket_start_m']:8.1f},{m['bracket_end_m']:8.1f}]m  "
                  f"apex_dist={m['apex_lap_distance_m']:8.1f}m{tags}")

    print(f"\nCount justification: {len(by_id)} clusters = {n_full} full (all {n_valid_laps} "
          f"valid laps) + {n_partial} partial (compound-straddle splits, not all laps take "
          f"this section the same way) + {n_singleton} singleton (a genuine extra event "
          f"unique to one lap).")

    if corners:
        summaries = summarise_corners(corners, cs, stab, state)

        for s in summaries[:3]:
            print(f"\nLap {s['lap_number']}  Corner {s['corner_number']}  "
                  f"({s['speed_class']}, apex_v={s['apex_speed']:.1f} km/h, "
                  f"t={s['apex_time']:.2f}s)")
            for phase in ["entry_1_brake", "entry_2_turnin", "apex_3", "exit_4", "exit_5"]:
                p = s["phases"][phase]
                csf = p["cs_ratio_f"]
                csr = p["cs_ratio_r"]
                sob = p["stability_observed_Nm_per_deg"]
                print(f"  {phase:>16}  n={p['n_samples']:3d}  "
                      f"valid_stab={p['valid_fraction_stab']*100:4.0f}%  | "
                      f"CSf={csf['median']:.2f} [{csf['p25']:.2f}..{csf['p75']:.2f}]  "
                      f"CSr={csr['median']:.2f} [{csr['p25']:.2f}..{csr['p75']:.2f}]  "
                      f"Stab={sob['median']:>6.0f} [{sob['p25']:>6.0f}..{sob['p75']:>6.0f}] Nm/deg")

else:
    print("State preparation failed - check required channels")