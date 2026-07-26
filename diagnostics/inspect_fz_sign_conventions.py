# WP5b(b) phase 1, turn (a) verification: ax (longitudinal-transfer
# direction) and ay (left/right loading) sign checks against real Dubai
# data, for the chair-identical Fz formulas in
# modules.stability_analysis.estimate_vertical_loads
# (docs/literature/data_handler.py:1548-1607, internal reference).
# Report-only -- estimate_vertical_loads is not called from any pipeline
# or UI path yet (Module 6/UI wiring is WP5b(b) phase 1 turn (b)).

import numpy as np
from modules.csv_parser import parse_csv
from modules.stability_analysis import load_parameters, prepare_vehicle_state
from modules.geo import project_latlon_to_xy, compute_gps_origin

data = parse_csv("C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt")
params = load_parameters()
state = prepare_vehicle_state(data["channels"], params)
moving = state["moving_mask"]

print("=== ax sign convention: braking vs accelerating (Dubai, all moving samples) ===")
brake = state["brake_f_bar"]
throttle = state["throttle_pct"]
ax = state["ax_mps2"]

brake_thresh = np.nanpercentile(brake[moving], 90)
heavy_brake = moving & (brake > brake_thresh)
heavy_accel = moving & (throttle > 80) & (brake < 1.0)

med_ax_brake = np.median(ax[heavy_brake])
med_ax_accel = np.median(ax[heavy_accel])

print(f"Heavy-braking samples (log_pbrake_f > p90={brake_thresh:.1f} bar): {heavy_brake.sum()}")
print(f"  median ax_mps2: {med_ax_brake:+.2f}")
print(f"Heavy-accel samples (ecu_aps>80%, brake<1 bar): {heavy_accel.sum()}")
print(f"  median ax_mps2: {med_ax_accel:+.2f}")

print(
    "\nChair formula: dfz_long_transfer_N = m*ax*h_cog/wb; "
    "fz_f_N = static_f + aero_f - dfz_long_transfer_N. "
    "Front-loading under braking (nose dips down) requires "
    "dfz_long_transfer_N < 0 during braking, i.e. ax < 0 during braking."
)
if med_ax_brake < 0 and med_ax_accel > 0:
    print(
        f"  -> MATCH: ax negative under braking ({med_ax_brake:+.2f}), positive "
        f"under acceleration ({med_ax_accel:+.2f}). The formula as written loads "
        f"the front under braking and the rear under acceleration -- both "
        f"physically correct. No sign flip needed."
    )
elif med_ax_brake > 0 and med_ax_accel < 0:
    print(
        f"  -> MISMATCH: ax positive under braking ({med_ax_brake:+.2f}), negative "
        f"under acceleration ({med_ax_accel:+.2f}) -- the OPPOSITE of the chair's "
        f"assumed convention. As written the formula would UNLOAD the front under "
        f"braking. ax_mps2's sign must be flipped (or the formula's +/- swapped) "
        f"before fz_f_N/fz_r_N are trusted."
    )
else:
    print(
        f"  -> INCONCLUSIVE: braking ({med_ax_brake:+.2f}) and accel "
        f"({med_ax_accel:+.2f}) medians don't form a clean opposite-sign pair -- "
        f"needs a closer look before trusting either."
    )

print("\n=== ay sign convention: left/right loading vs GPS-derived turn direction ===")
corners = data.get("corners", [])
gps_lat_ch = data["channels"].get("log_gps_lat")
gps_lon_ch = data["channels"].get("log_gps_lon")
origin_lat, origin_lon = compute_gps_origin(gps_lat_ch, gps_lon_ch)

if origin_lat is None or not corners:
    print("GPS or corner data unavailable -- cannot run the ay check.")
else:
    lat_t, lat_d = gps_lat_ch["time"], gps_lat_ch["data"]
    lon_t, lon_d = gps_lon_ch["time"], gps_lon_ch["data"]

    # Widest-|apex_lateral_g| corner: clearest, least ambiguous turn-direction
    # signal (sharpest, most sustained curvature -> cleanest heading-change
    # measurement from the GPS trace, least sensitive to GPS noise).
    corner = max(corners, key=lambda c: abs(c.get("apex_lateral_g", 0.0)))
    t0 = corner["segments"]["entry_1_brake"][0]
    t1 = corner["segments"]["exit_5"][1]

    print(
        f"Selected corner: lap {corner['lap_number']} corner {corner['corner_number']} "
        f"({corner['speed_class']}, apex_lateral_g={corner['apex_lateral_g']:.2f}), "
        f"window [{t0:.2f}, {t1:.2f}]s"
    )

    # Ground truth: turn direction from the GPS trace itself, independent of
    # any onboard sensor's sign convention. x=east, y=north (modules/geo.py).
    # Standard math heading (CCW from +x/east): a right (clockwise) turn is a
    # net NEGATIVE heading change; a left (CCW) turn is net POSITIVE.
    win_t = lat_t[(lat_t >= t0) & (lat_t <= t1)]
    if len(win_t) < 5:
        print("  Too few GPS samples in this corner's window -- cannot determine turn direction.")
    else:
        win_lat = np.interp(win_t, lat_t, lat_d)
        win_lon = np.interp(win_t, lon_t, lon_d)
        x, y = project_latlon_to_xy(win_lat, win_lon, origin_lat, origin_lon)
        dx, dy = np.diff(x), np.diff(y)
        heading = np.unwrap(np.arctan2(dy, dx))
        net_heading_change_deg = np.degrees(heading[-1] - heading[0])
        turn = "RIGHT (clockwise)" if net_heading_change_deg < 0 else "LEFT (counter-clockwise)"
        print(f"  GPS-derived net heading change: {net_heading_change_deg:+.1f} deg -> {turn}")

        win_mask = (state["time"] >= t0) & (state["time"] <= t1)
        mean_ay = np.mean(state["ay_mps2"][win_mask])
        mean_steer = np.mean(state["steer_sw_rad"][win_mask]) * 180.0 / np.pi
        print(f"  mean ay_mps2 over window: {mean_ay:+.2f}   mean steer_sw_deg: {mean_steer:+.1f}")

        print(
            "\nChair formula: lateral_transfer_front = m*ay*h_cog/front_track; "
            "fz_fl_N = fz_f_N/2 - transfer/2, fz_fr_N = fz_f_N/2 + transfer/2 "
            "(positive ay -> RIGHT tire loads more). "
            "Physically, in a right turn the LEFT (outside) tires load more; "
            "in a left turn the RIGHT (outside) tires load more."
        )
        loaded_side_by_formula = "RIGHT" if mean_ay > 0 else "LEFT"
        expected_loaded_side = "LEFT" if "RIGHT" in turn else "RIGHT"
        print(f"  Formula (as written) loads: {loaded_side_by_formula}   Physically expected: {expected_loaded_side}")
        if loaded_side_by_formula == expected_loaded_side:
            print("  -> MATCH: fz_fl_N/fz_fr_N as written load the physically-correct (outside) side. No sign flip needed.")
        else:
            print(
                "  -> MISMATCH: fz_fl_N/fz_fr_N as written load the INSIDE tire under load, "
                "the opposite of physical expectation. ay_mps2's sign convention (or the "
                "formula's L/R assignment) must be flipped before per-wheel Fz is trusted."
            )
