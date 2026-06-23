from modules.csv_parser import parse_csv

data = parse_csv('C:/UNI/Bachelorarbeit/Data/Sample/Sample_Dubai.txt')

valid_laps = [l for l in data["laps"] if l["is_valid_for_analysis"]]
print(f'Laps analysed: {len(valid_laps)}')
print(f'Corners detected: {len(data["corners"])}')

for lap in data["laps"]:
    print(f'Lap {lap["lap_number"]}: time={lap["lap_time"]:.3f}s, '
          f'fastest={lap["is_fastest"]}, valid={lap["is_valid_for_analysis"]}, '
          f'warnings={lap["warnings"]}')

if data["corners"]:
    c = data["corners"][0]
    print(f'First corner: lap {c["lap_number"]}, #{c["corner_number"]}, '
          f'{c["speed_class"]} ({c["apex_speed"]:.0f} km/h), method={c["method"]}')
    print(f'Warnings: {c["warnings"]}')

    by_class = {}
    for c in data["corners"]:
        by_class[c["speed_class"]] = by_class.get(c["speed_class"], 0) + 1
    print(f'By class: {by_class}')

    corners_per_lap = {}
    for c in data["corners"]:
        corners_per_lap[c["lap_number"]] = corners_per_lap.get(c["lap_number"], 0) + 1
    print(f'Corners per lap: {corners_per_lap}')