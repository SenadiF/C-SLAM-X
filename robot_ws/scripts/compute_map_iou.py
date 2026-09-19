#!/usr/bin/env python3
"""
compute_map_iou.py
===================
Compares two ROS-style saved maps (.pgm + .yaml pairs, as produced by
`ros2 run nav2_map_server map_saver_cli`) and reports the map-quality
metrics for Table 2: occupied-cell IoU, occupancy agreement / RMSE on
cells both maps actually know about, and how much of the ground truth's
footprint the candidate map also covers.

Usage:
  python3 compute_map_iou.py ground_truth_map.yaml candidate_map.yaml

Both maps are expected to use the same resolution (checked, warns if
not - every SLAM config in this workspace uses 0.05m, so this should
hold for any map saved from it). Alignment is done by converting each
ground-truth pixel to a world (x, y) point via its own origin/resolution
and looking up the same point in the candidate map via its own
origin/resolution - this handles the two maps having different sizes
and origins (normal, since a SLAM map's bounding box grows to fit
whatever was actually explored), but does not attempt sub-pixel image
registration, so a rotational/translational misalignment between the
two runs (e.g. from odometry drift) will show up as reduced IoU/agreement
rather than being corrected for.

No third-party dependencies beyond PyYAML, which ships with ROS2.
"""

import sys
import os
import math

try:
    import yaml
except ImportError:
    yaml = None


FREE = 0
OCCUPIED = 1
UNKNOWN = -1


def parse_pgm(path):
    """Parse a binary (P5) or ASCII (P2) PGM file into (width, height, maxval, pixels)."""
    with open(path, 'rb') as f:
        data = f.read()

    pos = 0

    def read_token():
        nonlocal pos
        while True:
            while pos < len(data) and data[pos:pos + 1].isspace():
                pos += 1
            if pos < len(data) and data[pos:pos + 1] == b'#':
                while pos < len(data) and data[pos:pos + 1] != b'\n':
                    pos += 1
                continue
            break
        start = pos
        while pos < len(data) and not data[pos:pos + 1].isspace():
            pos += 1
        return data[start:pos]

    magic = read_token()
    width = int(read_token())
    height = int(read_token())
    maxval = int(read_token())

    if magic == b'P5':
        pos += 1  # single whitespace byte separating header from binary data
        pixels = list(data[pos:pos + width * height])
    elif magic == b'P2':
        pixels = [int(tok) for tok in data[pos:].split()][:width * height]
    else:
        raise ValueError(f"Unsupported PGM magic number: {magic!r} in {path}")

    if len(pixels) != width * height:
        raise ValueError(
            f"{path}: expected {width * height} pixels, got {len(pixels)} - "
            f"truncated or corrupt file?"
        )

    return width, height, maxval, pixels


def _parse_yaml_minimal(path):
    """Fallback parser for the handful of keys map_saver_cli writes, in
    case PyYAML isn't importable for some reason."""
    meta = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#') or ':' not in line:
                continue
            key, _, value = line.partition(':')
            key, value = key.strip(), value.strip()
            if key == 'origin':
                meta[key] = [float(x.strip()) for x in value.strip('[]').split(',')]
            elif value.lower() in ('true', 'false'):
                meta[key] = value.lower() == 'true'
            else:
                try:
                    meta[key] = int(value)
                except ValueError:
                    try:
                        meta[key] = float(value)
                    except ValueError:
                        meta[key] = value.strip('"').strip("'")
    return meta


def load_map(yaml_path):
    meta = yaml.safe_load(open(yaml_path)) if yaml else _parse_yaml_minimal(yaml_path)

    pgm_path = meta['image']
    if not os.path.isabs(pgm_path):
        pgm_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), pgm_path)

    width, height, maxval, pixels = parse_pgm(pgm_path)

    resolution = float(meta['resolution'])
    origin_x, origin_y = float(meta['origin'][0]), float(meta['origin'][1])
    negate = int(meta.get('negate', 0))
    occupied_thresh = float(meta.get('occupied_thresh', 0.65))
    free_thresh = float(meta.get('free_thresh', 0.196))

    grid = [[UNKNOWN] * width for _ in range(height)]
    for row in range(height):
        for col in range(width):
            v = pixels[row * width + col]
            p = v / maxval if maxval > 0 else 0.0
            if negate:
                p = 1.0 - p
            occ_prob = 1.0 - p  # white (v=maxval) -> free; black (v=0) -> occupied
            if occ_prob > occupied_thresh:
                grid[row][col] = OCCUPIED
            elif occ_prob < free_thresh:
                grid[row][col] = FREE
            # else stays UNKNOWN

    return {
        'grid': grid, 'resolution': resolution,
        'origin_x': origin_x, 'origin_y': origin_y,
        'width': width, 'height': height,
    }


def world_xy_of_pixel(map_data, row, col):
    """World (x, y) of the center of pgm pixel (row, col), row 0 = top of image.
    map_saver_cli flips vertically when writing (pgm row 0 = highest y,
    matching image conventions; OccupancyGrid row 0 = lowest y)."""
    grid_row = map_data['height'] - 1 - row
    x = map_data['origin_x'] + (col + 0.5) * map_data['resolution']
    y = map_data['origin_y'] + (grid_row + 0.5) * map_data['resolution']
    return x, y


def value_at_world(map_data, x, y):
    resolution = map_data['resolution']
    col = int((x - map_data['origin_x']) / resolution)
    grid_row = int((y - map_data['origin_y']) / resolution)
    row = map_data['height'] - 1 - grid_row
    if row < 0 or row >= map_data['height'] or col < 0 or col >= map_data['width']:
        return UNKNOWN
    return map_data['grid'][row][col]


def compare(ground_truth, candidate):
    if abs(ground_truth['resolution'] - candidate['resolution']) > 1e-6:
        print(
            f"WARNING: resolutions differ ({ground_truth['resolution']} vs "
            f"{candidate['resolution']}) - comparison will be approximate.\n"
        )

    tp_occ = fp_occ = fn_occ = 0
    both_known = matching_known = 0
    sq_error_sum = 0
    error_samples = 0
    gt_known_cells = 0
    cand_known_in_gt_footprint = 0
    total_cells = ground_truth['height'] * ground_truth['width']

    for row in range(ground_truth['height']):
        for col in range(ground_truth['width']):
            gt_val = ground_truth['grid'][row][col]
            x, y = world_xy_of_pixel(ground_truth, row, col)
            cand_val = value_at_world(candidate, x, y)

            if gt_val != UNKNOWN:
                gt_known_cells += 1
            if cand_val != UNKNOWN:
                cand_known_in_gt_footprint += 1

            gt_occ = (gt_val == OCCUPIED)
            cand_occ = (cand_val == OCCUPIED)

            if gt_val != UNKNOWN and cand_val != UNKNOWN:
                both_known += 1
                if gt_occ == cand_occ:
                    matching_known += 1
                error = 0.0 if gt_occ == cand_occ else 1.0
                sq_error_sum += error * error
                error_samples += 1

            if gt_occ and cand_occ:
                tp_occ += 1
            elif cand_occ and not gt_occ:
                fp_occ += 1
            elif gt_occ and not cand_occ:
                fn_occ += 1

    denom = tp_occ + fp_occ + fn_occ
    iou_occupied = tp_occ / denom if denom > 0 else float('nan')
    agreement = matching_known / both_known if both_known > 0 else float('nan')
    rmse = math.sqrt(sq_error_sum / error_samples) if error_samples > 0 else float('nan')

    return {
        'iou_occupied_cells': iou_occupied,
        'occupancy_agreement_on_jointly_known_cells': agreement,
        'rmse_occupancy_on_jointly_known_cells': rmse,
        'jointly_known_cells': both_known,
        'ground_truth_known_cells': gt_known_cells,
        'ground_truth_total_cells': total_cells,
        'ground_truth_coverage_pct': 100.0 * gt_known_cells / total_cells,
        'candidate_known_cells_in_gt_footprint': cand_known_in_gt_footprint,
        'candidate_coverage_of_gt_footprint_pct': 100.0 * cand_known_in_gt_footprint / total_cells,
    }


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)

    ground_truth = load_map(sys.argv[1])
    candidate = load_map(sys.argv[2])

    results = compare(ground_truth, candidate)

    print(f"\n=== Map comparison: {sys.argv[2]}  vs ground truth {sys.argv[1]} ===")
    for k, v in results.items():
        if isinstance(v, float):
            print(f"  {k:44s}: {v:.4f}")
        else:
            print(f"  {k:44s}: {v}")
    print()


if __name__ == '__main__':
    main()
