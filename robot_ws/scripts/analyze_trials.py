#!/usr/bin/env python3
"""
analyze_trials.py
==================
Aggregates repeated-trial CSVs produced by metrics_logger.py (one row per
trial, appended automatically each run by run_trials.sh) into mean +/-
standard deviation per numeric column - the format needed for a results
table backed by multiple trials, not a single cherry-picked run.

Usage:
  python3 analyze_trials.py ~/comparison_results/*.csv
  python3 analyze_trials.py ~/comparison_results/single_robot_robot1.csv \
                             ~/comparison_results/two_robot_known_robot1.csv

No third-party dependencies - stdlib only.
"""

import sys
import csv
import statistics


def load_rows(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def to_float(value):
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    # NaN passes float() (e.g. a trial where no goal was ever reached
    # legitimately writes 'nan' for avg_time_to_goal_s) but isn't a real
    # number for averaging purposes, and crashes statistics.stdev() on
    # Python 3.12 if let through - treat it the same as non-numeric.
    if parsed != parsed:
        return None
    return parsed


def summarize(path):
    try:
        rows = load_rows(path)
    except FileNotFoundError:
        print(f"\n=== {path} ===\n  file not found")
        return

    if not rows:
        print(f"\n=== {path} ===\n  no rows")
        return

    strategy = rows[0].get('strategy', '?')
    robot = rows[0].get('robot_name', '?')
    n = len(rows)

    print(f"\n=== {path}  (strategy={strategy}, robot={robot}, n={n} trials) ===")

    skip_fields = ('strategy', 'robot_name')
    numeric_fields = [k for k in rows[0].keys() if k not in skip_fields]

    for field in numeric_fields:
        raw_values = [r.get(field) for r in rows]
        values = [to_float(v) for v in raw_values]
        values = [v for v in values if v is not None]

        if not values:
            # e.g. every trial's time_to_map_coverage_s was 'not_reached'
            print(f"  {field:32s}: no numeric data ({raw_values[0]!r} in all {len(raw_values)} trials)")
            continue

        mean = statistics.mean(values)
        std = statistics.stdev(values) if len(values) > 1 else 0.0
        missing = len(raw_values) - len(values)
        missing_note = f", {missing} non-numeric" if missing else ""

        print(
            f"  {field:32s}: {mean:8.4f} +/- {std:7.4f}  "
            f"(n={len(values)}{missing_note}, min={min(values):.4f}, max={max(values):.4f})"
        )


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    for path in sys.argv[1:]:
        summarize(path)


if __name__ == '__main__':
    main()
