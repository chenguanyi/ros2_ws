"""Analyze plant protection flight logs and write tuning suggestions."""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import os
import statistics
import sys
from datetime import datetime
from pathlib import Path
from typing import Iterable


CURRENT_PARAMS = {
    "position_tolerance_cm": 5.0,
    "height_tolerance_cm": 4.0,
    "yaw_tolerance_deg": 3.0,
    "kp_xy": 0.55,
    "kd_xy": 0.035,
    "max_linear_velocity": 36.0,
}

MAX_VERTICAL_VELOCITY_CM_S = 36.0
MAX_YAW_VELOCITY_DEG_S = 30.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze plant protection control/event CSV logs.",
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=None,
        help="Directory containing plant_control_*.csv and plant_events_*.csv.",
    )
    parser.add_argument("--control-csv", type=Path, default=None)
    parser.add_argument("--event-csv", type=Path, default=None)
    return parser.parse_args()


def workspace_log_candidates() -> list[Path]:
    candidates: list[Path] = []
    ros_log_dir = os.environ.get("ROS_LOG_DIR")
    if ros_log_dir:
        candidates.append(Path(ros_log_dir))

    cwd = Path.cwd()
    for root in (cwd, *cwd.parents):
        candidates.append(root / "mylog" / "plant_protection")
    return candidates


def latest_csv(directory: Path, pattern: str) -> Path | None:
    if not directory.exists():
        return None
    files = sorted(directory.glob(pattern), key=lambda path: path.stat().st_mtime)
    return files[-1] if files else None


def resolve_inputs(args: argparse.Namespace) -> tuple[Path, Path, Path]:
    if args.control_csv and args.event_csv:
        log_dir = args.log_dir or args.control_csv.parent
        return log_dir, args.control_csv, args.event_csv

    candidates = [args.log_dir] if args.log_dir else []
    candidates.extend(workspace_log_candidates())
    seen: set[Path] = set()
    for candidate in candidates:
        if candidate is None:
            continue
        candidate = candidate.resolve()
        if candidate in seen:
            continue
        seen.add(candidate)
        control_csv = args.control_csv or latest_csv(candidate, "plant_control_*.csv")
        event_csv = args.event_csv or latest_csv(candidate, "plant_events_*.csv")
        if control_csv and event_csv:
            return candidate, control_csv, event_csv

    search_hint = args.log_dir or Path.cwd() / "mylog" / "plant_protection"
    raise FileNotFoundError(
        "Could not find plant_control_*.csv and plant_events_*.csv under "
        f"{search_hint}"
    )


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def to_float(row: dict[str, str], key: str) -> float:
    value = row.get(key, "")
    if value is None or value == "":
        return math.nan
    try:
        return float(value)
    except ValueError:
        return math.nan


def to_int(row: dict[str, str], key: str) -> int:
    value = row.get(key, "")
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return 0


def finite(values: Iterable[float]) -> list[float]:
    return [value for value in values if math.isfinite(value)]


def percentile(values: list[float], pct: float) -> float:
    clean = sorted(finite(values))
    if not clean:
        return math.nan
    if len(clean) == 1:
        return clean[0]
    rank = (len(clean) - 1) * pct / 100.0
    lower = math.floor(rank)
    upper = math.ceil(rank)
    if lower == upper:
        return clean[int(rank)]
    fraction = rank - lower
    return clean[lower] * (1.0 - fraction) + clean[upper] * fraction


def nearest_control_index(times: list[float], stamp: float) -> int | None:
    if not times or not math.isfinite(stamp):
        return None
    pos = bisect.bisect_left(times, stamp)
    candidates = []
    if pos < len(times):
        candidates.append(pos)
    if pos > 0:
        candidates.append(pos - 1)
    if not candidates:
        return None
    return min(candidates, key=lambda idx: abs(times[idx] - stamp))


def sign_changes(values: Iterable[float], deadzone: float = 1.0) -> int:
    last_sign = 0
    changes = 0
    for value in values:
        if not math.isfinite(value) or abs(value) < deadzone:
            continue
        sign = 1 if value > 0.0 else -1
        if last_sign and sign != last_sign:
            changes += 1
        last_sign = sign
    return changes


def control_segments(control_rows: list[dict[str, str]]) -> list[list[dict[str, str]]]:
    segments: list[list[dict[str, str]]] = []
    current_key: tuple[float, float, float, float] | None = None
    current_segment: list[dict[str, str]] = []
    for row in control_rows:
        key = (
            round(to_float(row, "target_x_cm"), 3),
            round(to_float(row, "target_y_cm"), 3),
            round(to_float(row, "target_z_cm"), 3),
            round(to_float(row, "target_yaw_deg"), 3),
        )
        if current_key is not None and key != current_key and current_segment:
            segments.append(current_segment)
            current_segment = []
        current_key = key
        current_segment.append(row)
    if current_segment:
        segments.append(current_segment)
    return segments


def event_time(row: dict[str, str]) -> float:
    return to_float(row, "time_sec")


def pair_arrival_times(events: list[dict[str, str]]) -> list[dict[str, float | int | str]]:
    starts: dict[int, tuple[float, str]] = {}
    arrivals: list[dict[str, float | int | str]] = []
    for row in events:
        name = row.get("event", "")
        cell_id = to_int(row, "cell_id")
        stamp = event_time(row)
        if cell_id <= 0 or not math.isfinite(stamp):
            continue
        if name in {"go_to_cell", "go_to_start_cell"}:
            starts[cell_id] = (stamp, name)
        elif name in {"cell_reached", "reached_start_cell"} and cell_id in starts:
            start_time, start_event = starts.pop(cell_id)
            arrivals.append({
                "cell_id": cell_id,
                "from_event": start_event,
                "to_event": name,
                "duration_sec": max(0.0, stamp - start_time),
            })
    return arrivals


def analyze(control_rows: list[dict[str, str]], event_rows: list[dict[str, str]]) -> dict:
    control_times = [to_float(row, "time_sec") for row in control_rows]
    laser_events = [row for row in event_rows if row.get("event") == "laser_on"]

    laser_samples = []
    for event in laser_events:
        stamp = event_time(event)
        nearest_index = nearest_control_index(control_times, stamp)
        control = control_rows[nearest_index] if nearest_index is not None else {}

        error_xy = to_float(event, "error_xy_cm")
        error_z = to_float(event, "error_z_cm")
        error_yaw = abs(to_float(event, "error_yaw_deg"))
        if not math.isfinite(error_xy):
            error_xy = to_float(control, "error_xy_cm")
        if not math.isfinite(error_z):
            error_z = to_float(control, "error_z_cm")
        if not math.isfinite(error_yaw):
            error_yaw = abs(to_float(control, "error_yaw_deg"))

        laser_samples.append({
            "time_sec": stamp,
            "cell_id": to_int(event, "cell_id"),
            "error_xy_cm": error_xy,
            "error_z_cm": error_z,
            "error_yaw_deg": error_yaw,
            "nearest_control_dt_sec": (
                abs(control_times[nearest_index] - stamp)
                if nearest_index is not None else math.nan
            ),
        })

    xy_speeds = [
        math.hypot(to_float(row, "cmd_x_cm_s"), to_float(row, "cmd_y_cm_s"))
        for row in control_rows
    ]
    z_speeds = [abs(to_float(row, "cmd_z_cm_s")) for row in control_rows]
    yaw_speeds = [abs(to_float(row, "cmd_yaw_deg_s")) for row in control_rows]
    xy_sat_limit = CURRENT_PARAMS["max_linear_velocity"] * 0.95
    z_sat_limit = MAX_VERTICAL_VELOCITY_CM_S * 0.95
    yaw_sat_limit = MAX_YAW_VELOCITY_DEG_S * 0.95
    sample_count = max(1, len(control_rows))
    saturation = {
        "xy_fraction": sum(speed >= xy_sat_limit for speed in xy_speeds) / sample_count,
        "z_fraction": sum(speed >= z_sat_limit for speed in z_speeds) / sample_count,
        "yaw_fraction": sum(speed >= yaw_sat_limit for speed in yaw_speeds) / sample_count,
    }

    segment_stats = []
    for segment in control_segments(control_rows):
        if len(segment) < 5:
            continue
        segment_stats.append({
            "duration_sec": max(0.0, event_time(segment[-1]) - event_time(segment[0])),
            "max_error_xy_cm": max(finite(to_float(row, "error_xy_cm") for row in segment) or [0.0]),
            "min_error_xy_cm": min(finite(to_float(row, "error_xy_cm") for row in segment) or [0.0]),
            "sign_changes_x": sign_changes(to_float(row, "error_x_cm") for row in segment),
            "sign_changes_y": sign_changes(to_float(row, "error_y_cm") for row in segment),
        })

    arrival_times = pair_arrival_times(event_rows)
    xy_errors = [sample["error_xy_cm"] for sample in laser_samples]
    z_errors = [abs(sample["error_z_cm"]) for sample in laser_samples]
    yaw_errors = [abs(sample["error_yaw_deg"]) for sample in laser_samples]
    oscillation_counts = [
        stat["sign_changes_x"] + stat["sign_changes_y"] for stat in segment_stats
    ]

    return {
        "laser_samples": laser_samples,
        "laser_error_summary": {
            "count": len(laser_samples),
            "mean_xy_cm": statistics.fmean(finite(xy_errors)) if finite(xy_errors) else math.nan,
            "p95_xy_cm": percentile(xy_errors, 95.0),
            "max_xy_cm": max(finite(xy_errors) or [math.nan]),
            "p95_abs_z_cm": percentile(z_errors, 95.0),
            "p95_abs_yaw_deg": percentile(yaw_errors, 95.0),
        },
        "arrival_times": arrival_times,
        "arrival_summary": {
            "count": len(arrival_times),
            "mean_sec": (
                statistics.fmean(item["duration_sec"] for item in arrival_times)
                if arrival_times else math.nan
            ),
            "max_sec": max((item["duration_sec"] for item in arrival_times), default=math.nan),
        },
        "saturation": saturation,
        "segment_stats": segment_stats,
        "oscillation_summary": {
            "mean_sign_changes": (
                statistics.fmean(oscillation_counts) if oscillation_counts else math.nan
            ),
            "max_sign_changes": max(oscillation_counts, default=0),
        },
    }


def build_recommendation(metrics: dict) -> dict:
    summary = metrics["laser_error_summary"]
    saturation = metrics["saturation"]
    oscillation = metrics["oscillation_summary"]
    recommendation = dict(CURRENT_PARAMS)
    actions: list[str] = []

    p95_xy = summary["p95_xy_cm"]
    p95_z = summary["p95_abs_z_cm"]
    p95_yaw = summary["p95_abs_yaw_deg"]
    xy_sat = saturation["xy_fraction"]
    osc_max = oscillation["max_sign_changes"]

    if math.isfinite(p95_xy) and p95_xy <= 4.0:
        actions.append("Keep position_tolerance_cm at 5.0; laser timing is inside the tightened gate.")
    elif math.isfinite(p95_xy) and p95_xy <= 6.0:
        actions.append("Keep position_tolerance_cm at 5.0 for one more flight; errors are near the gate.")
    elif math.isfinite(p95_xy):
        recommendation["position_tolerance_cm"] = 4.5
        actions.append("Consider position_tolerance_cm=4.5 because laser-on XY error p95 is high.")

    if math.isfinite(p95_z) and p95_z > 4.0:
        recommendation["height_tolerance_cm"] = 3.5
        actions.append("Consider height_tolerance_cm=3.5; laser-on Z error p95 exceeded 4 cm.")
    else:
        actions.append("Keep height_tolerance_cm at 4.0 unless landing data shows a separate bias.")

    if math.isfinite(p95_yaw) and p95_yaw > 3.0:
        recommendation["yaw_tolerance_deg"] = 2.5
        actions.append("Consider yaw_tolerance_deg=2.5; yaw p95 exceeded the current gate.")
    else:
        actions.append("Keep yaw_tolerance_deg at 3.0.")

    if math.isfinite(p95_xy) and p95_xy > 6.0 and xy_sat > 0.25:
        recommendation["max_linear_velocity"] = 32.0
        actions.append("XY velocity saturates often while laser error is high; try max_linear_velocity=32.0.")
    elif math.isfinite(p95_xy) and p95_xy > 6.0 and osc_max >= 8:
        recommendation["kd_xy"] = round(CURRENT_PARAMS["kd_xy"] + 0.01, 3)
        recommendation["kp_xy"] = round(CURRENT_PARAMS["kp_xy"] * 0.9, 3)
        actions.append("Oscillation is high; try slightly more damping and slightly less kp_xy.")
    else:
        actions.append("Keep kp_xy/kd_xy/max_linear_velocity unchanged for now.")

    return {
        "current_params": CURRENT_PARAMS,
        "recommended_params": recommendation,
        "actions": actions,
        "auto_apply": False,
    }


def format_float(value: float, digits: int = 2) -> str:
    return "n/a" if not math.isfinite(value) else f"{value:.{digits}f}"


def write_report(
    path: Path,
    control_csv: Path,
    event_csv: Path,
    metrics: dict,
    recommendation: dict,
) -> None:
    lines = [
        "# Plant tuning report",
        "",
        f"- Control CSV: `{control_csv}`",
        f"- Event CSV: `{event_csv}`",
        "",
        "## Laser-on center error",
        "",
        "| cell | xy cm | z cm | yaw deg | control dt ms |",
        "| ---: | ---: | ---: | ---: | ---: |",
    ]
    for sample in metrics["laser_samples"]:
        lines.append(
            "| {cell} | {xy} | {z} | {yaw} | {dt} |".format(
                cell=sample["cell_id"],
                xy=format_float(sample["error_xy_cm"]),
                z=format_float(sample["error_z_cm"]),
                yaw=format_float(sample["error_yaw_deg"]),
                dt=format_float(sample["nearest_control_dt_sec"] * 1000.0, 1),
            )
        )

    summary = metrics["laser_error_summary"]
    arrival = metrics["arrival_summary"]
    saturation = metrics["saturation"]
    oscillation = metrics["oscillation_summary"]
    lines.extend([
        "",
        "## Summary",
        "",
        f"- Laser-on samples: {summary['count']}",
        f"- XY error mean/p95/max: {format_float(summary['mean_xy_cm'])} / "
        f"{format_float(summary['p95_xy_cm'])} / {format_float(summary['max_xy_cm'])} cm",
        f"- Z error p95: {format_float(summary['p95_abs_z_cm'])} cm",
        f"- Yaw error p95: {format_float(summary['p95_abs_yaw_deg'])} deg",
        f"- Arrival count/mean/max: {arrival['count']} / "
        f"{format_float(arrival['mean_sec'])} / {format_float(arrival['max_sec'])} sec",
        f"- Velocity saturation XY/Z/yaw: {saturation['xy_fraction']:.1%} / "
        f"{saturation['z_fraction']:.1%} / {saturation['yaw_fraction']:.1%}",
        f"- Oscillation sign changes mean/max: "
        f"{format_float(oscillation['mean_sign_changes'])} / {oscillation['max_sign_changes']}",
        "",
        "## Suggested parameters",
        "",
        "```json",
        json.dumps(recommendation["recommended_params"], indent=2, sort_keys=True),
        "```",
        "",
        "## Actions",
        "",
    ])
    lines.extend(f"- {action}" for action in recommendation["actions"])
    path.write_text("\n".join(lines) + "\n")


def main() -> int:
    args = parse_args()
    try:
        log_dir, control_csv, event_csv = resolve_inputs(args)
    except FileNotFoundError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    control_rows = read_csv(control_csv)
    event_rows = read_csv(event_csv)
    metrics = analyze(control_rows, event_rows)
    recommendation = build_recommendation(metrics)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_path = log_dir / f"tuning_report_{timestamp}.md"
    recommendation_path = log_dir / f"tuning_recommendation_{timestamp}.json"

    write_report(report_path, control_csv, event_csv, metrics, recommendation)
    recommendation_path.write_text(
        json.dumps({
            "control_csv": str(control_csv),
            "event_csv": str(event_csv),
            "metrics": metrics,
            "recommendation": recommendation,
        }, indent=2, sort_keys=True) + "\n"
    )

    print(f"Wrote report: {report_path}")
    print(f"Wrote recommendation: {recommendation_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
