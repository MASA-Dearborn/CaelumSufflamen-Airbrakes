from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DATA_DIR = ROOT / "flight data"

CURRENT_SD_REQUIRED_COLUMNS = {"t_us", "phase", "est_h", "est_v", "policy_cmd", "policy_valid"}
LEGACY_MC_REQUIRED_COLUMNS = {"t_us", "kf_h", "kf_v"}
RAW_SENSOR_COLUMNS = {"t_us", "bmp_T", "bmp_P", "bmp_alt", "ax", "ay", "az", "gx", "gy", "gz"}

K_G = 9.80665
POLICY_MIN_ALT_M = 30.0
POLICY_MIN_VZ_MPS = 15.0


def parse_float(cell: str | None) -> float:
    try:
        return float(cell) if cell is not None else math.nan
    except ValueError:
        return math.nan


def finite_values(rows: list[dict], field: str) -> list[float]:
    values = [parse_float(row.get(field)) for row in rows]
    return [value for value in values if math.isfinite(value)]


def read_csv(path: Path) -> tuple[list[str], list[dict]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)
        return list(reader.fieldnames or []), rows


def median(values: list[float]) -> float | None:
    if not values:
        return None
    return statistics.median(values)


def audit_mc_log(path: Path, fieldnames: list[str], rows: list[dict]) -> dict:
    h_values = finite_values(rows, "kf_h")
    t_values = finite_values(rows, "t_us")

    max_h_m = max(h_values) if h_values else None
    max_index = None
    v_at_max_h_mps = None
    if h_values:
        best_index = None
        best_h = -math.inf
        for index, row in enumerate(rows):
            h_m = parse_float(row.get("kf_h"))
            if math.isfinite(h_m) and h_m > best_h:
                best_h = h_m
                best_index = index
        if best_index is not None:
            max_index = best_index
            v_at_max_h_mps = parse_float(rows[best_index].get("kf_v"))

    rows_meeting_policy_gate = 0
    for row in rows:
        h_m = parse_float(row.get("kf_h"))
        v_mps = parse_float(row.get("kf_v"))
        if h_m >= POLICY_MIN_ALT_M and v_mps >= POLICY_MIN_VZ_MPS:
            rows_meeting_policy_gate += 1

    positive_drag_sample_count = 0
    drag_k_candidates: list[float] = []
    previous: tuple[float, float, float] | None = None
    for row in rows:
        t_s = parse_float(row.get("t_us")) * 1.0e-6
        h_m = parse_float(row.get("kf_h"))
        v_mps = parse_float(row.get("kf_v"))
        if not (math.isfinite(t_s) and math.isfinite(h_m) and math.isfinite(v_mps)):
            continue
        if previous is not None:
            prev_t_s, _, prev_v_mps = previous
            dt_s = t_s - prev_t_s
            if dt_s > 0.0 and v_mps > 5.0:
                dvdt_mps2 = (v_mps - prev_v_mps) / dt_s
                k_inv_m = -(dvdt_mps2 + K_G) / (v_mps * abs(v_mps))
                if math.isfinite(k_inv_m) and 0.0 < k_inv_m < 0.1:
                    positive_drag_sample_count += 1
                    drag_k_candidates.append(k_inv_m)
        previous = (t_s, h_m, v_mps)

    observed_apogee_available = (
        max_index is not None
        and max_index < len(rows) - 1
        and v_at_max_h_mps is not None
        and math.isfinite(v_at_max_h_mps)
        and abs(v_at_max_h_mps) <= 2.0
    )

    return {
        "path": str(path.relative_to(ROOT)),
        "schema": "legacy_mc",
        "row_count": len(rows),
        "duration_s": ((max(t_values) - min(t_values)) * 1.0e-6) if len(t_values) >= 2 else None,
        "has_current_sd_policy_columns": CURRENT_SD_REQUIRED_COLUMNS.issubset(set(fieldnames)),
        "max_kf_h_m": max_h_m,
        "max_kf_h_row_index": max_index,
        "max_kf_h_is_last_row": max_index == len(rows) - 1 if max_index is not None else None,
        "kf_v_at_max_h_mps": v_at_max_h_mps,
        "observed_apogee_available": observed_apogee_available,
        "rows_meeting_policy_gate": rows_meeting_policy_gate,
        "positive_drag_sample_count": positive_drag_sample_count,
        "median_positive_drag_k_inv_m": median(drag_k_candidates),
        "can_identify_body_cda": observed_apogee_available and positive_drag_sample_count > 0,
        "can_identify_brake_cda": False,
        "brake_cda_blocker": "No policy_cmd or actuator command column is present in this legacy MC schema.",
    }


def audit_raw_log(path: Path, fieldnames: list[str], rows: list[dict]) -> dict:
    bmp_alt = finite_values(rows, "bmp_alt")
    t_values = finite_values(rows, "t_us")
    return {
        "path": str(path.relative_to(ROOT)),
        "schema": "raw_sensor",
        "row_count": len(rows),
        "duration_s": ((max(t_values) - min(t_values)) * 1.0e-6) if len(t_values) >= 2 else None,
        "has_current_sd_policy_columns": CURRENT_SD_REQUIRED_COLUMNS.issubset(set(fieldnames)),
        "bmp_alt_min_m": min(bmp_alt) if bmp_alt else None,
        "bmp_alt_max_m": max(bmp_alt) if bmp_alt else None,
        "can_identify_body_cda": False,
        "can_identify_brake_cda": False,
        "blocker": "Raw sensor logs do not contain phase, estimator velocity, policy command, or observed apogee fields.",
    }


def audit_file(path: Path) -> dict:
    fieldnames, rows = read_csv(path)
    field_set = set(fieldnames)
    if LEGACY_MC_REQUIRED_COLUMNS.issubset(field_set):
        return audit_mc_log(path, fieldnames, rows)
    if RAW_SENSOR_COLUMNS.issubset(field_set):
        return audit_raw_log(path, fieldnames, rows)
    return {
        "path": str(path.relative_to(ROOT)),
        "schema": "unknown",
        "row_count": len(rows),
        "has_current_sd_policy_columns": CURRENT_SD_REQUIRED_COLUMNS.issubset(field_set),
        "can_identify_body_cda": False,
        "can_identify_brake_cda": False,
        "blocker": "CSV schema is not recognized by the current audit tool.",
    }


def audit_directory(data_dir: Path) -> dict:
    data_dir = data_dir.resolve()
    csv_paths = sorted(path for path in data_dir.rglob("*") if path.is_file() and path.suffix.lower() == ".csv")
    files = [audit_file(path) for path in csv_paths]
    mc_logs = [entry for entry in files if entry["schema"] == "legacy_mc"]
    raw_logs = [entry for entry in files if entry["schema"] == "raw_sensor"]
    body_identifiable = [entry for entry in files if entry.get("can_identify_body_cda")]
    brake_identifiable = [entry for entry in files if entry.get("can_identify_brake_cda")]

    return {
        "data_dir": str(data_dir.relative_to(ROOT)),
        "file_count": len(files),
        "legacy_mc_log_count": len(mc_logs),
        "raw_sensor_log_count": len(raw_logs),
        "body_identifiable_log_count": len(body_identifiable),
        "brake_identifiable_log_count": len(brake_identifiable),
        "can_update_policy_cda_body_m2": len(body_identifiable) > 0,
        "can_update_policy_cda_brake_m2": len(brake_identifiable) > 0,
        "conclusion": (
            "The provided previous-year logs are useful for replay/schema review, but they do not contain enough "
            "information to identify the current airbrake policy aerodynamic coefficients."
        ),
        "required_for_future_fit": [
            "observed apogee or complete coast-through-descent altitude history",
            "estimator altitude and vertical velocity during the coast/brake interval",
            "airbrake command or actuator deployment state",
            "vehicle mass and atmospheric-density assumption for the fitted interval",
        ],
        "files": files,
    }


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Audit previous-year flight CSVs for aerodynamic-fit suitability.")
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA_DIR)
    parser.add_argument("--json-out", type=Path)
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    result = audit_directory(args.data_dir)
    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(result, indent=2), encoding="utf-8")

    print(f"Audited {result['file_count']} CSV file(s) in {result['data_dir']}")
    print(f"legacy_mc_log_count={result['legacy_mc_log_count']}")
    print(f"raw_sensor_log_count={result['raw_sensor_log_count']}")
    print(f"body_identifiable_log_count={result['body_identifiable_log_count']}")
    print(f"brake_identifiable_log_count={result['brake_identifiable_log_count']}")
    print(f"can_update_policy_cda_body_m2={result['can_update_policy_cda_body_m2']}")
    print(f"can_update_policy_cda_brake_m2={result['can_update_policy_cda_brake_m2']}")
    print(result["conclusion"])
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
