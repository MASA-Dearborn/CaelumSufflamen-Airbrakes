from __future__ import annotations

import argparse
import csv
import json
import math
import re
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read_first_existing(*relative_paths: str) -> str:
    for relative_path in relative_paths:
        candidate = ROOT / relative_path
        if candidate.exists():
            return candidate.read_text(encoding="utf-8")
    joined = ", ".join(relative_paths)
    raise FileNotFoundError(f"Could not locate any expected source file: {joined}")


def extract_constant(text: str, name: str) -> str:
    static_pattern = rf"static const [A-Za-z0-9_:\*]+ {name}\s*=\s*([^;\r\n]+)"
    match = re.search(static_pattern, text)
    if match:
        return match.group(1).strip()

    define_pattern = rf"#define {name}\s+([^\r\n]+)"
    match = re.search(define_pattern, text)
    if match:
        return match.group(1).strip()

    raise ValueError(f"Could not find constant {name}")


def parse_numeric_literal(expr: str) -> float:
    cleaned = expr.replace("UL", "").replace("U", "").replace("f", "").replace("F", "")
    return float(cleaned)


CONFIG_H = read_first_existing("config.h", "utils/config.h")

K_G = parse_numeric_literal(extract_constant(CONFIG_H, "kG"))
DEFAULT_MASS_KG = parse_numeric_literal(extract_constant(CONFIG_H, "POLICY_VEHICLE_MASS_KG"))
DEFAULT_RHO_KGPM3 = parse_numeric_literal(extract_constant(CONFIG_H, "POLICY_RHO_KGPM3"))
DEFAULT_BODY_CDA_M2 = parse_numeric_literal(extract_constant(CONFIG_H, "POLICY_CDA_BODY_M2"))
DEFAULT_BRAKE_CDA_M2 = parse_numeric_literal(extract_constant(CONFIG_H, "POLICY_CDA_BRAKE_M2"))
DEFAULT_MIN_ALT_M = parse_numeric_literal(extract_constant(CONFIG_H, "POLICY_MIN_ALT_M"))
DEFAULT_MIN_VZ_MPS = parse_numeric_literal(extract_constant(CONFIG_H, "POLICY_MIN_VZ_MPS"))

PHASE_IDLE = 0
PHASE_BOOST = 1
PHASE_COAST = 2
PHASE_BRAKE = 3
PHASE_DESCENT = 4


@dataclass
class LogSample:
    t_us: int
    phase: int
    est_h_m: float
    est_v_mps: float
    policy_cmd: float
    policy_valid: bool


def parse_float(cell: str) -> float:
    try:
        return float(cell)
    except (TypeError, ValueError):
        return math.nan


def parse_int(cell: str) -> int | None:
    try:
        return int(float(cell))
    except (TypeError, ValueError):
        return None


def clamp01(x: float) -> float:
    if not math.isfinite(x):
        return 0.0
    return min(1.0, max(0.0, x))


def is_finite(x: float) -> bool:
    return math.isfinite(x)


def read_sd_log(path: Path) -> list[LogSample]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        lines = [line for line in handle if line.strip() and not line.startswith("#")]

    reader = csv.DictReader(lines)
    required = {"t_us", "phase", "est_h", "est_v", "policy_cmd", "policy_valid"}

    if reader.fieldnames is None:
        raise ValueError(f"{path} does not contain a readable CSV header")

    missing = sorted(required.difference(reader.fieldnames))
    if missing:
        joined = ", ".join(missing)
        raise ValueError(f"{path} is missing required columns: {joined}")

    rows: list[LogSample] = []
    for row in reader:
        phase = parse_int(row["phase"])
        t_us = parse_int(row["t_us"])
        if phase is None or t_us is None:
            continue

        policy_valid_int = parse_int(row["policy_valid"])
        rows.append(
            LogSample(
                t_us=t_us,
                phase=phase,
                est_h_m=parse_float(row["est_h"]),
                est_v_mps=parse_float(row["est_v"]),
                policy_cmd=parse_float(row["policy_cmd"]),
                policy_valid=(policy_valid_int == 1),
            )
        )

    if not rows:
        raise ValueError(f"{path} did not yield any usable data rows")

    return rows


def model_remaining_altitude_m(v_mps: float, k_inv_m: float) -> float:
    if not is_finite(v_mps) or v_mps <= 0.0:
        return 0.0

    v2 = v_mps * v_mps
    if not is_finite(k_inv_m) or k_inv_m < 1.0e-9:
        return v2 / (2.0 * K_G)

    argument = 1.0 + (k_inv_m * v2) / K_G
    if not is_finite(argument) or argument <= 0.0:
        return math.nan

    return math.log(argument) / (2.0 * k_inv_m)


def solve_drag_k_from_apogee_delta(delta_h_m: float, v_mps: float) -> float | None:
    if not is_finite(delta_h_m) or not is_finite(v_mps):
        return None
    if delta_h_m <= 0.0 or v_mps <= 0.0:
        return None

    ballistic_delta_m = model_remaining_altitude_m(v_mps, 0.0)
    if not is_finite(ballistic_delta_m) or delta_h_m > ballistic_delta_m * 1.000001:
        return None

    if abs(delta_h_m - ballistic_delta_m) <= 1.0e-6:
        return 0.0

    lo = 0.0
    hi = 1.0e-6

    while hi < 1.0e3:
        hi_delta_m = model_remaining_altitude_m(v_mps, hi)
        if not is_finite(hi_delta_m):
            return None
        if hi_delta_m <= delta_h_m:
            break
        hi *= 2.0
    else:
        return None

    for _ in range(80):
        mid = 0.5 * (lo + hi)
        mid_delta_m = model_remaining_altitude_m(v_mps, mid)
        if not is_finite(mid_delta_m):
            return None
        if mid_delta_m > delta_h_m:
            lo = mid
        else:
            hi = mid

    return 0.5 * (lo + hi)


def predict_apogee_m(
    h_m: float,
    v_mps: float,
    command01: float,
    *,
    body_cda_m2: float,
    brake_cda_m2: float,
    mass_kg: float,
    rho_kgpm3: float,
) -> float:
    if not is_finite(h_m) or not is_finite(v_mps):
        return math.nan
    if v_mps <= 0.0:
        return h_m
    if not is_finite(body_cda_m2) or not is_finite(brake_cda_m2):
        return math.nan
    if not is_finite(mass_kg) or mass_kg <= 0.0:
        return math.nan
    if not is_finite(rho_kgpm3) or rho_kgpm3 <= 0.0:
        return math.nan

    cda_m2 = body_cda_m2 + clamp01(command01) * brake_cda_m2
    if cda_m2 < 0.0:
        return math.nan

    k_inv_m = (rho_kgpm3 * cda_m2) / (2.0 * mass_kg)
    return h_m + model_remaining_altitude_m(v_mps, k_inv_m)


def rmse(values: list[float]) -> float | None:
    if not values:
        return None
    mean_sq = sum(v * v for v in values) / float(len(values))
    return math.sqrt(mean_sq)


def mean(values: list[float]) -> float | None:
    if not values:
        return None
    return statistics.fmean(values)


def median(values: list[float]) -> float | None:
    if not values:
        return None
    return statistics.median(values)


def analyze_log(
    path: Path,
    *,
    mass_kg: float,
    rho_kgpm3: float,
    current_body_cda_m2: float,
    current_brake_cda_m2: float,
    closed_cmd_threshold: float,
    open_cmd_threshold: float,
    min_alt_m: float,
    min_vz_mps: float,
) -> dict:
    rows = read_sd_log(path)

    apogee_candidates = [
        row.est_h_m
        for row in rows
        if row.phase in (PHASE_BOOST, PHASE_COAST, PHASE_BRAKE, PHASE_DESCENT) and is_finite(row.est_h_m)
    ]
    if not apogee_candidates:
        raise ValueError(f"{path} does not contain any usable in-flight estimator altitude samples")

    actual_apogee_m = max(apogee_candidates)

    fitted_body_samples_m2: list[float] = []
    fitted_brake_candidate_samples_m2: list[float] = []
    current_prediction_errors_m: list[float] = []
    raw_fit_samples: list[tuple[float, float, float]] = []

    skipped_phase = 0
    skipped_gate = 0
    skipped_model = 0

    for row in rows:
        if row.phase not in (PHASE_COAST, PHASE_BRAKE):
            skipped_phase += 1
            continue

        if not is_finite(row.est_h_m) or not is_finite(row.est_v_mps):
            skipped_gate += 1
            continue

        if row.est_h_m < min_alt_m or row.est_v_mps < min_vz_mps or row.est_h_m >= actual_apogee_m:
            skipped_gate += 1
            continue

        delta_h_m = actual_apogee_m - row.est_h_m
        k_inv_m = solve_drag_k_from_apogee_delta(delta_h_m, row.est_v_mps)
        if k_inv_m is None:
            skipped_model += 1
            continue

        equiv_cda_m2 = (2.0 * mass_kg * k_inv_m) / rho_kgpm3
        if not is_finite(equiv_cda_m2) or equiv_cda_m2 < 0.0:
            skipped_model += 1
            continue

        raw_fit_samples.append((row.est_h_m, row.est_v_mps, row.policy_cmd))

        if clamp01(row.policy_cmd) <= closed_cmd_threshold:
            fitted_body_samples_m2.append(equiv_cda_m2)

        if clamp01(row.policy_cmd) >= open_cmd_threshold:
            fitted_brake_candidate_samples_m2.append(equiv_cda_m2)

        predicted_apogee_current_m = predict_apogee_m(
            row.est_h_m,
            row.est_v_mps,
            row.policy_cmd,
            body_cda_m2=current_body_cda_m2,
            brake_cda_m2=current_brake_cda_m2,
            mass_kg=mass_kg,
            rho_kgpm3=rho_kgpm3,
        )
        if is_finite(predicted_apogee_current_m):
            current_prediction_errors_m.append(predicted_apogee_current_m - actual_apogee_m)

    recommended_body_cda_m2 = median(fitted_body_samples_m2)

    recommended_brake_cda_m2: float | None = None
    brake_increment_candidates_m2: list[float] = []
    if recommended_body_cda_m2 is not None:
        for row in rows:
            if row.phase not in (PHASE_COAST, PHASE_BRAKE):
                continue
            if not is_finite(row.est_h_m) or not is_finite(row.est_v_mps):
                continue
            if row.est_h_m < min_alt_m or row.est_v_mps < min_vz_mps or row.est_h_m >= actual_apogee_m:
                continue

            command01 = clamp01(row.policy_cmd)
            if command01 < open_cmd_threshold:
                continue

            delta_h_m = actual_apogee_m - row.est_h_m
            k_inv_m = solve_drag_k_from_apogee_delta(delta_h_m, row.est_v_mps)
            if k_inv_m is None:
                continue

            equiv_cda_m2 = (2.0 * mass_kg * k_inv_m) / rho_kgpm3
            candidate_brake_cda_m2 = (equiv_cda_m2 - recommended_body_cda_m2) / command01
            if is_finite(candidate_brake_cda_m2) and candidate_brake_cda_m2 >= 0.0:
                brake_increment_candidates_m2.append(candidate_brake_cda_m2)

        if brake_increment_candidates_m2:
            recommended_brake_cda_m2 = median(brake_increment_candidates_m2)

    fitted_prediction_errors_m: list[float] = []
    if recommended_body_cda_m2 is not None and recommended_brake_cda_m2 is not None:
        for h_m, v_mps, command01 in raw_fit_samples:
            predicted_apogee_fitted_m = predict_apogee_m(
                h_m,
                v_mps,
                command01,
                body_cda_m2=recommended_body_cda_m2,
                brake_cda_m2=recommended_brake_cda_m2,
                mass_kg=mass_kg,
                rho_kgpm3=rho_kgpm3,
            )
            if is_finite(predicted_apogee_fitted_m):
                fitted_prediction_errors_m.append(predicted_apogee_fitted_m - actual_apogee_m)

    return {
        "log_path": str(path),
        "actual_apogee_m": actual_apogee_m,
        "mass_kg": mass_kg,
        "rho_kgpm3": rho_kgpm3,
        "eligible_sample_count": len(raw_fit_samples),
        "body_fit_sample_count": len(fitted_body_samples_m2),
        "brake_fit_sample_count": len(brake_increment_candidates_m2),
        "skipped_phase_count": skipped_phase,
        "skipped_gate_count": skipped_gate,
        "skipped_model_count": skipped_model,
        "current_body_cda_m2": current_body_cda_m2,
        "current_brake_cda_m2": current_brake_cda_m2,
        "recommended_body_cda_m2": recommended_body_cda_m2,
        "recommended_brake_cda_m2": recommended_brake_cda_m2,
        "current_prediction_bias_m": mean(current_prediction_errors_m),
        "current_prediction_rmse_m": rmse(current_prediction_errors_m),
        "fitted_prediction_bias_m": mean(fitted_prediction_errors_m),
        "fitted_prediction_rmse_m": rmse(fitted_prediction_errors_m),
    }


def aggregate_summaries(summaries: list[dict]) -> dict:
    body_estimates_m2 = [
        summary["recommended_body_cda_m2"]
        for summary in summaries
        if summary["recommended_body_cda_m2"] is not None
    ]
    brake_estimates_m2 = [
        summary["recommended_brake_cda_m2"]
        for summary in summaries
        if summary["recommended_brake_cda_m2"] is not None
    ]
    current_biases_m = [
        summary["current_prediction_bias_m"]
        for summary in summaries
        if summary["current_prediction_bias_m"] is not None
    ]
    current_rmses_m = [
        summary["current_prediction_rmse_m"]
        for summary in summaries
        if summary["current_prediction_rmse_m"] is not None
    ]
    fitted_biases_m = [
        summary["fitted_prediction_bias_m"]
        for summary in summaries
        if summary["fitted_prediction_bias_m"] is not None
    ]
    fitted_rmses_m = [
        summary["fitted_prediction_rmse_m"]
        for summary in summaries
        if summary["fitted_prediction_rmse_m"] is not None
    ]

    return {
        "log_count": len(summaries),
        "recommended_body_cda_m2": median(body_estimates_m2),
        "recommended_brake_cda_m2": median(brake_estimates_m2),
        "median_current_prediction_bias_m": median(current_biases_m),
        "median_current_prediction_rmse_m": median(current_rmses_m),
        "median_fitted_prediction_bias_m": median(fitted_biases_m),
        "median_fitted_prediction_rmse_m": median(fitted_rmses_m),
        "total_eligible_sample_count": sum(summary["eligible_sample_count"] for summary in summaries),
    }


def analyze_logs(
    log_paths: list[Path],
    *,
    mass_kg: float,
    rho_kgpm3: float,
    current_body_cda_m2: float,
    current_brake_cda_m2: float,
    closed_cmd_threshold: float,
    open_cmd_threshold: float,
    min_alt_m: float,
    min_vz_mps: float,
) -> dict:
    per_log = [
        analyze_log(
            path,
            mass_kg=mass_kg,
            rho_kgpm3=rho_kgpm3,
            current_body_cda_m2=current_body_cda_m2,
            current_brake_cda_m2=current_brake_cda_m2,
            closed_cmd_threshold=closed_cmd_threshold,
            open_cmd_threshold=open_cmd_threshold,
            min_alt_m=min_alt_m,
            min_vz_mps=min_vz_mps,
        )
        for path in log_paths
    ]

    return {
        "per_log": per_log,
        "aggregate": aggregate_summaries(per_log),
    }


def print_summary(result: dict) -> None:
    aggregate = result["aggregate"]
    print(f"Analyzed {aggregate['log_count']} log(s)")
    print(f"Eligible fit samples: {aggregate['total_eligible_sample_count']}")

    for summary in result["per_log"]:
        print()
        print(f"Log: {summary['log_path']}")
        print(f"  actual_apogee_m: {summary['actual_apogee_m']:.3f}")
        print(f"  eligible_samples: {summary['eligible_sample_count']}")
        print(f"  recommended_body_cda_m2: {summary['recommended_body_cda_m2']}")
        print(f"  recommended_brake_cda_m2: {summary['recommended_brake_cda_m2']}")
        print(f"  current_prediction_bias_m: {summary['current_prediction_bias_m']}")
        print(f"  current_prediction_rmse_m: {summary['current_prediction_rmse_m']}")
        print(f"  fitted_prediction_bias_m: {summary['fitted_prediction_bias_m']}")
        print(f"  fitted_prediction_rmse_m: {summary['fitted_prediction_rmse_m']}")

    print()
    print("Aggregate recommendation")
    print(f"  POLICY_CDA_BODY_M2  = {aggregate['recommended_body_cda_m2']}")
    print(f"  POLICY_CDA_BRAKE_M2 = {aggregate['recommended_brake_cda_m2']}")
    print(f"  median_current_prediction_bias_m  = {aggregate['median_current_prediction_bias_m']}")
    print(f"  median_current_prediction_rmse_m  = {aggregate['median_current_prediction_rmse_m']}")
    print(f"  median_fitted_prediction_bias_m   = {aggregate['median_fitted_prediction_bias_m']}")
    print(f"  median_fitted_prediction_rmse_m   = {aggregate['median_fitted_prediction_rmse_m']}")

    if aggregate["recommended_body_cda_m2"] is not None and aggregate["recommended_brake_cda_m2"] is not None:
        print()
        print("Suggested config snippet")
        print(f"  static const float POLICY_CDA_BODY_M2 = {aggregate['recommended_body_cda_m2']:.6f}f;")
        print(f"  static const float POLICY_CDA_BRAKE_M2 = {aggregate['recommended_brake_cda_m2']:.6f}f;")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Estimate airbrake-policy aerodynamic constants from committed SD logs."
    )
    parser.add_argument("logs", nargs="+", type=Path, help="One or more LOG###.CSV files from the SD logger")
    parser.add_argument("--mass-kg", type=float, default=DEFAULT_MASS_KG)
    parser.add_argument("--rho-kgpm3", type=float, default=DEFAULT_RHO_KGPM3)
    parser.add_argument("--current-body-cda-m2", type=float, default=DEFAULT_BODY_CDA_M2)
    parser.add_argument("--current-brake-cda-m2", type=float, default=DEFAULT_BRAKE_CDA_M2)
    parser.add_argument("--closed-cmd-threshold", type=float, default=0.05)
    parser.add_argument("--open-cmd-threshold", type=float, default=0.20)
    parser.add_argument("--min-alt-m", type=float, default=DEFAULT_MIN_ALT_M)
    parser.add_argument("--min-vz-mps", type=float, default=DEFAULT_MIN_VZ_MPS)
    parser.add_argument("--json-out", type=Path, default=None, help="Optional machine-readable summary output path")
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)

    result = analyze_logs(
        args.logs,
        mass_kg=args.mass_kg,
        rho_kgpm3=args.rho_kgpm3,
        current_body_cda_m2=args.current_body_cda_m2,
        current_brake_cda_m2=args.current_brake_cda_m2,
        closed_cmd_threshold=args.closed_cmd_threshold,
        open_cmd_threshold=args.open_cmd_threshold,
        min_alt_m=args.min_alt_m,
        min_vz_mps=args.min_vz_mps,
    )

    print_summary(result)

    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(result, indent=2, sort_keys=True), encoding="utf-8")

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
