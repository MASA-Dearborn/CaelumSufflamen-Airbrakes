from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from pathlib import Path


THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

from policy_aero_empirical_fit import (  # noqa: E402
    DEFAULT_BODY_CDA_M2,
    DEFAULT_BRAKE_CDA_M2,
    DEFAULT_MASS_KG,
    DEFAULT_MIN_ALT_M,
    DEFAULT_MIN_VZ_MPS,
    DEFAULT_RHO_KGPM3,
    PHASE_BOOST,
    PHASE_BRAKE,
    PHASE_COAST,
    PHASE_DESCENT,
    predict_apogee_m,
    read_sd_log,
)


def is_finite(x: float) -> bool:
    return math.isfinite(x)


def rmse(values: list[float]) -> float | None:
    if not values:
        return None
    return math.sqrt(sum(v * v for v in values) / float(len(values)))


def mean(values: list[float]) -> float | None:
    if not values:
        return None
    return statistics.fmean(values)


def median(values: list[float]) -> float | None:
    if not values:
        return None
    return statistics.median(values)


def validate_log(
    path: Path,
    *,
    mass_kg: float,
    rho_kgpm3: float,
    body_cda_m2: float,
    brake_cda_m2: float,
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
    errors_m: list[float] = []
    skipped_count = 0

    for row in rows:
        if row.phase not in (PHASE_COAST, PHASE_BRAKE):
            skipped_count += 1
            continue
        if not is_finite(row.est_h_m) or not is_finite(row.est_v_mps):
            skipped_count += 1
            continue
        if row.est_h_m < min_alt_m or row.est_v_mps < min_vz_mps or row.est_h_m >= actual_apogee_m:
            skipped_count += 1
            continue

        predicted_apogee_m = predict_apogee_m(
            row.est_h_m,
            row.est_v_mps,
            row.policy_cmd,
            body_cda_m2=body_cda_m2,
            brake_cda_m2=brake_cda_m2,
            mass_kg=mass_kg,
            rho_kgpm3=rho_kgpm3,
        )
        if not is_finite(predicted_apogee_m):
            skipped_count += 1
            continue

        errors_m.append(predicted_apogee_m - actual_apogee_m)

    if not errors_m:
        raise ValueError(f"{path} did not yield any eligible replay samples")

    return {
        "log_path": str(path),
        "actual_apogee_m": actual_apogee_m,
        "eligible_sample_count": len(errors_m),
        "skipped_sample_count": skipped_count,
        "prediction_bias_m": mean(errors_m),
        "prediction_median_error_m": median(errors_m),
        "prediction_rmse_m": rmse(errors_m),
        "prediction_max_abs_error_m": max(abs(error_m) for error_m in errors_m),
    }


def validate_logs(
    log_paths: list[Path],
    *,
    mass_kg: float,
    rho_kgpm3: float,
    body_cda_m2: float,
    brake_cda_m2: float,
    min_alt_m: float,
    min_vz_mps: float,
    max_rmse_m: float,
    max_abs_error_m: float,
) -> dict:
    per_log = [
        validate_log(
            path,
            mass_kg=mass_kg,
            rho_kgpm3=rho_kgpm3,
            body_cda_m2=body_cda_m2,
            brake_cda_m2=brake_cda_m2,
            min_alt_m=min_alt_m,
            min_vz_mps=min_vz_mps,
        )
        for path in log_paths
    ]

    all_biases = [summary["prediction_bias_m"] for summary in per_log]
    all_rmses = [summary["prediction_rmse_m"] for summary in per_log]
    all_max_abs = [summary["prediction_max_abs_error_m"] for summary in per_log]

    aggregate_rmse_m = median(all_rmses)
    aggregate_max_abs_error_m = max(all_max_abs)
    passed = (
        aggregate_rmse_m is not None
        and aggregate_rmse_m <= max_rmse_m
        and aggregate_max_abs_error_m <= max_abs_error_m
    )

    return {
        "inputs": {
            "mass_kg": mass_kg,
            "rho_kgpm3": rho_kgpm3,
            "body_cda_m2": body_cda_m2,
            "brake_cda_m2": brake_cda_m2,
            "min_alt_m": min_alt_m,
            "min_vz_mps": min_vz_mps,
            "max_rmse_m": max_rmse_m,
            "max_abs_error_m": max_abs_error_m,
        },
        "per_log": per_log,
        "aggregate": {
            "log_count": len(per_log),
            "total_eligible_sample_count": sum(summary["eligible_sample_count"] for summary in per_log),
            "median_prediction_bias_m": median(all_biases),
            "median_prediction_rmse_m": aggregate_rmse_m,
            "max_prediction_abs_error_m": aggregate_max_abs_error_m,
            "passed": passed,
        },
    }


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Replay SD logs through the configured policy apogee model.")
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument("--mass-kg", type=float, default=DEFAULT_MASS_KG)
    parser.add_argument("--rho-kgpm3", type=float, default=DEFAULT_RHO_KGPM3)
    parser.add_argument("--body-cda-m2", type=float, default=DEFAULT_BODY_CDA_M2)
    parser.add_argument("--brake-cda-m2", type=float, default=DEFAULT_BRAKE_CDA_M2)
    parser.add_argument("--min-alt-m", type=float, default=DEFAULT_MIN_ALT_M)
    parser.add_argument("--min-vz-mps", type=float, default=DEFAULT_MIN_VZ_MPS)
    parser.add_argument("--max-rmse-m", type=float, default=0.05)
    parser.add_argument("--max-abs-error-m", type=float, default=0.10)
    parser.add_argument("--json-out", type=Path)
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    result = validate_logs(
        args.logs,
        mass_kg=args.mass_kg,
        rho_kgpm3=args.rho_kgpm3,
        body_cda_m2=args.body_cda_m2,
        brake_cda_m2=args.brake_cda_m2,
        min_alt_m=args.min_alt_m,
        min_vz_mps=args.min_vz_mps,
        max_rmse_m=args.max_rmse_m,
        max_abs_error_m=args.max_abs_error_m,
    )

    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(result, indent=2), encoding="utf-8")

    aggregate = result["aggregate"]
    print(f"Analyzed {aggregate['log_count']} log(s)")
    print(f"Eligible replay samples: {aggregate['total_eligible_sample_count']}")
    print(f"median_prediction_bias_m={aggregate['median_prediction_bias_m']}")
    print(f"median_prediction_rmse_m={aggregate['median_prediction_rmse_m']}")
    print(f"max_prediction_abs_error_m={aggregate['max_prediction_abs_error_m']}")
    print(f"passed={aggregate['passed']}")
    return 0 if aggregate["passed"] else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
