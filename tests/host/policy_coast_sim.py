from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
CONFIG_H = ROOT / "utils" / "config.h"


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


CONFIG_TEXT = CONFIG_H.read_text(encoding="utf-8")
K_G = parse_numeric_literal(extract_constant(CONFIG_TEXT, "kG"))
POLICY_TARGET_APOGEE_M = parse_numeric_literal(extract_constant(CONFIG_TEXT, "POLICY_TARGET_APOGEE_M"))
POLICY_MIN_ALT_M = parse_numeric_literal(extract_constant(CONFIG_TEXT, "POLICY_MIN_ALT_M"))
POLICY_MIN_VZ_MPS = parse_numeric_literal(extract_constant(CONFIG_TEXT, "POLICY_MIN_VZ_MPS"))
POLICY_APOGEE_DEADBAND_M = parse_numeric_literal(extract_constant(CONFIG_TEXT, "POLICY_APOGEE_DEADBAND_M"))
POLICY_VEHICLE_MASS_KG = parse_numeric_literal(extract_constant(CONFIG_TEXT, "POLICY_VEHICLE_MASS_KG"))
POLICY_RHO_KGPM3 = parse_numeric_literal(extract_constant(CONFIG_TEXT, "POLICY_RHO_KGPM3"))
POLICY_CDA_BODY_M2 = parse_numeric_literal(extract_constant(CONFIG_TEXT, "POLICY_CDA_BODY_M2"))
POLICY_CDA_BRAKE_M2 = parse_numeric_literal(extract_constant(CONFIG_TEXT, "POLICY_CDA_BRAKE_M2"))
POLICY_MAX_COMMAND01 = parse_numeric_literal(extract_constant(CONFIG_TEXT, "POLICY_MAX_COMMAND01"))
POLICY_SLEW_PER_SEC = parse_numeric_literal(extract_constant(CONFIG_TEXT, "POLICY_SLEW_PER_SEC"))
POLICY_BISECTION_STEPS = int(round(parse_numeric_literal(extract_constant(CONFIG_TEXT, "POLICY_BISECTION_STEPS"))))


def clamp01(x: float) -> float:
    if not math.isfinite(x):
        return 0.0
    return min(1.0, max(0.0, x))


def policy_drag_k(command01: float) -> float:
    u = clamp01(command01)
    cda_m2 = POLICY_CDA_BODY_M2 + u * POLICY_CDA_BRAKE_M2
    if POLICY_RHO_KGPM3 <= 0.0 or POLICY_VEHICLE_MASS_KG <= 0.0 or cda_m2 < 0.0:
        return 0.0
    return (POLICY_RHO_KGPM3 * cda_m2) / (2.0 * POLICY_VEHICLE_MASS_KG)


def predict_apogee_m(h_m: float, v_mps: float, command01: float) -> float:
    if not math.isfinite(h_m) or not math.isfinite(v_mps):
        return math.nan
    if v_mps <= 0.0:
        return h_m
    k_inv_m = policy_drag_k(command01)
    v2 = v_mps * v_mps
    if not math.isfinite(k_inv_m) or k_inv_m < 1.0e-7:
        return h_m + v2 / (2.0 * K_G)
    argument = 1.0 + (k_inv_m * v2) / K_G
    if not math.isfinite(argument) or argument <= 0.0:
        return math.nan
    return h_m + math.log(argument) / (2.0 * k_inv_m)


def solve_policy_command01(h_m: float, v_mps: float, target_apogee_m: float) -> float:
    if not math.isfinite(h_m) or not math.isfinite(v_mps) or not math.isfinite(target_apogee_m):
        return 0.0
    if v_mps <= 0.0:
        return 0.0
    if h_m < POLICY_MIN_ALT_M or v_mps < POLICY_MIN_VZ_MPS:
        return 0.0

    u_max = clamp01(POLICY_MAX_COMMAND01)
    apogee_u0 = predict_apogee_m(h_m, v_mps, 0.0)
    if not math.isfinite(apogee_u0):
        return 0.0
    if apogee_u0 <= target_apogee_m + POLICY_APOGEE_DEADBAND_M:
        return 0.0

    apogee_umax = predict_apogee_m(h_m, v_mps, u_max)
    if not math.isfinite(apogee_umax):
        return 0.0
    if apogee_umax > target_apogee_m:
        return u_max

    lo = 0.0
    hi = u_max
    for _ in range(POLICY_BISECTION_STEPS):
        mid = 0.5 * (lo + hi)
        apogee_mid = predict_apogee_m(h_m, v_mps, mid)
        if not math.isfinite(apogee_mid):
            return 0.0
        if apogee_mid > target_apogee_m:
            lo = mid
        else:
            hi = mid
    return clamp01(0.5 * (lo + hi))


def resolve_command01(
    *,
    mode: str,
    current_command01: float,
    h_m: float,
    v_mps: float,
    target_apogee_m: float,
    manual_command01: float,
    dt_s: float,
) -> float:
    if mode == "closed":
        desired = 0.0
    elif mode == "open":
        desired = clamp01(POLICY_MAX_COMMAND01)
    elif mode == "manual":
        desired = clamp01(manual_command01)
    elif mode == "policy":
        desired = solve_policy_command01(h_m, v_mps, target_apogee_m)
    else:
        raise ValueError(f"Unsupported mode: {mode}")

    max_step = POLICY_SLEW_PER_SEC * max(0.0, dt_s)
    limited = min(desired, current_command01 + max_step)
    limited = max(limited, current_command01 - max_step)
    return clamp01(limited)


def simulate_coast(
    *,
    h0_m: float,
    v0_mps: float,
    mode: str,
    target_apogee_m: float,
    manual_command01: float = 0.0,
    dt_s: float = 0.02,
    max_time_s: float = 30.0,
) -> dict:
    if dt_s <= 0.0:
        raise ValueError("dt_s must be positive")
    if max_time_s <= 0.0:
        raise ValueError("max_time_s must be positive")

    t_s = 0.0
    h_m = h0_m
    v_mps = v0_mps
    command01 = 0.0
    samples: list[dict] = []
    apogee_m = h_m
    max_command01 = 0.0

    while t_s <= max_time_s:
        command01 = resolve_command01(
            mode=mode,
            current_command01=command01,
            h_m=h_m,
            v_mps=v_mps,
            target_apogee_m=target_apogee_m,
            manual_command01=manual_command01,
            dt_s=dt_s,
        )

        drag_k_inv_m = policy_drag_k(command01)
        accel_mps2 = -K_G - drag_k_inv_m * v_mps * abs(v_mps)

        samples.append(
            {
                "t_s": t_s,
                "h_m": h_m,
                "v_mps": v_mps,
                "a_mps2": accel_mps2,
                "command01": command01,
                "predicted_apogee_m": predict_apogee_m(h_m, max(0.0, v_mps), command01),
            }
        )

        apogee_m = max(apogee_m, h_m)
        max_command01 = max(max_command01, command01)

        if t_s > 0.0 and v_mps <= 0.0:
            break

        next_h_m = h_m + v_mps * dt_s + 0.5 * accel_mps2 * dt_s * dt_s
        next_v_mps = v_mps + accel_mps2 * dt_s
        t_s += dt_s
        h_m = next_h_m
        v_mps = next_v_mps

    return {
        "mode": mode,
        "h0_m": h0_m,
        "v0_mps": v0_mps,
        "dt_s": dt_s,
        "max_time_s": max_time_s,
        "target_apogee_m": target_apogee_m,
        "manual_command01": clamp01(manual_command01),
        "apogee_m": apogee_m,
        "time_to_apogee_s": t_s,
        "max_command01": max_command01,
        "samples": samples,
    }


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a minimal 1D host-side coast simulation using the firmware's drag model."
    )
    parser.add_argument("--mode", choices=("closed", "open", "manual", "policy"), default="policy")
    parser.add_argument("--h0-m", type=float, required=True)
    parser.add_argument("--v0-mps", type=float, required=True)
    parser.add_argument("--target-apogee-m", type=float, default=POLICY_TARGET_APOGEE_M)
    parser.add_argument("--manual-command01", type=float, default=0.0)
    parser.add_argument("--dt-s", type=float, default=0.02)
    parser.add_argument("--max-time-s", type=float, default=30.0)
    parser.add_argument("--csv-out", type=Path)
    parser.add_argument("--json-out", type=Path)
    return parser.parse_args(argv)


def write_csv(path: Path, samples: list[dict]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=["t_s", "h_m", "v_mps", "a_mps2", "command01", "predicted_apogee_m"],
        )
        writer.writeheader()
        writer.writerows(samples)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    result = simulate_coast(
        h0_m=args.h0_m,
        v0_mps=args.v0_mps,
        mode=args.mode,
        target_apogee_m=args.target_apogee_m,
        manual_command01=args.manual_command01,
        dt_s=args.dt_s,
        max_time_s=args.max_time_s,
    )

    if args.csv_out is not None:
        args.csv_out.parent.mkdir(parents=True, exist_ok=True)
        write_csv(args.csv_out, result["samples"])

    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(result, indent=2), encoding="utf-8")

    print(f"mode={result['mode']}")
    print(f"apogee_m={result['apogee_m']:.3f}")
    print(f"time_to_apogee_s={result['time_to_apogee_s']:.3f}")
    print(f"max_command01={result['max_command01']:.3f}")
    print("note=minimal 1D analytical coast model; not a full sensor/estimator/hardware simulation")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
