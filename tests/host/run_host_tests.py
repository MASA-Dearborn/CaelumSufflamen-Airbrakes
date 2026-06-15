from __future__ import annotations

import csv
import importlib.util
import math
import re
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read_text(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def read_first_existing(*relative_paths: str) -> str:
    for relative_path in relative_paths:
        candidate = ROOT / relative_path
        if candidate.exists():
            return candidate.read_text(encoding="utf-8")
    joined = ", ".join(relative_paths)
    raise AssertionError(f"Could not locate any expected source file: {joined}")


def extract_constant(text: str, name: str) -> str:
    static_pattern = rf"static const [A-Za-z0-9_:\*]+ {name}\s*=\s*([^;\r\n]+)"
    match = re.search(static_pattern, text)
    if match:
        return match.group(1).strip()

    define_pattern = rf"#define {name}\s+([^\r\n]+)"
    match = re.search(define_pattern, text)
    if match:
        return match.group(1).strip()

    raise AssertionError(f"Could not find constant {name}")


def parse_numeric_literal(expr: str) -> float:
    cleaned = expr.replace("UL", "").replace("U", "").replace("f", "").replace("F", "")
    return float(cleaned)


def parse_int_constant(text: str, name: str) -> int:
    return int(round(parse_numeric_literal(extract_constant(text, name))))


def parse_float_constant(text: str, name: str) -> float:
    return parse_numeric_literal(extract_constant(text, name))


CONFIG_H = read_first_existing("config.h", "utils/config.h")
FLIGHT_PHASE_CPP = read_first_existing("flight_phase.cpp", "src/flight_phase.cpp")
COMMANDS_CPP = read_first_existing("commands.cpp", "utils/commands.cpp")
ACTUATOR_CPP = read_first_existing("actuator.cpp", "src/actuator.cpp")
SD_LOGGER_CPP = read_first_existing("sd_logger.cpp", "utils/sd_logger.cpp")
README_MD = read_text("README.md")
BUILDING_MD = read_text("BUILDING.md")
BUILD_WRAPPER_PS1 = read_text("tools/teensy41_arduino_cli.ps1")
AERO_FIT_SCRIPT = ROOT / "tests" / "host" / "policy_aero_empirical_fit.py"
COAST_SIM_SCRIPT = ROOT / "tests" / "host" / "policy_coast_sim.py"
REPLAY_VALIDATOR_SCRIPT = ROOT / "tests" / "host" / "replay_policy_validation.py"
FLIGHT_DATA_AUDIT_SCRIPT = ROOT / "tests" / "host" / "audit_previous_year_flight_data.py"
PREVIOUS_YEAR_DATA_DIR = ROOT / "flight data"


K_G = parse_float_constant(CONFIG_H, "kG")
POLICY_TARGET_APOGEE_M = parse_float_constant(CONFIG_H, "POLICY_TARGET_APOGEE_M")
POLICY_MIN_ALT_M = parse_float_constant(CONFIG_H, "POLICY_MIN_ALT_M")
POLICY_MIN_VZ_MPS = parse_float_constant(CONFIG_H, "POLICY_MIN_VZ_MPS")
POLICY_APOGEE_DEADBAND_M = parse_float_constant(CONFIG_H, "POLICY_APOGEE_DEADBAND_M")
POLICY_VEHICLE_MASS_KG = parse_float_constant(CONFIG_H, "POLICY_VEHICLE_MASS_KG")
POLICY_RHO_KGPM3 = parse_float_constant(CONFIG_H, "POLICY_RHO_KGPM3")
POLICY_CDA_BODY_M2 = parse_float_constant(CONFIG_H, "POLICY_CDA_BODY_M2")
POLICY_CDA_BRAKE_M2 = parse_float_constant(CONFIG_H, "POLICY_CDA_BRAKE_M2")
POLICY_MAX_COMMAND01 = parse_float_constant(CONFIG_H, "POLICY_MAX_COMMAND01")
POLICY_SLEW_PER_SEC = parse_float_constant(CONFIG_H, "POLICY_SLEW_PER_SEC")
POLICY_MAX_EST_AGE_MS = parse_int_constant(CONFIG_H, "POLICY_MAX_EST_AGE_MS")
POLICY_BISECTION_STEPS = parse_int_constant(CONFIG_H, "POLICY_BISECTION_STEPS")
POLICY_SIGMA_MARGIN_N = parse_float_constant(CONFIG_H, "POLICY_SIGMA_MARGIN_N")
POLICY_MAX_UNCERTAINTY_MARGIN_M = parse_float_constant(CONFIG_H, "POLICY_MAX_UNCERTAINTY_MARGIN_M")
CMD_BUF_N = parse_int_constant(CONFIG_H, "CMD_BUF_N")

FLIGHT_PHASE_BOOST_ACCEL_NORM_MPS2 = parse_float_constant(CONFIG_H, "FLIGHT_PHASE_BOOST_ACCEL_NORM_MPS2")
FLIGHT_PHASE_BOOST_MIN_ALT_M = parse_float_constant(CONFIG_H, "FLIGHT_PHASE_BOOST_MIN_ALT_M")
FLIGHT_PHASE_DESCENT_VZ_MPS = parse_float_constant(CONFIG_H, "FLIGHT_PHASE_DESCENT_VZ_MPS")

FLIGHT_PHASE_LAUNCH_MIN_VZ_MPS = parse_float_constant(FLIGHT_PHASE_CPP, "FLIGHT_PHASE_LAUNCH_MIN_VZ_MPS")
FLIGHT_PHASE_BURNOUT_ACCEL_NORM_MPS2 = parse_float_constant(FLIGHT_PHASE_CPP, "FLIGHT_PHASE_BURNOUT_ACCEL_NORM_MPS2")
FLIGHT_PHASE_COAST_MIN_ALT_M = parse_float_constant(FLIGHT_PHASE_CPP, "FLIGHT_PHASE_COAST_MIN_ALT_M")
FLIGHT_PHASE_COAST_MIN_VZ_MPS = parse_float_constant(FLIGHT_PHASE_CPP, "FLIGHT_PHASE_COAST_MIN_VZ_MPS")
FLIGHT_PHASE_BRAKE_MIN_COMMAND01 = parse_float_constant(FLIGHT_PHASE_CPP, "FLIGHT_PHASE_BRAKE_MIN_COMMAND01")
FLIGHT_PHASE_LAUNCH_CONFIRM_MS = parse_int_constant(FLIGHT_PHASE_CPP, "FLIGHT_PHASE_LAUNCH_CONFIRM_MS")
FLIGHT_PHASE_BURNOUT_CONFIRM_MS = parse_int_constant(FLIGHT_PHASE_CPP, "FLIGHT_PHASE_BURNOUT_CONFIRM_MS")
FLIGHT_PHASE_DESCENT_CONFIRM_MS = parse_int_constant(FLIGHT_PHASE_CPP, "FLIGHT_PHASE_DESCENT_CONFIRM_MS")
FLIGHT_PHASE_MIN_BOOST_DWELL_MS = parse_int_constant(FLIGHT_PHASE_CPP, "FLIGHT_PHASE_MIN_BOOST_DWELL_MS")
FLIGHT_PHASE_MIN_COAST_DWELL_MS = parse_int_constant(FLIGHT_PHASE_CPP, "FLIGHT_PHASE_MIN_COAST_DWELL_MS")


DISARMED = 0
SAFE = 1
ARMED = 2

IDLE = 0
BOOST = 1
COAST = 2
BRAKE = 3
DESCENT = 4


def clamp01(x: float) -> float:
    if not math.isfinite(x):
        return 0.0
    return min(1.0, max(0.0, x))


def policy_drag_k(command01: float) -> float:
    u = clamp01(command01)
    cda = POLICY_CDA_BODY_M2 + u * POLICY_CDA_BRAKE_M2
    if POLICY_RHO_KGPM3 <= 0.0 or POLICY_VEHICLE_MASS_KG <= 0.0 or cda < 0.0:
        return 0.0
    return (POLICY_RHO_KGPM3 * cda) / (2.0 * POLICY_VEHICLE_MASS_KG)


def policy_predict_apogee_m(h_m: float, v_mps: float, command01: float) -> float:
    if not math.isfinite(h_m) or not math.isfinite(v_mps):
        return math.nan
    if v_mps <= 0.0:
        return h_m
    k = policy_drag_k(command01)
    v2 = v_mps * v_mps
    if not math.isfinite(k) or k < 1.0e-7:
        return h_m + v2 / (2.0 * K_G)
    argument = 1.0 + (k * v2) / K_G
    if not math.isfinite(argument) or argument <= 0.0:
        return math.nan
    return h_m + math.log(argument) / (2.0 * k)


def policy_solve_command01(h_m: float, v_mps: float, target_apogee_m: float) -> float:
    if not math.isfinite(h_m) or not math.isfinite(v_mps) or not math.isfinite(target_apogee_m):
        return 0.0
    if v_mps <= 0.0:
        return 0.0

    u_max = clamp01(POLICY_MAX_COMMAND01)
    apogee_u0 = policy_predict_apogee_m(h_m, v_mps, 0.0)
    if not math.isfinite(apogee_u0):
        return 0.0
    if apogee_u0 <= target_apogee_m + POLICY_APOGEE_DEADBAND_M:
        return 0.0

    apogee_umax = policy_predict_apogee_m(h_m, v_mps, u_max)
    if not math.isfinite(apogee_umax):
        return 0.0
    if apogee_umax > target_apogee_m:
        return u_max

    lo = 0.0
    hi = u_max
    for _ in range(POLICY_BISECTION_STEPS):
        mid = 0.5 * (lo + hi)
        apogee_mid = policy_predict_apogee_m(h_m, v_mps, mid)
        if not math.isfinite(apogee_mid):
            return 0.0
        if apogee_mid > target_apogee_m:
            lo = mid
        else:
            hi = mid
    return clamp01(0.5 * (lo + hi))


def policy_uncertainty_margin_m(p00: float) -> float:
    if not math.isfinite(p00) or p00 < 0.0:
        return 0.0
    margin = POLICY_SIGMA_MARGIN_N * math.sqrt(p00)
    if not math.isfinite(margin) or margin < 0.0:
        return 0.0
    return min(margin, POLICY_MAX_UNCERTAINTY_MARGIN_M)


def load_module(module_name: str, path: Path):
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise AssertionError(f"Could not import module from {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


class PolicyRuntime:
    def __init__(self) -> None:
        self.prev_command01 = 0.0
        self.prev_ms = 0

    def reset(self, now_ms: int) -> None:
        self.prev_command01 = 0.0
        self.prev_ms = now_ms

    def apply_slew_limit(self, desired_command01: float, dt_s: float) -> float:
        desired = clamp01(desired_command01)
        if not math.isfinite(dt_s) or dt_s < 0.0:
            return self.prev_command01
        max_step = POLICY_SLEW_PER_SEC * dt_s
        limited = min(desired, self.prev_command01 + max_step)
        limited = max(limited, self.prev_command01 - max_step)
        limited = clamp01(limited)
        self.prev_command01 = limited
        return limited

    def compute(
        self,
        *,
        policy_runtime_enabled: bool,
        arm_state: int,
        software_arm_token: bool,
        phase: int,
        est_valid: bool,
        est_h_m: float,
        est_v_mps: float,
        est_p00: float,
        est_t_ms: int,
        now_ms: int,
    ) -> tuple[bool, float]:
        if not policy_runtime_enabled:
            self.reset(now_ms)
            return False, 0.0
        if arm_state != ARMED or not software_arm_token:
            self.reset(now_ms)
            return False, 0.0
        if phase not in (COAST, BRAKE):
            self.reset(now_ms)
            return False, 0.0
        if not est_valid or not math.isfinite(est_h_m) or not math.isfinite(est_v_mps):
            self.reset(now_ms)
            return False, 0.0
        if (now_ms - est_t_ms) > POLICY_MAX_EST_AGE_MS:
            self.reset(now_ms)
            return False, 0.0
        if est_h_m < POLICY_MIN_ALT_M or est_v_mps < POLICY_MIN_VZ_MPS:
            self.reset(now_ms)
            return False, 0.0

        uncertainty_margin = policy_uncertainty_margin_m(est_p00)
        target_eff_m = max(0.0, POLICY_TARGET_APOGEE_M - uncertainty_margin)

        dt_s = (now_ms - self.prev_ms) * 0.001
        self.prev_ms = now_ms
        if not math.isfinite(dt_s) or dt_s < 0.0 or dt_s > 1.0:
            dt_s = 0.0

        desired = policy_solve_command01(est_h_m, est_v_mps, target_eff_m)
        command01 = self.apply_slew_limit(desired, dt_s)
        return command01 > 0.0, command01


@dataclass
class PhaseInput:
    now_ms: int
    h_m: float
    v_mps: float
    a_norm: float
    policy_valid: bool = False
    policy_command01: float = 0.0
    est_valid: bool = True
    imu_valid: bool = True


class FlightPhaseDetector:
    def __init__(self) -> None:
        self.phase = IDLE
        self.launch_latched = False
        self.burnout_latched = False
        self.descent_latched = False
        self.launch_latch_ms = 0
        self.burnout_latch_ms = 0
        self.launch_candidate_ms = 0
        self.burnout_candidate_ms = 0
        self.descent_candidate_ms = 0

    @staticmethod
    def elapsed(now_ms: int, then_ms: int) -> int:
        return now_ms - then_ms

    def condition_confirmed(self, condition: bool, now_ms: int, dwell_ms: int, candidate_ms_name: str) -> bool:
        candidate_ms = getattr(self, candidate_ms_name)
        if not condition:
            setattr(self, candidate_ms_name, 0)
            return False
        if candidate_ms == 0:
            candidate_ms = now_ms
            setattr(self, candidate_ms_name, candidate_ms)
        return self.elapsed(now_ms, candidate_ms) >= dwell_ms

    def update(self, sample: PhaseInput) -> int:
        if not sample.est_valid or not sample.imu_valid:
            if not self.launch_latched:
                self.launch_candidate_ms = 0
                self.phase = IDLE
            elif not self.burnout_latched:
                self.burnout_candidate_ms = 0
                self.descent_candidate_ms = 0
            elif not self.descent_latched:
                self.descent_candidate_ms = 0
            return self.phase

        h_m = sample.h_m
        v_mps = sample.v_mps
        a_norm = sample.a_norm
        if not all(math.isfinite(x) for x in (h_m, v_mps, a_norm)):
            if not self.launch_latched:
                self.launch_candidate_ms = 0
                self.phase = IDLE
            elif not self.burnout_latched:
                self.burnout_candidate_ms = 0
                self.descent_candidate_ms = 0
            elif not self.descent_latched:
                self.descent_candidate_ms = 0
            return self.phase

        launch_by_accel = (
            a_norm >= FLIGHT_PHASE_BOOST_ACCEL_NORM_MPS2 and
            h_m >= FLIGHT_PHASE_BOOST_MIN_ALT_M
        )
        launch_by_motion = (
            h_m >= FLIGHT_PHASE_BOOST_MIN_ALT_M and
            v_mps >= FLIGHT_PHASE_LAUNCH_MIN_VZ_MPS
        )

        if not self.launch_latched:
            if self.condition_confirmed(
                launch_by_accel or launch_by_motion,
                sample.now_ms,
                FLIGHT_PHASE_LAUNCH_CONFIRM_MS,
                "launch_candidate_ms",
            ):
                self.launch_latched = True
                self.launch_latch_ms = sample.now_ms
                self.phase = BOOST
            else:
                self.phase = IDLE
            return self.phase

        if not self.burnout_latched:
            boost_dwell_met = self.elapsed(sample.now_ms, self.launch_latch_ms) >= FLIGHT_PHASE_MIN_BOOST_DWELL_MS
            pre_burnout_descent_candidate = (
                boost_dwell_met and
                h_m >= FLIGHT_PHASE_COAST_MIN_ALT_M and
                v_mps <= FLIGHT_PHASE_DESCENT_VZ_MPS
            )
            if self.condition_confirmed(
                pre_burnout_descent_candidate,
                sample.now_ms,
                FLIGHT_PHASE_DESCENT_CONFIRM_MS,
                "descent_candidate_ms",
            ):
                self.burnout_latched = True
                self.descent_latched = True
                self.burnout_latch_ms = sample.now_ms
                self.phase = DESCENT
                return self.phase

            burnout_candidate = (
                boost_dwell_met and
                a_norm <= FLIGHT_PHASE_BURNOUT_ACCEL_NORM_MPS2 and
                h_m >= FLIGHT_PHASE_COAST_MIN_ALT_M and
                v_mps >= FLIGHT_PHASE_COAST_MIN_VZ_MPS
            )
            if self.condition_confirmed(
                burnout_candidate,
                sample.now_ms,
                FLIGHT_PHASE_BURNOUT_CONFIRM_MS,
                "burnout_candidate_ms",
            ):
                self.burnout_latched = True
                self.burnout_latch_ms = sample.now_ms
                self.phase = COAST
            else:
                self.phase = BOOST
            return self.phase

        if not self.descent_latched:
            coast_dwell_met = self.elapsed(sample.now_ms, self.burnout_latch_ms) >= FLIGHT_PHASE_MIN_COAST_DWELL_MS
            descent_candidate = (
                coast_dwell_met and
                h_m >= FLIGHT_PHASE_COAST_MIN_ALT_M and
                v_mps <= FLIGHT_PHASE_DESCENT_VZ_MPS
            )
            if self.condition_confirmed(
                descent_candidate,
                sample.now_ms,
                FLIGHT_PHASE_DESCENT_CONFIRM_MS,
                "descent_candidate_ms",
            ):
                self.descent_latched = True
                self.phase = DESCENT
                return self.phase

        if self.descent_latched:
            self.phase = DESCENT
            return self.phase

        brake_active = sample.policy_valid and sample.policy_command01 >= FLIGHT_PHASE_BRAKE_MIN_COMMAND01
        self.phase = BRAKE if brake_active else COAST
        return self.phase


class CommandParserModel:
    def __init__(self, max_len: int) -> None:
        self.max_len = max_len
        self.buf = []
        self.discarding = False
        self.executed = []
        self.errors = []

    def feed(self, text: str) -> None:
        for ch in text:
            if ch in "\r\n":
                if self.discarding:
                    self.discarding = False
                    self.buf.clear()
                    continue
                if self.buf:
                    self.executed.append("".join(self.buf))
                    self.buf.clear()
                continue

            if self.discarding:
                continue

            if len(self.buf) + 1 < self.max_len:
                self.buf.append(ch)
            else:
                self.buf.clear()
                self.discarding = True
                self.errors.append("ERR,CMD_TOO_LONG")


def test_policy_valid_command_in_coast() -> None:
    runtime = PolicyRuntime()
    runtime.reset(0)
    valid, command01 = runtime.compute(
        policy_runtime_enabled=True,
        arm_state=ARMED,
        software_arm_token=True,
        phase=COAST,
        est_valid=True,
        est_h_m=120.0,
        est_v_mps=80.0,
        est_p00=1.0,
        est_t_ms=100,
        now_ms=200,
    )
    assert valid, "Expected a valid policy command in COAST when armed and enabled"
    assert command01 > 0.0, "Expected a non-zero command in overshoot conditions"


def test_policy_stays_invalid_when_disarmed() -> None:
    runtime = PolicyRuntime()
    runtime.reset(0)
    valid, command01 = runtime.compute(
        policy_runtime_enabled=True,
        arm_state=SAFE,
        software_arm_token=False,
        phase=COAST,
        est_valid=True,
        est_h_m=120.0,
        est_v_mps=80.0,
        est_p00=1.0,
        est_t_ms=100,
        now_ms=200,
    )
    assert not valid, "Disarmed or un-tokened state must not produce a valid policy command"
    assert command01 == 0.0


def test_phase_detector_reaches_coast_and_descent() -> None:
    detector = FlightPhaseDetector()
    assert detector.update(PhaseInput(0, 0.0, 0.0, 9.8)) == IDLE
    assert detector.update(PhaseInput(70, 3.0, 8.0, 30.0)) == IDLE
    assert detector.update(PhaseInput(140, 6.0, 20.0, 30.0)) == BOOST
    assert detector.update(PhaseInput(390, 40.0, 35.0, 12.0)) == BOOST
    assert detector.update(PhaseInput(520, 80.0, 30.0, 12.0)) == COAST
    assert detector.update(PhaseInput(770, 120.0, 0.0, 9.8)) == COAST
    assert detector.update(PhaseInput(1085, 118.0, -5.0, 9.8)) == DESCENT


def test_command_overflow_discard_until_newline() -> None:
    parser = CommandParserModel(CMD_BUF_N)
    parser.feed(("X" * CMD_BUF_N) + "STATUS\n")
    assert parser.errors == ["ERR,CMD_TOO_LONG"], "Expected one overflow error"
    assert parser.executed == [], "Overflow line suffix must not execute as a command"
    parser.feed("STATUS\n")
    assert parser.executed == ["STATUS"], "Parser must recover cleanly after newline"


def test_source_integrations_present() -> None:
    assert "writeMicroseconds" in ACTUATOR_CPP, "Actuator should write microseconds directly"
    assert "telemetry_warn_mask(state)" in SD_LOGGER_CPP, "SD logger should share the telemetry warning mask helper"
    assert "ARM <DISARMED|SAFE|ARMED>|POLICY 0|1" in COMMANDS_CPP, "Command help text should expose the new control path"
    assert "Runtime Control Path" in README_MD, "README should document the runtime arming/policy flow"
    assert "teensy:avr:teensy41" in BUILDING_MD, "BUILDING.md should pin the intended Teensy FQBN"
    assert "staged_sketch" in BUILD_WRAPPER_PS1, "Build wrapper should stage a normalized Arduino sketch"
    assert "previous_year_flight_data_audit.json" in CONFIG_H, "Config should reference the real-data audit result"


def test_policy_coast_sim_reduces_apogee_with_more_brake() -> None:
    module = load_module("policy_coast_sim", COAST_SIM_SCRIPT)

    closed = module.simulate_coast(
        h0_m=120.0,
        v0_mps=90.0,
        mode="closed",
        target_apogee_m=POLICY_TARGET_APOGEE_M,
        dt_s=0.02,
        max_time_s=30.0,
    )
    open_loop = module.simulate_coast(
        h0_m=120.0,
        v0_mps=90.0,
        mode="open",
        target_apogee_m=POLICY_TARGET_APOGEE_M,
        dt_s=0.02,
        max_time_s=30.0,
    )
    policy = module.simulate_coast(
        h0_m=120.0,
        v0_mps=90.0,
        mode="policy",
        target_apogee_m=POLICY_TARGET_APOGEE_M,
        dt_s=0.02,
        max_time_s=30.0,
    )

    assert open_loop["apogee_m"] < closed["apogee_m"], "Full brake should reduce apogee versus closed brake"
    assert policy["apogee_m"] <= closed["apogee_m"], "Policy simulation should not exceed the closed-brake apogee"
    assert 0.0 <= policy["max_command01"] <= 1.0, "Policy command should remain normalized"
    assert policy["max_command01"] > 0.0, "Overshoot scenario should deploy some brake in policy mode"


def test_previous_year_flight_data_audit_blocks_aero_fit() -> None:
    module = load_module("audit_previous_year_flight_data", FLIGHT_DATA_AUDIT_SCRIPT)
    result = module.audit_directory(PREVIOUS_YEAR_DATA_DIR)

    assert result["file_count"] >= 1
    assert not result["can_update_policy_cda_body_m2"]
    assert not result["can_update_policy_cda_brake_m2"]


def test_empirical_aero_fit_on_analytic_fixture() -> None:
    module = load_module("policy_aero_empirical_fit", AERO_FIT_SCRIPT)

    actual_apogee_m = 300.0
    phase_rows = [
        (50.0, 0.00, COAST),
        (90.0, 0.00, COAST),
        (130.0, 0.25, BRAKE),
        (170.0, 0.50, BRAKE),
        (210.0, 0.75, BRAKE),
        (250.0, 1.00, BRAKE),
    ]

    def required_velocity_for_apogee(delta_h_m: float, command01: float) -> float:
        cda_m2 = POLICY_CDA_BODY_M2 + command01 * POLICY_CDA_BRAKE_M2
        k_inv_m = (POLICY_RHO_KGPM3 * cda_m2) / (2.0 * POLICY_VEHICLE_MASS_KG)
        if k_inv_m < 1.0e-9:
            return math.sqrt(2.0 * K_G * delta_h_m)
        return math.sqrt((K_G / k_inv_m) * (math.exp(2.0 * k_inv_m * delta_h_m) - 1.0))

    with tempfile.TemporaryDirectory() as temp_dir:
        log_path = Path(temp_dir) / "LOG_FIXTURE.CSV"
        with log_path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=["t_us", "phase", "est_h", "est_v", "policy_cmd", "policy_valid"],
            )
            writer.writeheader()

            t_us = 0
            for h_m, command01, phase in phase_rows:
                delta_h_m = actual_apogee_m - h_m
                writer.writerow(
                    {
                        "t_us": t_us,
                        "phase": phase,
                        "est_h": h_m,
                        "est_v": required_velocity_for_apogee(delta_h_m, command01),
                        "policy_cmd": command01,
                        "policy_valid": 1 if command01 > 0.0 else 0,
                    }
                )
                t_us += 20000

            writer.writerow(
                {
                    "t_us": t_us,
                    "phase": DESCENT,
                    "est_h": actual_apogee_m,
                    "est_v": -5.0,
                    "policy_cmd": 0.0,
                    "policy_valid": 0,
                }
            )

        result = module.analyze_logs(
            [log_path],
            mass_kg=POLICY_VEHICLE_MASS_KG,
            rho_kgpm3=POLICY_RHO_KGPM3,
            current_body_cda_m2=POLICY_CDA_BODY_M2,
            current_brake_cda_m2=POLICY_CDA_BRAKE_M2,
            closed_cmd_threshold=0.05,
            open_cmd_threshold=0.20,
            min_alt_m=POLICY_MIN_ALT_M,
            min_vz_mps=POLICY_MIN_VZ_MPS,
        )

        aggregate = result["aggregate"]
        body_cda_m2 = aggregate["recommended_body_cda_m2"]
        brake_cda_m2 = aggregate["recommended_brake_cda_m2"]

        assert body_cda_m2 is not None, "Analytic fixture should recover body CDA"
        assert brake_cda_m2 is not None, "Analytic fixture should recover brake CDA"
        assert abs(body_cda_m2 - POLICY_CDA_BODY_M2) < 1.0e-4
        assert abs(brake_cda_m2 - POLICY_CDA_BRAKE_M2) < 1.0e-4


def run_test(name: str, fn) -> bool:
    try:
        fn()
    except Exception as exc:  # noqa: BLE001
        print(f"[FAIL] {name}: {exc}")
        return False
    print(f"[PASS] {name}")
    return True


def main() -> int:
    tests = [
        ("policy_valid_command_in_coast", test_policy_valid_command_in_coast),
        ("policy_invalid_when_disarmed", test_policy_stays_invalid_when_disarmed),
        ("phase_detector_reaches_coast_and_descent", test_phase_detector_reaches_coast_and_descent),
        ("command_overflow_discard_until_newline", test_command_overflow_discard_until_newline),
        ("source_integrations_present", test_source_integrations_present),
        ("policy_coast_sim_reduces_apogee_with_more_brake", test_policy_coast_sim_reduces_apogee_with_more_brake),
        ("previous_year_flight_data_audit_blocks_aero_fit", test_previous_year_flight_data_audit_blocks_aero_fit),
        ("empirical_aero_fit_on_analytic_fixture", test_empirical_aero_fit_on_analytic_fixture),
    ]

    failures = 0
    for name, fn in tests:
        if not run_test(name, fn):
            failures += 1

    if failures:
        print(f"{failures} host-side test(s) failed.")
        return 1

    print("All host-side tests passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
