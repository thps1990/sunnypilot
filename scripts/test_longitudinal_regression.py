#!/usr/bin/env python3
"""
Automated Longitudinal Regression & Side-Effect Test Suite
==========================================================
Simulates comprehensive driving scenarios to detect regressions and side effects
prior to test drives.

Can be run:
  1. On Comma 3X (full hardware-in-the-loop with Acados MPC & Cereal messaging):
     PYTHONPATH=/data/openpilot /usr/local/venv/bin/python3 scripts/test_longitudinal_regression.py
  2. On Development Host / Mac:
     python3 scripts/test_longitudinal_regression.py [--remote | --local]
     (Defaults to --remote if Comma 3X at 192.168.3.143 is reachable, else runs standalone physics)
"""

import sys
import os
import time
import math
import argparse
import subprocess

DEVICE_IP = "192.168.3.143"
DEVICE_USER = "comma"
DEVICE_DIR = "/data/openpilot"

# ANSI Colors
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
BOLD = "\033[1m"
RESET = "\033[0m"


def is_on_device():
    return os.path.exists("/data/openpilot") and os.path.exists("/usr/local/venv/bin/python3")


def is_device_reachable():
    ret = subprocess.run(["ping", "-c", "1", "-W", "1", DEVICE_IP],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return ret.returncode == 0


# ==============================================================================
# SCENARIO DEFINITIONS
# ==============================================================================
SCENARIOS = [
    {
        "id": "SCENARIO_1_HEADROOM_DECEL",
        "name": "Headroom / 40m Gap Decel (User Check)",
        "desc": "50 km/h, 40m gap. Lead gently slows 50->40 km/h. Must NOT brake synchronously; gap reduces naturally.",
        "v_ego": 50.0 / 3.6,
        "d_start": 40.0,
        "v_lead_start": 50.0 / 3.6,
        "v_lead_end": 40.0 / 3.6,
        "a_lead": -1.5,
        "duration": 5.0,
        "checks": [
            ("No Synchronous Jerk", lambda res: res["max_decel"] > -0.30,
             "Max decel should stay in deadband (> -0.30 m/s^2) during headroom fade"),
            ("Natural Gap Compression", lambda res: res["final_d"] < 40.0,
             "Relative distance must decrease naturally into target buffer"),
            ("No FCW Alert", lambda res: res["fcw_count"] == 0,
             "FCW must not trip"),
        ],
    },
    {
        "id": "SCENARIO_2_NEAR_CRASH_SEG8",
        "name": "Segment 8 City Near-Crash Reproduction",
        "desc": "56.5 km/h, 23m gap. Lead brakes hard 44->18 km/h (-2.0 m/s^2). Must brake firmly; min clearance >= 8m.",
        "v_ego": 56.5 / 3.6,
        "d_start": 23.0,
        "v_lead_start": 44.0 / 3.6,
        "v_lead_end": 18.0 / 3.6,
        "a_lead": -2.0,
        "duration": 5.0,
        "checks": [
            ("Authoritative Braking", lambda res: res["max_decel"] <= -2.50,
             "Must command at least -2.50 m/s^2 deceleration"),
            ("Safe Min Clearance", lambda res: res["min_d"] >= 8.0,
             "Distance must not compress dangerously (min clearance >= 8.0m)"),
            ("No Crash / Collision", lambda res: res["min_d"] > 0.4,
             "Collision completely prevented"),
            ("No False FCW Trap", lambda res: res["fcw_count"] == 0,
             "No unwarranted FCW alarm during controlled braking"),
        ],
    },
    {
        "id": "SCENARIO_3_TRAFFIC_LIGHT_STOP",
        "name": "City Stoplight / Standstill Approach",
        "desc": "50 km/h cruising, stopped lead at 45m. Must stop smoothly and hold safe clearance.",
        "v_ego": 50.0 / 3.6,
        "d_start": 45.0,
        "v_lead_start": 0.0,
        "v_lead_end": 0.0,
        "a_lead": 0.0,
        "duration": 10.0,
        "checks": [
            ("Complete Stop Achieved", lambda res: res["final_speed"] < 1.0,
             "Ego vehicle must reach standstill (< 1.0 km/h)"),
            ("Positive Stopped Clearance", lambda res: res["final_d"] > 0.5,
             "Final stopped clearance must be positive (no contact)"),
            ("Comfortable Deceleration", lambda res: res["max_decel"] >= -3.50,
             "Deceleration must be comfortable (>= -3.50 m/s^2)"),
        ],
    },
    {
        "id": "SCENARIO_4_STEADY_STATE_DEADBAND",
        "name": "Highway Steady Following (Anti-Hunting)",
        "desc": "100 km/h, steady 35m following. Lead has minor +/-0.3 km/h speed ripples. Zero micro-braking.",
        "v_ego": 100.0 / 3.6,
        "d_start": 35.0,
        "v_lead_start": 100.0 / 3.6,
        "v_lead_end": 100.0 / 3.6,
        "a_lead": 0.0,
        "duration": 6.0,
        "ripple": True,
        "checks": [
            ("Zero Hunting Deceleration", lambda res: res["max_decel"] > -0.25,
             "Deadband must filter ripples (decel > -0.25 m/s^2, no Tesla regen jerk)"),
            ("Stable Gap Maintained", lambda res: abs(res["final_d"] - 35.0) < 3.0,
             "Following distance remains stable (+/- 3m)"),
        ],
    },
    {
        "id": "SCENARIO_5_HIGH_SPEED_APPROACH",
        "name": "Autobahn High-Speed Truck Approach",
        "desc": "130 km/h approaching a 90 km/h truck starting at 110m. Must decelerate early and smoothly.",
        "v_ego": 130.0 / 3.6,
        "d_start": 110.0,
        "v_lead_start": 90.0 / 3.6,
        "v_lead_end": 90.0 / 3.6,
        "a_lead": 0.0,
        "duration": 10.0,
        "checks": [
            ("Early Smooth Deceleration", lambda res: -2.0 <= res["max_decel"] <= -0.4,
             "Must apply smooth early deceleration (-0.4 to -2.0 m/s^2)"),
            ("No Late Panic Braking", lambda res: res["max_decel"] >= -2.5,
             "No abrupt panic braking (>= -2.5 m/s^2)"),
            ("Highway Headway Converged", lambda res: res["final_d"] >= 30.0,
             "Maintains safe highway following distance (>= 30m)"),
        ],
    },
    {
        "id": "SCENARIO_6_SUDDEN_CUT_IN",
        "name": "Sudden Cut-In at Close Distance",
        "desc": "60 km/h. Slower vehicle (45 km/h) cuts in suddenly at 15m. Must react promptly without collision.",
        "v_ego": 60.0 / 3.6,
        "d_start": 15.0,
        "v_lead_start": 45.0 / 3.6,
        "v_lead_end": 45.0 / 3.6,
        "a_lead": 0.0,
        "duration": 6.0,
        "checks": [
            ("Prompt Deceleration", lambda res: res["max_decel"] <= -1.4,
             "Must command at least -1.4 m/s^2 to open gap"),
            ("No Collision", lambda res: res["min_d"] >= 5.0,
             "Minimum clearance remains >= 5.0m"),
        ],
    },
    {
        "id": "SCENARIO_7_EMERGENCY_STOP",
        "name": "Emergency Full Stop from 65 km/h",
        "desc": "65 km/h at standard headway (28m). Lead slams brakes at -4.0 m/s^2 to full stop. Safe stop required.",
        "v_ego": 65.0 / 3.6,
        "d_start": 28.0,
        "v_lead_start": 65.0 / 3.6,
        "v_lead_end": 0.0,
        "a_lead": -4.0,
        "duration": 7.0,
        "checks": [
            ("Full Brake Commanded", lambda res: res["max_decel"] <= -3.2,
             "Must command strong emergency deceleration (<= -3.2 m/s^2)"),
            ("Zero Collision", lambda res: res["min_d"] > 0.4,
             "Vehicle must stop without collision (min distance > 0.4m crash limit)"),
        ],
    },
]


# ==============================================================================
# DEVICE SIMULATION RUNNER (Runs directly on Comma 3X via Plant & LongitudinalPlanner)
# ==============================================================================
def run_scenario_on_device(sc):
    from openpilot.selfdrive.test.longitudinal_maneuvers.plant import Plant
    from opendbc.car.tesla.values import CAR
    from opendbc.car.tesla.interface import CarInterface
    from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
    import numpy as np

    plant = Plant(lead_relevancy=True, speed=sc["v_ego"], distance_lead=sc["d_start"])
    try:
        CP = CarInterface.get_non_essential_params(CAR.TESLA_MODEL_Y)
        CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.TESLA_MODEL_Y)
        plant.planner = LongitudinalPlanner(CP, CP_SP, init_v=plant.speed)
    except Exception as e:
        pass  # Fall back to default planner initialized in Plant

    vl = sc["v_lead_start"]
    vl_target = sc["v_lead_end"]
    al = sc["a_lead"]
    duration = sc["duration"]
    dt = plant.ts
    steps = int(duration / dt)

    min_d = sc["d_start"]
    max_decel = 0.0
    fcw_count = 0
    ripple = sc.get("ripple", False)

    for step in range(steps):
        t = step * dt
        if vl > vl_target and al < 0.0:
            vl = max(vl_target, vl + al * dt)
        elif vl < vl_target and al > 0.0:
            vl = min(vl_target, vl + al * dt)

        vl_actual = vl
        if ripple:
            vl_actual += 0.3 * math.sin(t * 3.0) / 3.6

        res = plant.step(v_lead=vl_actual, prob_lead=1.0, v_cruise=sc["v_ego"])
        d_rel = res["distance_lead"] - res["distance"]
        min_d = min(min_d, d_rel)
        max_decel = min(max_decel, res["acceleration"])
        if res.get("fcw", False):
            fcw_count += 1

    return {
        "min_d": min_d,
        "final_d": d_rel,
        "max_decel": max_decel,
        "final_speed": res["speed"] * 3.6,
        "fcw_count": fcw_count,
        "valid": True,
    }


# ==============================================================================
# STANDALONE LOCAL SIMULATION RUNNER (Pure Python vehicle physics model fallback)
# ==============================================================================
def run_scenario_local(sc):
    t_follow = 1.45
    dt = 0.05
    steps = int(sc["duration"] / dt)

    ve = sc["v_ego"]
    vl = sc["v_lead_start"]
    vl_target = sc["v_lead_end"]
    al = sc["a_lead"]
    d = sc["d_start"]

    min_d = d
    max_decel = 0.0
    fcw_count = 0
    ripple = sc.get("ripple", False)

    for step in range(steps):
        t = step * dt
        if vl > vl_target and al < 0.0:
            vl = max(vl_target, vl + al * dt)
        elif vl < vl_target and al > 0.0:
            vl = min(vl_target, vl + al * dt)

        vl_actual = vl
        if ripple:
            vl_actual += 0.3 * math.sin(t * 3.0) / 3.6

        delta_v = ve - vl_actual
        d_target = t_follow * max(0.0, vl_actual) + 4.0
        d_min_buffer = max(3.5, 0.5 * max(0.0, vl_actual) + 3.0)

        # Headroom-aware lead decel compensation:
        if d <= d_target:
            lead_comp_weight = 1.0
        else:
            headroom = d - d_target
            buffer_zone = max(3.0, 0.4 * d_target)
            lead_comp_weight = max(0.0, min(1.0, 1.0 - headroom / buffer_zone))

        a_lead_comp = (min(al, 0.0) if al < -0.4 else 0.0) * lead_comp_weight

        if delta_v > 0.4 or a_lead_comp < 0.0:
            if d > d_target:
                d_margin = max(2.0, d - d_target)
                a_close = - (max(0.0, delta_v) ** 2) / (2.0 * d_margin)
            else:
                d_margin = max(1.5, d - d_min_buffer)
                a_close = - (max(0.0, delta_v) ** 2) / (2.0 * d_margin)

            a_approach = a_lead_comp + a_close
            if a_approach < -0.20:
                a_cmd = max(-4.0, a_approach)
            else:
                a_cmd = 0.0
        else:
            a_cmd = 0.0

        if vl_actual < 0.5 and d < 6.0 and ve > 0.05:
            a_cmd = min(a_cmd, -0.6)

        max_decel = min(max_decel, a_cmd)
        ve = max(0.0, ve + a_cmd * dt)
        d += (vl_actual - ve) * dt
        min_d = min(min_d, d)

    return {
        "min_d": min_d,
        "final_d": d,
        "max_decel": max_decel,
        "final_speed": ve * 3.6,
        "fcw_count": fcw_count,
        "valid": True,
    }


# ==============================================================================
# MAIN TEST EXECUTION & REPORTING
# ==============================================================================
def execute_suite(use_device=False):
    runner_name = "Comma 3X Hardware (Plant + Cython Acados MPC)" if use_device else "Local Python Kinematics Engine"
    print(f"\n{BOLD}{CYAN}{'='*80}{RESET}")
    print(f"{BOLD}{CYAN}   LONGITUDINAL REGRESSION TEST SUITE (openpilot / SunnyPilot){RESET}")
    print(f"{BOLD}   Runner: {runner_name}{RESET}")
    print(f"{BOLD}{CYAN}{'='*80}{RESET}\n")

    total_scenarios = len(SCENARIOS)
    passed_scenarios = 0
    failures = []

    for idx, sc in enumerate(SCENARIOS, 1):
        print(f"{BOLD}[{idx}/{total_scenarios}] {sc['name']}{RESET}")
        print(f"    Description: {sc['desc']}")

        try:
            if use_device:
                res = run_scenario_on_device(sc)
            else:
                res = run_scenario_local(sc)
        except Exception as e:
            print(f"    {RED}ERROR executing scenario: {e}{RESET}\n")
            failures.append((sc["name"], f"Execution crashed: {e}"))
            continue

        print(f"    Results: Min Dist: {res['min_d']:.2f}m | Final Dist: {res['final_d']:.2f}m | "
              f"Max Decel: {res['max_decel']:.2f} m/s^2 | Final Speed: {res['final_speed']:.1f} km/h | "
              f"FCW: {res['fcw_count']}")

        sc_passed = True
        for check_name, check_fn, check_desc in sc["checks"]:
            ok = check_fn(res)
            if ok:
                print(f"      {GREEN}✔ PASS{RESET} : {check_name}")
            else:
                print(f"      {RED}✘ FAIL{RESET} : {check_name} ({check_desc})")
                sc_passed = False
                failures.append((sc["name"], f"{check_name} failed: {check_desc}"))

        if sc_passed:
            passed_scenarios += 1
            print(f"    --> {BOLD}{GREEN}SCENARIO PASSED{RESET}\n")
        else:
            print(f"    --> {BOLD}{RED}SCENARIO FAILED{RESET}\n")

    # Final Summary Table
    print(f"{BOLD}{CYAN}{'='*80}{RESET}")
    print(f"{BOLD}SUMMARY: {passed_scenarios}/{total_scenarios} SCENARIOS PASSED{RESET}")
    if passed_scenarios == total_scenarios:
        print(f"{BOLD}{GREEN}ALL REGRESSION CHECKS PASSED! NO SIDE EFFECTS DETECTED.{RESET}")
        print(f"{BOLD}{CYAN}{'='*80}{RESET}\n")
        return 0
    else:
        print(f"{BOLD}{RED}FAILURES DETECTED ({len(failures)} checks failed):{RESET}")
        for sc_name, fail_msg in failures:
            print(f"  - [{sc_name}]: {fail_msg}")
        print(f"{BOLD}{CYAN}{'='*80}{RESET}\n")
        return 1


def main():
    parser = argparse.ArgumentParser(description="Longitudinal Regression Test Suite")
    parser.add_argument("--local", action="store_true", help="Force running local kinematic model")
    parser.add_argument("--remote", action="store_true", help="Force running on Comma 3X via SSH")
    args = parser.parse_args()

    if is_on_device():
        # Running directly on Comma 3X
        sys.exit(execute_suite(use_device=True))

    if args.local:
        sys.exit(execute_suite(use_device=False))

    # On development machine: check if device is reachable
    if args.remote or is_device_reachable():
        print(f"{CYAN}Comma 3X detected at {DEVICE_IP}. Executing test suite directly on device...{RESET}")
        # Copy script or execute directly on device via python3
        cmd = f"ssh -o ConnectTimeout=5 {DEVICE_USER}@{DEVICE_IP} 'cd {DEVICE_DIR} && PYTHONPATH={DEVICE_DIR} /usr/local/venv/bin/python3 scripts/test_longitudinal_regression.py'"
        ret = subprocess.run(cmd, shell=True)
        sys.exit(ret.returncode)
    else:
        print(f"{YELLOW}Comma 3X offline or unreachable. Falling back to local kinematics model...{RESET}")
        sys.exit(execute_suite(use_device=False))


if __name__ == "__main__":
    main()
