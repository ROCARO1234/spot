#!/usr/bin/env python3
"""Reproducible offline validation. Never imports or opens real hardware."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import time
import unittest

import numpy as np

from robot_config import load_config
from simulation import simulate, summary

ROOT = Path(__file__).resolve().parent


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default=str(ROOT/"validation_results.json"))
    args = parser.parse_args(argv)
    began = time.perf_counter()
    suite = unittest.defaultTestLoader.discover(str(ROOT/"tests"))
    result = unittest.TextTestRunner(verbosity=1).run(suite)
    report = {"generated_at_utc": datetime.now(timezone.utc).isoformat(),
              "hardware_exercised": False, "python": platform.python_version(), "numpy": np.__version__,
              "unit_tests": {"run": result.testsRun, "failures": len(result.failures),
                             "errors": len(result.errors), "skipped": len(result.skipped)},
              "profiles": [], "source_sha256": {}}
    paths = [*ROOT.glob("*.py"), *(ROOT/"tests").glob("*.py"), *(ROOT/"config").glob("*.json")]
    for path in sorted(paths):
        report["source_sha256"][path.relative_to(ROOT).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
    config = load_config()
    cycle = 4*(config.gait["transfer_time_s"]+config.gait["swing_time_s"])
    commands = [(vx, vy, wz) for vx in (-config.gait["max_translation_mps"], 0., config.gait["max_translation_mps"])
                for vy in (-config.gait["max_translation_mps"], 0., config.gait["max_translation_mps"])
                for wz in (-config.gait["max_yaw_rps"], 0., config.gait["max_yaw_rps"])]
    for index, command in enumerate(commands, 1):
        entry = {"requested_velocity": list(command)}
        try:
            records = simulate(config, command, 2*cycle)
            entry.update(summary(records))
            entry["passed"] = entry["final_state"] == "STAND" and entry["max_swing_legs"] <= 1
        except Exception as exc:
            entry.update({"passed": False, "error": f"{type(exc).__name__}: {exc}"})
        report["profiles"].append(entry)
        print(f"Profile {index}/{len(commands)} {command}: {'OK' if entry['passed'] else 'FAILED'}", flush=True)
    report["passed"] = result.wasSuccessful() and all(p["passed"] for p in report["profiles"])
    report["wall_time_seconds"] = round(time.perf_counter()-began, 3)
    path = Path(args.output).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(f"Report: {path}\nPassed: {report['passed']}; real hardware exercised: false")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
