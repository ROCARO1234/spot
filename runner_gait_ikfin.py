#!/usr/bin/env python3
"""Corrected standalone SM5 runner. Default = offline, no I2C imports.

python3 runner_gait_ikfin.py --sim
python3 runner_gait_ikfin.py --dry-run --vx -0.003 --duration 12
python3 runner_gait_ikfin.py --check

Read docs/usage.md before using --hardware. Real hardware is never activated
by running the script without that flag, importing it, or opening the preview.
"""
from __future__ import annotations

import argparse
from contextlib import nullcontext
import json
import signal
import sys
import time

from controller import MotionController
from keyboard_control import DeadmanKeys, Keyboard
from robot_config import DEFAULT_CONFIG, SafetyError, finite_number, load_config
from robot_io import DryRunOutput, HardwareOutput
from simulation import preview, simulate, summary, write_csv
from telemetry import DEFAULT_PORT, TelemetryServer


def parser():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = p.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", help="offline validation; this is the default")
    mode.add_argument("--sim", action="store_true", help="offline validation + animated kinematic preview")
    mode.add_argument("--hardware", action="store_true", help="explicit real output; requires reviewed configuration")
    p.add_argument("--config", default=str(DEFAULT_CONFIG))
    p.add_argument("--poses", help="optional pose file, otherwise beside robot.json")
    p.add_argument("--check", action="store_true", help="check configuration without touching hardware")
    p.add_argument("--keyboard", action="store_true", help="WASD/QE with repeat timeout; X emergency stop")
    p.add_argument("--real-time", action="store_true", help="run dry output against the wall clock")
    p.add_argument("--telemetry", action="store_true", help="read-only live command feed on localhost; implies real-time")
    p.add_argument("--telemetry-port", type=int, default=DEFAULT_PORT, help="localhost telemetry port, default 8765")
    p.add_argument("--duration", type=float, default=12., help="command duration; normal landing/settling is additional")
    p.add_argument("--vx", type=float, help="nominal forward body velocity, metres/second")
    p.add_argument("--vy", type=float, help="nominal leftward body velocity, metres/second")
    p.add_argument("--wz", type=float, help="nominal counterclockwise yaw rate, radians/second")
    p.add_argument("--initial-pose", choices=("STAND",), help="confirmed supported starting pose; REQUIRED with --hardware")
    p.add_argument("--render", metavar="PNG", help="save an offline preview image")
    p.add_argument("--csv", metavar="CSV", help="save OFFLINE command trace (does not record real feedback)")
    return p


def check(config):
    MotionController(config)  # verify neutral IK as well as the JSON schema
    print("Configuration + STAND IK: OK")
    for name in config.poses:
        try:
            config.pose(name)
            print(f"Pose {name}: within configured software limits")
        except SafetyError as exc:
            print(f"Pose {name}: DISABLED — {exc}")
    blockers = config.hardware_blockers()
    if blockers:
        print("Hardware: BLOCKED (offline operation is available)")
        for issue in blockers:
            print(" - " + issue)
    else:
        print("Hardware review flags: set by user; physical calibration still cannot be verified here")


def run_realtime(config, command, duration, initial_pose, *, hardware=False, keyboard=False, telemetry_port=None):
    controller = MotionController(config, initial_pose)
    controller.set_command(*(command if not keyboard else (0., 0., 0.)))
    output = HardwareOutput(config) if hardware else DryRunOutput(config)
    stream = (TelemetryServer(config, port=telemetry_port, mode="HARDWARE" if hardware else "DRY_RUN")
              if telemetry_port is not None else None)
    keyboard_context = Keyboard() if keyboard else nullcontext(None)
    deadman = DeadmanKeys(config.control["keyboard_timeout_s"], config.control["keyboard_speed_mps"],
                         config.control["keyboard_yaw_rps"])
    period = 1/config.control["rate_hz"]
    old_signals = {}
    try:
        # Establish and validate terminal state before turning power on.
        with keyboard_context as keys:
            for name in ("SIGTERM", "SIGHUP"):
                sig = getattr(signal, name, None)
                if sig is not None:
                    old_signals[sig] = signal.signal(sig, _interrupt)
            if stream is not None:
                stream.open()  # port/setup failures must occur before motor power
                print(f"Read-only telemetry: 127.0.0.1:{stream.port} (use viewer_live.py / SSH tunnel)")
            output.open(controller.last_pose)
            print("HARDWARE ARMED — support the robot; X/Ctrl+C requests power-off" if hardware else "DRY RUN — no motor access")
            if keyboard:
                print("Hold W/S forward/back, A/D left/right, Q/E turn. Space normal stop; X emergency; Esc stop + exit.")
                print("1 LIE (normally disabled), 2 CROUCH_READY, 3 STAND, 4 UP_READY (normally disabled).")
            start = previous = time.monotonic()
            last_publish = start
            if stream is not None:
                stream.publish(controller, 0.)
            next_tick = start + period
            last_status = start - 1
            stop_started = None
            while True:
                now = time.monotonic()
                if now < next_tick:
                    time.sleep(next_tick-now)
                now = time.monotonic()
                dt = now - previous
                previous = now
                # Skip missed deadlines, never advance gait with fictional dt.
                next_tick = now + period
                finite_number(dt, "real control-loop gap", 1e-6, config.control["max_loop_gap_s"])
                if keys:
                    for key in keys.read(now):
                        deadman.feed(key, now)
                        if key in ("x", "\x03"):
                            raise KeyboardInterrupt
                        if key == "\x1b":
                            stop_started = now if stop_started is None else stop_started
                        if key in ("1", "2", "3", "4") and stop_started is None:
                            try:
                                controller.request_pose({"1": "LIE", "2": "CROUCH_READY", "3": "STAND", "4": "UP_READY"}[key])
                            except SafetyError as exc:
                                print(f"Pose refused: {exc}")
                    try:
                        controller.set_command(*deadman.command(now))
                    except SafetyError:
                        controller.set_command()
                if now-start >= duration and stop_started is None:
                    stop_started = now
                if stop_started is not None:
                    controller.set_command()
                pose = controller.update(dt)
                output.write(pose, dt)
                if stream is not None and (now-last_publish >= .1 or controller.settled and stop_started is not None):
                    # Publish only after the entire frame was accepted by output.
                    stream.publish(controller, now-start)
                    last_publish = now
                if now-last_status >= 1.:
                    print(f"t={now-start:5.1f}s  {controller.state:16s}  "
                          f"swing={','.join(controller.frame.swing_legs) or '-'}  "
                          f"tracking={controller.tracking_error_deg:.2f}deg")
                    last_status = now
                if stop_started is not None and controller.settled:
                    break
                if stop_started is not None and now-stop_started > 3*controller.planner.cycle_time + 10:
                    raise SafetyError("Normal stop exceeded its deadline")
    finally:
        # Motor shutdown always precedes network cleanup, even on an exception.
        try:
            output.close()
        finally:
            for sig, previous_handler in old_signals.items():
                signal.signal(sig, previous_handler)
            if stream is not None:
                try:
                    stream.close()
                except Exception as exc:
                    print(f"Telemetry cleanup error: {exc}", file=sys.stderr)
        for error in output.cleanup_errors:
            print("POWER-OFF ERROR: " + error + " — use the physical power disconnect", file=sys.stderr)
        if output.cleanup_errors:
            raise SafetyError("Shutdown was incomplete; use the physical power disconnect")


def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    if args.hardware and args.real_time:
        p.error("--real-time is offline-only; --hardware is already real-time")
    if (args.hardware or args.keyboard or args.real_time or args.telemetry) and (args.render or args.csv):
        p.error("--render/--csv export an offline trajectory; use viewer_live.py for live commands")
    if args.sim and (args.keyboard or args.real_time or args.telemetry):
        p.error("--sim replays an offline trajectory; use --dry-run --keyboard --telemetry and viewer_live.py for live display")
    try:
        finite_number(args.duration, "duration", .02, 300.)
        config = load_config(args.config, args.poses)
        if args.check:
            check(config)
            return 0
        explicit_velocity = any(v is not None for v in (args.vx, args.vy, args.wz))
        command = tuple(0. if v is None else v for v in (args.vx, args.vy, args.wz))
        if not explicit_velocity and not args.hardware and not args.keyboard:
            command = (.003, 0., 0.)  # offline demonstration only
        for value in command:
            finite_number(value, "velocity", -10., 10.)
        initial = args.initial_pose or "STAND"
        if args.hardware:
            if not args.initial_pose:
                raise SafetyError("--hardware requires --initial-pose matching the supported robot's physical pose")
            blockers = config.hardware_blockers()
            if blockers:
                raise SafetyError("HARDWARE BLOCKED:\n  " + "\n  ".join(blockers))
            print("Offline trajectory preflight; motors remain OFF.")
            if args.keyboard:
                if initial != "STAND":
                    raise SafetyError("Keyboard hardware tests must initially be aligned in STAND")
                speed, yaw = config.control["keyboard_speed_mps"], config.control["keyboard_yaw_rps"]
                preflight_commands = [(vx, vy, wz) for vx in (-speed, 0., speed)
                                      for vy in (-speed, 0., speed) for wz in (-yaw, 0., yaw)]
                for index, candidate in enumerate(preflight_commands, 1):
                    print(f"Preflight {index}/{len(preflight_commands)}: {candidate}", flush=True)
                    simulate(config, candidate, duration=2*crawl_cycle(config))
            else:
                simulate(config, command, args.duration, initial)
            print("Offline preflight: OK. This does not verify real calibration, balance or contacts.")
        if args.hardware or args.keyboard or args.real_time or args.telemetry:
            run_realtime(config, command, args.duration, initial, hardware=args.hardware, keyboard=args.keyboard,
                         telemetry_port=args.telemetry_port if args.telemetry else None)
        else:
            records = simulate(config, command, args.duration, initial)
            print("OFFLINE / NO I2C")
            print(json.dumps(summary(records), indent=2))
            if args.csv:
                print("Offline CSV: " + str(write_csv(records, args.csv)))
            if args.sim or args.render:
                preview(config, records, output=args.render, animate=args.sim)
        return 0
    except KeyboardInterrupt:
        print("Emergency stop requested. Output closed; do not assume the robot can support itself without torque.")
        return 130
    except (SafetyError, OSError, ImportError, ValueError) as exc:
        print(f"STOPPED: {exc}", file=sys.stderr)
        return 2


def _interrupt(*_):
    raise KeyboardInterrupt


def crawl_cycle(config):
    return 4*(config.gait["transfer_time_s"] + config.gait["swing_time_s"])


if __name__ == "__main__":
    raise SystemExit(main())
