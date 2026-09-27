#!/usr/bin/env python3
"""Laptop viewer: read command angles through localhost or an SSH tunnel.

This program cannot arm or control motors. Closing it does not stop the robot.
Start the runner with --telemetry first. See LAPTOP_LIVE_RO.md.
"""
from __future__ import annotations

import argparse
import sys
from urllib.error import URLError

from controller import MotionController
from robot_config import SafetyError
from simulation import preview
from telemetry import DEFAULT_PORT, TelemetryClient, command_record


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="local/tunnel port, default 8765")
    parser.add_argument("--snapshot", metavar="PNG", help="save the current received frame instead of opening a GUI")
    args = parser.parse_args(argv)
    client = None
    try:
        client = TelemetryClient(port=args.port)
        config = client.connect()
        if args.snapshot:
            record, label, fresh = client.sample()
            if not fresh:
                raise SafetyError("No current command frame available; start the runner with --telemetry first")
            preview(config, [record], output=args.snapshot, live_source=client.sample)
            print("Saved command snapshot: " + args.snapshot)
        else:
            print("Vizualizare fără control al motoarelor. Folosește tastatura în terminalul runnerului.")
            print("Închiderea ferestrei NU oprește robotul. X/Ctrl+C în terminal sau oprire fizică pentru urgență.")
            client.start()
            initial = command_record(MotionController(config), 0.)
            preview(config, [initial], animate=True, live_source=client.sample)
        return 0
    except KeyboardInterrupt:
        return 130
    except (OSError, URLError, SafetyError, ImportError, ValueError) as exc:
        print(f"VIEWER: {exc}\nPornește runnerul cu --telemetry; pentru Raspberry Pi verifică tunelul SSH.", file=sys.stderr)
        return 2
    finally:
        if client is not None:
            client.close()


if __name__ == "__main__":
    raise SystemExit(main())
