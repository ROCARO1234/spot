"""Nonblocking terminal input with a repeat timeout; no OS key-state guessing."""
from __future__ import annotations

import os
import sys
import time


class Keyboard:
    def __init__(self, stream=None):
        self.stream = stream or sys.stdin
        self.old = None
        self.buffer = ""
        self.escape_since = None

    def __enter__(self):
        if not self.stream.isatty():
            raise ValueError("--keyboard requires an interactive terminal; use --duration for a timed test")
        if os.name != "nt":
            import termios
            import tty
            self.fd = self.stream.fileno()
            self.old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def __exit__(self, *_):
        if self.old is not None:
            import termios
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def read(self, now=None):
        now = time.monotonic() if now is None else now
        if os.name == "nt":
            import msvcrt
            keys = []
            while msvcrt.kbhit():
                key = msvcrt.getwch()
                if key in ("\x00", "\xe0"):
                    key = {"H": "w", "P": "s", "K": "a", "M": "d"}.get(msvcrt.getwch(), "")
                keys.append(key.lower())
            return keys
        import select
        if select.select([self.fd], [], [], 0)[0]:
            data = os.read(self.fd, 1024)
            if not data:
                return ["\x1b"]  # EOF requests controlled stop and exit
            self.buffer += data.decode("utf-8", errors="ignore")
        return self.parse_buffer(now)

    def parse_buffer(self, now):
        """Consume a whole ANSI sequence so its digits cannot trigger poses."""
        keys = []
        while self.buffer:
            if self.buffer[0] != "\x1b":
                keys.append(self.buffer[0].lower())
                self.buffer = self.buffer[1:]
                continue
            if self.buffer.startswith("\x1b["):
                end = next((i for i in range(2, len(self.buffer)) if "@" <= self.buffer[i] <= "~"), None)
                if end is not None:
                    key = {"A": "w", "B": "s", "D": "a", "C": "d"}.get(self.buffer[end])
                    if key:
                        keys.append(key)
                    self.buffer = self.buffer[end+1:]
                    self.escape_since = None
                    continue
            elif self.buffer.startswith("\x1bO") and len(self.buffer) >= 3:
                key = {"A": "w", "B": "s", "D": "a", "C": "d"}.get(self.buffer[2])
                if key:
                    keys.append(key)
                self.buffer = self.buffer[3:]
                self.escape_since = None
                continue
            if self.escape_since is None:
                self.escape_since = now
            if now - self.escape_since < .05:
                break
            keys.append("\x1b")
            self.buffer = "" if self.buffer.startswith(("\x1b[", "\x1bO")) else self.buffer[1:]
            self.escape_since = None
        return keys


class DeadmanKeys:
    """Terminal key repeat is not key-up; expires old motion commands."""
    def __init__(self, timeout, speed, yaw):
        self.timeout, self.speed, self.yaw = timeout, speed, yaw
        self.last_seen = {}

    def feed(self, key, now):
        if key in "wasdqe" and len(key) == 1:
            self.last_seen[key] = now
        elif key in (" ", "x", "\x1b", "1", "2", "3", "4"):
            self.last_seen.clear()

    def command(self, now):
        def held(key):
            return key in self.last_seen and 0 <= now-self.last_seen[key] < self.timeout
        return (self.speed*(int(held("w"))-int(held("s"))),
                self.speed*(int(held("a"))-int(held("d"))),
                self.yaw*(int(held("q"))-int(held("e"))))
