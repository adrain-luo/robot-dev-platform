"""Refuse to start an actuator writer while another one runs on this computer.

Run as ``ExecStartPre=`` in the systemd unit of every program that writes
rt/lowcmd (G1D) or rt/arm_sdk (G1)::

    python3 -m g1_guardian.writer_check --self g1-guardian.service \
        --units-file /etc/g1/actuator-writer-units

The units file lists every writer unit on this computer, one per line
(``#`` starts a comment). The check reads each listed unit's ActiveState
with ``systemctl show`` and exits non-zero if any unit other than ``--self``
is running, so systemd does not start ``--self``.

It only reads unit states and never stops anything. systemd's
``Conflicts=`` is deliberately not used: starting one unit would stop the
other, and on G1D stopping a writer that holds the arm drops the arm.

Scope: one computer, systemd units only. A writer on another computer, or
one started by hand, is caught on the DDS bus instead: the guardian refuses
to engage while it sees foreign frames, and the platform and the refactor
refuse to take over for the same reason. Two units started at the same
instant can both pass this check; the bus checks cover that case too.

Exit status: 0 no other writer is running; 1 another writer is running
(named on stderr); 2 bad arguments, unreadable units file, or systemctl not
usable (fail closed).

Standard library only, with no imports from the rest of g1_guardian, so the
file can be copied as is (vision_arm_control_refactor keeps a copy in
deploy/). Keep the copies identical.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from typing import Callable, List, Optional, Sequence, Tuple

# States in which a unit's process may exist (deactivating: e.g. the guardian
# ramping stiffness down before it exits, still sending frames).
RUNNING_STATES = frozenset({"active", "activating", "reloading", "deactivating", "refreshing"})
UNIT_NAME = re.compile(r"^[A-Za-z0-9:_.@-]+\.(service|scope)$")
SYSTEMCTL_TIMEOUT_S = 5.0

StateFn = Callable[[str], str]


class CheckError(Exception):
    """Configuration or systemctl problem: the check fails closed (exit 2)."""


def parse_units(text: str, source: str = "units file") -> List[str]:
    units: List[str] = []
    for n, raw in enumerate(text.splitlines(), 1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        if not UNIT_NAME.match(line):
            raise CheckError(f"{source}:{n}: not a unit name: {line!r}")
        if line not in units:
            units.append(line)
    return units


def read_units_file(path: str) -> List[str]:
    try:
        with open(path, encoding="utf-8") as f:
            text = f.read()
    except OSError as exc:
        raise CheckError(f"cannot read the writer units file {path}: {exc}. "
                         f"Create it from deploy/actuator-writer-units.example") from None
    return parse_units(text, path)


def systemctl_state(unit: str) -> str:
    """ActiveState of a unit ("inactive" for units that do not exist)."""
    try:
        res = subprocess.run(
            ["systemctl", "show", "--property=ActiveState", "--value", unit],
            capture_output=True, text=True, timeout=SYSTEMCTL_TIMEOUT_S, check=False,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise CheckError(f"systemctl show {unit} failed: {exc}") from None
    if res.returncode != 0:
        raise CheckError(f"systemctl show {unit} exited {res.returncode}: {res.stderr.strip()}")
    return res.stdout.strip()


def running_writers(self_unit: str, units: Sequence[str], state: StateFn = systemctl_state) -> List[Tuple[str, str]]:
    """(unit, state) for every listed unit other than self_unit that may be running."""
    out = []
    for unit in units:
        if unit == self_unit:
            continue
        st = state(unit)
        if st in RUNNING_STATES:
            out.append((unit, st))
    return out


def main(argv: Optional[Sequence[str]] = None, *, state: StateFn = systemctl_state) -> int:
    p = argparse.ArgumentParser(
        prog="python3 -m g1_guardian.writer_check",
        description="Exit non-zero if another actuator-writer unit is running on this computer.")
    p.add_argument("--self", dest="self_unit", required=True, help="this unit's name, e.g. g1-guardian.service")
    p.add_argument("--units-file", help="file listing every writer unit on this computer")
    p.add_argument("units", nargs="*", help="more writer units (added to the file's list)")
    args = p.parse_args(argv)
    try:
        if not UNIT_NAME.match(args.self_unit):
            raise CheckError(f"--self is not a unit name: {args.self_unit!r}")
        units = read_units_file(args.units_file) if args.units_file else []
        for u in parse_units("\n".join(args.units), "command line"):
            if u not in units:
                units.append(u)
        if not units:
            raise CheckError("no writer units given (use --units-file and/or list units)")
        if args.self_unit not in units:
            raise CheckError(f"{args.self_unit} is not in the writer list; add it there so every writer "
                             f"on this computer is listed in one place")
        busy = running_writers(args.self_unit, units, state)
    except CheckError as exc:
        print(f"writer_check: refusing to start {args.self_unit}: {exc}", file=sys.stderr)
        return 2
    if busy:
        names = ", ".join(f"{u} ({st})" for u, st in busy)
        print(f"writer_check: refusing to start {args.self_unit}: another actuator writer is running on this "
              f"computer: {names}. Two writers on one topic fight each other. Do not stop the running one "
              f"while it holds the arm (the G1D arm drops when frames stop): support the arm or put it down "
              f"first.", file=sys.stderr)
        return 1
    print(f"writer_check: no other actuator writer running; {args.self_unit} may start "
          f"(checked {len(units) - 1} other unit(s))", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
