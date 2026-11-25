"""g1_guardian.writer_check: refuse to start a second actuator writer on one computer.

Runs against a fake ``systemctl`` on PATH that logs every call, so the tests also
prove the check only ever *reads* unit states (it must never stop a writer: on G1D
that drops the arm).
"""

import os
import subprocess
import sys
from pathlib import Path

import pytest

from g1_guardian import writer_check as wc

REPO = Path(__file__).resolve().parents[1]
SRC = REPO / "src"

FAKE_SYSTEMCTL = """#!/bin/sh
echo "$@" >> "$FAKE_SYSTEMCTL_LOG"
if [ "$1" != "show" ]; then echo "unexpected systemctl $1" >&2; exit 99; fi
unit="$4"
if [ -n "$FAKE_SYSTEMCTL_FAIL" ]; then echo "Failed to connect to bus" >&2; exit 1; fi
state=$(grep "^$unit=" "$FAKE_SYSTEMCTL_STATES" | cut -d= -f2)
echo "${state:-inactive}"
"""


@pytest.fixture
def fake_systemctl(tmp_path, monkeypatch):
    bindir = tmp_path / "bin"
    bindir.mkdir()
    exe = bindir / "systemctl"
    exe.write_text(FAKE_SYSTEMCTL)
    exe.chmod(0o755)
    states = tmp_path / "states"
    states.write_text("")
    log = tmp_path / "log"
    monkeypatch.setenv("PATH", f"{bindir}{os.pathsep}{os.environ['PATH']}")
    monkeypatch.setenv("FAKE_SYSTEMCTL_STATES", str(states))
    monkeypatch.setenv("FAKE_SYSTEMCTL_LOG", str(log))
    monkeypatch.delenv("FAKE_SYSTEMCTL_FAIL", raising=False)

    class Fake:
        def set(self, **units):
            states.write_text("".join(f"{u.replace('_', '-')}.service={s}\n" for u, s in units.items()))

        def calls(self):
            return log.read_text().splitlines() if log.exists() else []

    return Fake()


@pytest.fixture
def units_file(tmp_path):
    p = tmp_path / "actuator-writer-units"
    p.write_text("# writers on this computer\ng1-guardian.service\nfatigue.service  # Unitree side\n\n"
                 "vision-arm-control-refactor.service\n")
    return str(p)


def _run(*args):
    return wc.main(list(args))


def test_parse_units_skips_comments_blanks_and_duplicates():
    assert wc.parse_units("# c\n\na.service\n b.service # x\na.service\nc@1.service\nd.scope\n") == [
        "a.service", "b.service", "c@1.service", "d.scope"]
    for bad in ("a", "a.service; reboot", "a b.service", "../a.service", "a.timer"):
        with pytest.raises(wc.CheckError):
            wc.parse_units(bad)


def test_running_states_count_and_stopped_states_do_not():
    states = {"a.service": "active", "b.service": "activating", "c.service": "deactivating",
              "d.service": "reloading", "e.service": "inactive", "f.service": "failed", "g.service": ""}
    busy = wc.running_writers("self.service", list(states) + ["self.service"], states.get)
    assert busy == [("a.service", "active"), ("b.service", "activating"),
                    ("c.service", "deactivating"), ("d.service", "reloading")]


def test_starts_when_no_other_writer_runs(fake_systemctl, units_file, capsys):
    fake_systemctl.set(g1_guardian="active", fatigue="inactive")  # self being "activating/active" is fine
    assert _run("--self", "g1-guardian.service", "--units-file", units_file) == 0
    assert "may start" in capsys.readouterr().err


def test_refuses_when_another_writer_runs_and_never_stops_it(fake_systemctl, units_file, capsys):
    fake_systemctl.set(fatigue="active", vision_arm_control_refactor="deactivating")
    assert _run("--self", "g1-guardian.service", "--units-file", units_file) == 1
    err = capsys.readouterr().err
    assert "fatigue.service (active)" in err and "vision-arm-control-refactor.service (deactivating)" in err
    assert "Do not stop the running one" in err
    calls = fake_systemctl.calls()
    assert calls and all(c.startswith("show --property=ActiveState --value ") for c in calls), calls
    assert not any("g1-guardian.service" in c for c in calls), "自己的状态不用查"


def test_extra_units_on_the_command_line_are_checked(fake_systemctl, units_file):
    fake_systemctl.set(stage_b_executor="active")
    assert _run("--self", "g1-guardian.service", "--units-file", units_file, "stage-b-executor.service") == 1
    assert _run("--self", "g1-guardian.service", "g1-guardian.service", "other.service") == 0


@pytest.mark.parametrize("args, needle", [
    (["--self", "g1-guardian.service", "--units-file", "/nonexistent/units"], "cannot read"),
    (["--self", "g1-guardian.service"], "no writer units"),
    (["--self", "not a unit", "a.service"], "--self is not a unit name"),
    (["--self", "g1-guardian.service", "other.service"], "not in the writer list"),
])
def test_bad_configuration_fails_closed(fake_systemctl, args, needle, capsys):
    assert _run(*args) == 2
    assert needle in capsys.readouterr().err
    assert fake_systemctl.calls() == []


def test_systemctl_failure_fails_closed(fake_systemctl, units_file, monkeypatch, capsys):
    monkeypatch.setenv("FAKE_SYSTEMCTL_FAIL", "1")
    assert _run("--self", "g1-guardian.service", "--units-file", units_file) == 2
    assert "exited 1" in capsys.readouterr().err


def test_missing_systemctl_fails_closed(units_file, monkeypatch, tmp_path, capsys):
    monkeypatch.setenv("PATH", str(tmp_path / "empty"))
    assert _run("--self", "g1-guardian.service", "--units-file", units_file) == 2
    assert "systemctl show" in capsys.readouterr().err


def test_runs_as_a_module_like_execstartpre(fake_systemctl, units_file):
    env = dict(os.environ, PYTHONPATH=str(SRC))
    base = [sys.executable, "-m", "g1_guardian.writer_check", "--self", "g1-guardian.service",
            "--units-file", units_file]
    assert subprocess.run(base, env=env, capture_output=True).returncode == 0
    fake_systemctl.set(fatigue="active")
    res = subprocess.run(base, env=env, capture_output=True, text=True)
    assert res.returncode == 1 and "fatigue.service" in res.stderr


def test_shipped_unit_and_example_list_agree():
    unit = (REPO / "deploy" / "g1-guardian.service").read_text()
    pre = [ln for ln in unit.replace("\\\n", " ").splitlines() if ln.startswith("ExecStartPre=")]
    assert len(pre) == 1
    assert "-m g1_guardian.writer_check" in pre[0]
    assert "--self g1-guardian.service" in pre[0] and "--units-file /etc/g1/actuator-writer-units" in pre[0]
    assert "Conflicts=" not in unit, "Conflicts= 会停掉另一个写入者（G1D 掉臂）"
    example = wc.parse_units((REPO / "deploy" / "actuator-writer-units.example").read_text())
    assert "g1-guardian.service" in example


def test_writer_check_uses_only_the_standard_library():
    """可以原样复制到 refactor：不导入 g1_guardian 的其他模块。"""
    import ast

    tree = ast.parse(Path(wc.__file__).read_text())
    mods = {a.name.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
    mods |= {n.module.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and n.module}
    assert mods <= {"__future__", "argparse", "re", "subprocess", "sys", "typing"}, mods
