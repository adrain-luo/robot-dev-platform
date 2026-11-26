"""Guardian access control, layer 1: the shipped firewall agrees with the config and the unit.

Layer 2 (the config's access lists, enforced in the core and the service) is
tested in test_guardian_core.py and test_guardian_service.py.
"""

import re
import shutil
import subprocess
from pathlib import Path

import pytest

from g1_guardian.config import builtin_config_path, load_config
from g1_guardian.protocol import DEFAULT_PORT

REPO = Path(__file__).resolve().parents[1]
NFT = REPO / "deploy" / "g1-guardian-firewall.nft"
FW_UNIT = REPO / "deploy" / "g1-guardian-firewall.service"
GUARDIAN_UNIT = REPO / "deploy" / "g1-guardian.service"
G1D = load_config(builtin_config_path("g1d"))


def _rules():
    text = NFT.read_text()
    body = text[text.index("chain input {"):]
    return [ln.strip() for ln in body.splitlines()[1:] if ln.strip() and ln.strip() != "}" and not ln.strip().startswith("type ")]


def _exec_start(unit_text):
    line = [ln for ln in unit_text.replace("\\\n", " ").splitlines() if ln.startswith("ExecStart=")]
    assert len(line) == 1
    return line[0].split()


def _arg(argv, name):
    return argv[argv.index(name) + 1]


def test_firewall_lets_in_exactly_the_configs_remote_addresses():
    rules = _rules()
    assert len(rules) == 3, rules
    loopback, box, drop = rules
    assert loopback == f'iif "lo" udp dport {DEFAULT_PORT} accept'
    m = re.fullmatch(r'iifname "(\S+)" ip saddr \{ ([0-9., ]+) \} udp dport (\d+) accept', box)
    assert m, box
    iface, addrs, port = m.group(1), {a.strip() for a in m.group(2).split(",")}, int(m.group(3))
    assert addrs == set(G1D.access.remote), "firewall and config access lists disagree"
    assert port == DEFAULT_PORT
    assert iface == _arg(_exec_start(GUARDIAN_UNIT.read_text()), "--net"), "same wired interface as the guardian's DDS"
    assert drop == f"udp dport {DEFAULT_PORT} counter drop"


def test_firewall_filters_only_the_guardian_port_in_its_own_table():
    text = NFT.read_text()
    assert "policy accept;" in text, "everything else on this computer passes"
    assert text.count("table inet g1_guardian {") == 1
    assert "delete table inet g1_guardian" in text, "reloading replaces the table"
    assert all(f"udp dport {DEFAULT_PORT}" in r for r in _rules())
    assert "flush ruleset" not in text


def test_guardian_unit_binds_a_local_address_on_the_configured_port():
    argv = _exec_start(GUARDIAN_UNIT.read_text())
    assert _arg(argv, "--config") == "g1d"
    assert _arg(argv, "--bind") in G1D.access.local, "the operator's CLI on the body must count as local"
    assert int(_arg(argv, "--port")) == DEFAULT_PORT


def test_firewall_unit_loads_the_rules_and_never_touches_the_guardian():
    unit = FW_UNIT.read_text()
    lines = [ln for ln in unit.splitlines() if ln and not ln.startswith("#")]
    assert "ExecStart=/usr/sbin/nft -f /etc/g1/g1-guardian-firewall.nft" in lines
    assert "ExecStop=/usr/sbin/nft delete table inet g1_guardian" in lines
    assert "Type=oneshot" in lines and "RemainAfterExit=yes" in lines
    mentions = [ln for ln in lines if "g1-guardian.service" in ln]
    assert mentions == ["Before=network-pre.target g1-guardian.service"], "ordering only"
    for key in ("Conflicts=", "PartOf=", "BindsTo=", "Requires=", "PropagatesStopTo="):
        assert not any(ln.startswith(key) for ln in lines), key
    assert "g1-guardian-firewall" not in GUARDIAN_UNIT.read_text(), "the guardian unit does not depend on it"


def _can_check_in_a_namespace():
    if not (shutil.which("unshare") and shutil.which("nft")):
        return False
    probe = subprocess.run(["unshare", "-rn", "true"], capture_output=True)
    return probe.returncode == 0


@pytest.mark.skipif(not _can_check_in_a_namespace(), reason="needs nft and unprivileged user namespaces")
def test_firewall_rules_parse():
    """``nft -c`` (check only) inside a throwaway user + network namespace: nothing is loaded
    into this machine's own firewall."""
    res = subprocess.run(["unshare", "-rn", "nft", "-c", "-f", str(NFT)], capture_output=True, text=True)
    assert res.returncode == 0, res.stderr
