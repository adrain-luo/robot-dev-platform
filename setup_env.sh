#!/usr/bin/env bash
# 在 ~/abyss/.venv 建独立 Python 3.10（不读 ~/.local，不指向其它工程）。
set -euo pipefail
ROOT="${ABYSS_VENV:-$HOME/abyss/.venv}"
REQ="$(cd "$(dirname "$0")" && pwd)/requirements.txt"
PY="${PYTHON:-/usr/bin/python3}"

if [ ! -x "$ROOT/bin/python" ]; then
  "$PY" -m venv --without-pip --prompt abyss "$ROOT"
  curl -fsSL https://bootstrap.pypa.io/get-pip.py | "$ROOT/bin/python"
fi
"$ROOT/bin/python" -m pip install --upgrade pip
"$ROOT/bin/python" -m pip install -r "$REQ"
"$ROOT/bin/python" -m pip uninstall -y unitree_sdk2 >/dev/null 2>&1 || true

act="$ROOT/bin/activate"
grep -q 'PYTHONNOUSERSITE' "$act" || printf '\nexport PYTHONNOUSERSITE=1\n' >> "$act"
if ! grep -q 'unset PYTHONPATH' "$act"; then
  printf '%s\n' 'if [ -n "${PYTHONPATH:-}" ]; then' '    _OLD_VIRTUAL_PYTHONPATH="${PYTHONPATH:-}"' '    unset PYTHONPATH' 'fi' >> "$act"
fi

HERE="$(cd "$(dirname "$0")" && pwd)"
env -u PYTHONPATH PYTHONNOUSERSITE=1 "$ROOT/bin/python" - <<PY
import site, sys
from pathlib import Path
sys.path.insert(0, str(Path("$HERE") / "arm_control"))
import cyclonedds, numpy
from dds.sdk import ensure_sdk
root = ensure_sdk()
from unitree_sdk2py.core.channel import ChannelFactoryInitialize  # noqa: F401
print("ok", sys.executable)
print("python", sys.version.split()[0])
print("user_site", site.ENABLE_USER_SITE)
print("sdk", root)
print("sys.path:")
for p in sys.path:
    print(" ", p)
for name, mod in [("numpy", numpy), ("cyclonedds", cyclonedds)]:
    loc = Path(mod.__file__).resolve()
    print(name, loc)
    assert ".local" not in str(loc)
    assert "opt/ros" not in str(loc)
assert not site.ENABLE_USER_SITE
assert not any("opt/ros" in (p or "") for p in sys.path)
assert "third_party/unitree_sdk2_python" in str(root)
print("unitree_sdk2py.core.channel ok")
PY
echo "activate: source $ROOT/bin/activate"
