#!/usr/bin/env bash
# 建 / 更新 conda 环境 abyss（所有设备统一用它跑本项目），并自检 SDK 能导入。
# 用法：bash setup_env.sh            环境名可用 ABYSS_ENV 改
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ENV_NAME="${ABYSS_ENV:-abyss}"
CONDA="${CONDA_EXE:-$(command -v conda || echo "$HOME/miniconda3/bin/conda")}"
[ -x "$CONDA" ] || { echo "找不到 conda（先装 miniconda3，或设 CONDA_EXE）" >&2; exit 1; }

if "$CONDA" env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
  # 不加 --prune：环境里别的工程装的包不删
  "$CONDA" env update -n "$ENV_NAME" -f "$HERE/environment.yml"
else
  "$CONDA" env create -n "$ENV_NAME" -f "$HERE/environment.yml"
fi
PY="$("$CONDA" run -n "$ENV_NAME" python -c 'import sys; print(sys.executable)')"
# PyPI 上的 unitree_sdk2 缺 b2，必须用 third_party/unitree_sdk2_python
"$PY" -m pip uninstall -y unitree_sdk2 >/dev/null 2>&1 || true

env -u PYTHONPATH PYTHONNOUSERSITE=1 "$PY" - <<PY
import site, sys
from pathlib import Path
sys.path.insert(0, str(Path("$HERE") / "src"))
import cv2, cyclonedds, numpy, yaml, zmq  # noqa: F401
from robot_platform.hw.dds.sdk import ensure_sdk
root = ensure_sdk()
from unitree_sdk2py.core.channel import ChannelFactoryInitialize  # noqa: F401
print("ok", sys.executable)
print("python", sys.version.split()[0])
print("sdk", root)
for name, mod in [("numpy", numpy), ("cyclonedds", cyclonedds)]:
    loc = Path(mod.__file__).resolve()
    print(name, loc)
    assert ".local" not in str(loc)
    assert "opt/ros" not in str(loc)
assert "third_party/unitree_sdk2_python" in str(root)
print("unitree_sdk2py.core.channel ok")
PY
echo "使用：conda activate $ENV_NAME"
