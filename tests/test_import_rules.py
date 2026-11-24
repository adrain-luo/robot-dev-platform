"""依赖规则（架构评审 §六 / §十 阶段 1 退出门槛），用 AST 检查源码，CI 和本地 pytest 都会跑。

- 任务代码（tasks/、calib/）：不碰 app.loop / require_loop / hw. / unitree_sdk2py / ArmLoop / ArmView，
  只经运控客户端（租约）动手臂；tasks/ 里连 app 都不出现（App 不再把自己传给任务）。
- 感知（perception/）不导入任务。
- g1_contracts 只用标准库；g1_guardian 只用标准库和（延迟导入的）unitree_sdk2py，绝不导入 robot_platform。

每条规则都有自测：往一段假源码里放违规，检查器必须报出来，免得规则写错了永远“通过”。
"""

import ast
import sys
from pathlib import Path
from typing import Iterable, List

SRC = Path(__file__).resolve().parents[1] / "src"
PLATFORM = SRC / "robot_platform"
STDLIB = set(sys.stdlib_module_names) | {"__future__"}

TASK_FORBIDDEN_MODULES = ("robot_platform.hw", "unitree_sdk2py")
TASK_FORBIDDEN_NAMES = {"ArmLoop", "ArmView"}


def _imports(tree: ast.AST):
    """(模块名, 导入的名字列表)；相对导入跳过（本仓库不用）。"""
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for a in node.names:
                yield a.name, []
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            yield node.module, [a.name for a in node.names]


def _starts(module: str, prefixes: Iterable[str]) -> bool:
    return any(module == p or module.startswith(p + ".") for p in prefixes)


def task_violations(source: str, *, forbid_app: bool) -> List[str]:
    tree = ast.parse(source)
    out = []
    for module, names in _imports(tree):
        if _starts(module, TASK_FORBIDDEN_MODULES):
            out.append(f"imports {module}")
        bad = TASK_FORBIDDEN_NAMES.intersection(names)
        if bad:
            out.append(f"imports {', '.join(sorted(bad))} from {module}")
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute):
            base = node.value
            is_app = (isinstance(base, ast.Name) and base.id == "app") or \
                     (isinstance(base, ast.Attribute) and base.attr == "app")
            if node.attr == "loop" and is_app:
                out.append(f"line {node.lineno}: app.loop")
            if node.attr == "require_loop":
                out.append(f"line {node.lineno}: require_loop")
            if forbid_app and node.attr == "app":
                out.append(f"line {node.lineno}: .app")
        elif isinstance(node, ast.Name):
            if node.id == "require_loop":
                out.append(f"line {node.lineno}: require_loop")
            if forbid_app and node.id == "app":
                out.append(f"line {node.lineno}: app")
        elif isinstance(node, ast.arg) and forbid_app and node.arg == "app":
            out.append(f"line {node.lineno}: parameter app")
        elif isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "getattr":
            if len(node.args) >= 2 and isinstance(node.args[1], ast.Constant) and node.args[1].value == "loop":
                out.append(f"line {node.lineno}: getattr(..., 'loop')")
    return out


def import_violations(source: str, *, allowed_top: Iterable[str]) -> List[str]:
    allowed = STDLIB | set(allowed_top)
    return [f"imports {m}" for m, _ in _imports(ast.parse(source)) if m.split(".")[0] not in allowed]


def forbidden_import_violations(source: str, *, prefixes: Iterable[str]) -> List[str]:
    return [f"imports {m}" for m, _ in _imports(ast.parse(source)) if _starts(m, prefixes)]


def _scan(root: Path, check) -> List[str]:
    found = []
    for path in sorted(root.rglob("*.py")):
        for v in check(path.read_text(encoding="utf-8")):
            found.append(f"{path.relative_to(SRC)}: {v}")
    return found


# ---------------------------------------------------------------- 规则

def test_tasks_never_touch_the_app_the_loop_or_hardware():
    assert _scan(PLATFORM / "tasks", lambda s: task_violations(s, forbid_app=True)) == []


def test_calibration_tasks_use_the_motion_client_not_the_loop_or_hardware():
    assert _scan(PLATFORM / "calib", lambda s: task_violations(s, forbid_app=False)) == []


def test_perception_does_not_import_tasks():
    assert _scan(PLATFORM / "perception",
                 lambda s: forbidden_import_violations(s, prefixes=["robot_platform.tasks"])) == []


def test_contracts_use_only_the_standard_library():
    assert _scan(SRC / "g1_contracts", lambda s: import_violations(s, allowed_top=["g1_contracts"])) == []


def test_guardian_stays_free_of_platform_code():
    assert _scan(SRC / "g1_guardian",
                 lambda s: import_violations(s, allowed_top=["g1_guardian", "unitree_sdk2py"])) == []


# ---------------------------------------------------------------- 检查器自测

BAD_TASK = '''
import unitree_sdk2py.core.channel
from robot_platform.hw.dds.sdk import init_channel
from robot_platform.control.arm_loop import ArmLoop, PermissionErrorMotion
def go(app, x):
    app.loop.set_target(x)
    x.app.loop.hold_arm("right")
    app.require_loop()
    getattr(app, "loop")
'''


def test_checker_catches_each_kind_of_task_violation():
    found = task_violations(BAD_TASK, forbid_app=True)
    for needle in ("imports unitree_sdk2py.core.channel", "imports robot_platform.hw.dds.sdk",
                   "imports ArmLoop", "app.loop", "require_loop", "getattr(..., 'loop')",
                   "parameter app", ".app"):
        assert any(needle in v for v in found), (needle, found)
    # calib/ 允许 self.app（拿状态、配置、运控客户端），但仍不许 app.loop
    calib = task_violations("def f(self):\n    self.app.motion()\n    self.app.loop\n", forbid_app=False)
    assert calib == ["line 3: app.loop"]
    ok = "from robot_platform.control.arm_loop import PermissionErrorMotion\ndef f(motion):\n    motion.lease_arm('right', 't')\n"
    assert task_violations(ok, forbid_app=True) == []


def test_checker_catches_non_stdlib_and_forbidden_imports():
    assert import_violations("import numpy\nimport json\nfrom g1_contracts.motion import Ack\n",
                             allowed_top=["g1_contracts"]) == ["imports numpy"]
    assert import_violations("def f():\n    from robot_platform.core import layout\n",
                             allowed_top=["g1_guardian"]) == ["imports robot_platform.core"]
    assert forbidden_import_violations("from robot_platform.tasks.board_touch import x\nimport robot_platform.geometry\n",
                                       prefixes=["robot_platform.tasks"]) == ["imports robot_platform.tasks.board_touch"]
