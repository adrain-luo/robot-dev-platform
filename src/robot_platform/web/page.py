"""首页：index.html 里几个下拉框的选项在服务端先填好（姿态集、测量会话、对比记录、手眼会话、空间点）。"""

from __future__ import annotations

import re
import traceback
from html import escape

from robot_platform.calib.handeye import session as he_session
from robot_platform.calib.payload import identify as iden
from robot_platform.calib.payload import poses as posesets
from robot_platform.calib.payload import validate as vali
from robot_platform.paths import STATIC_DIR
from robot_platform.tasks.spatial_points import list_saved as list_spatial_saved


def _opt(value: str, label: str) -> str:
    return f'<option value="{escape(value, quote=True)}">{escape(label)}</option>'


def _inject_options(html: str, select_id: str, options: list) -> str:
    pattern = rf'(<select\b[^>]*\bid="{re.escape(select_id)}"[^>]*>)\s*(</select>)'
    inner = "".join(options)
    new, _ = re.subn(pattern, rf"\1{inner}\2", html, count=1)
    return new


def page_html() -> str:
    html = (STATIC_DIR / "index.html").read_text(encoding="utf-8")
    try:
        pose_opts = [_opt("", "选择姿态集…")]
        for item in posesets.list_sets():
            side = "左" if item.get("arm") == "left" else "右"
            cond = f" · cond {item['cond']}" if item.get("cond") is not None else ""
            pose_opts.append(_opt(item["name"], f"{item['name']} · {side} · {item['n']}点{cond}"))
        html = _inject_options(html, "poseset-select", pose_opts)
        sess_opts = [_opt("", "选择测量会话再求解…")]
        sessions = iden.list_sessions()
        if not sessions:
            sess_opts = [_opt("", "还没有测量会话（先点测量当前集）")]
        for item in sessions:
            side = "左" if item.get("arm") == "left" else "右"
            bi = "双向" if item.get("bidirectional") else "单向"
            sess_opts.append(_opt(item["name"], f"{item['name']} · {side} {item.get('poseset') or ''} {item.get('n')}样 {bi}"))
        html = _inject_options(html, "id-session", sess_opts)
        vals = vali.list_results()
        val_opts = [_opt("", "打开已保存的对比…" if vals else "还没有对比记录")]
        for item in vals:
            side = "左" if item.get("arm") == "left" else "右"
            val_opts.append(_opt(item["name"], f"data/validate/{item['name']}.json · {side} {item.get('poseset') or ''}"))
        html = _inject_options(html, "val-session", val_opts)
        he_opts = [_opt("", "选择采集会话…")]
        sessions = he_session.list_sessions()
        if not sessions:
            he_opts = [_opt("", "还没有采集会话")]
        for item in sessions:
            side = "左" if item.get("arm") == "left" else "右"
            ps = f" · {item['poseset']}" if item.get("poseset") else ""
            he_opts.append(_opt(item["name"], f"{item['name']} · {side} {item.get('n', 0)}张{ps}"))
        html = _inject_options(html, "he-session", he_opts)
        pts = list_spatial_saved()
        pt_opts = [_opt("", "选择空间点…" if pts else "还没有空间点")]
        for item in pts:
            side = "左" if item.get("arm") == "left" else "右" if item.get("arm") == "right" else ""
            pt_opts.append(_opt(item["name"], f"{item['name']} · {side} · {item.get('tcp_name') or ''}".rstrip(" ·")))
        html = _inject_options(html, "pt-select", pt_opts)
    except Exception:
        traceback.print_exc()
    return html
