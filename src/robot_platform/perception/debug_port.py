"""G1D 调试口（YOLO / 烟盒姿态页）：取左右图与掩膜、读调试页里的内外参做对比匹配。"""

from __future__ import annotations

import html
import json
import re
import urllib.request
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from robot_platform.perception.cloud_api import _optical_from_down_deg
from robot_platform.perception.stereo_match import StereoMatcher


_DEBUG_HOSTS = (
    "http://192.168.123.164:18081",
)


def _kmat(src: Dict[str, Any]) -> np.ndarray:
    return np.array(
        [[float(src["fx"]), 0.0, float(src["cx"])],
         [0.0, float(src["fy"]), float(src["cy"])],
         [0.0, 0.0, 1.0]],
        dtype=float,
    )


def _T_from_debug_basis(basis: Dict[str, Any], translation: np.ndarray) -> np.ndarray:
    """调试口的三个轴写在左相机光学系里。躯干是 X 朝前、Y 朝左、Z 朝上。"""
    x_right = np.asarray(basis["x_right_unit_xyz"], dtype=float)
    forward = np.asarray(basis["ground_forward_unit_xyz"], dtype=float)
    down = np.asarray(basis["vertical_down_unit_xyz"], dtype=float)
    cam = np.column_stack([x_right, forward, down])
    torso = np.column_stack([
        np.array([0.0, -1.0, 0.0]),
        np.array([1.0, 0.0, 0.0]),
        np.array([0.0, 0.0, -1.0]),
    ])
    R = torso @ np.linalg.inv(cam)
    u, _, vt = np.linalg.svd(R)
    R = u @ vt
    if np.linalg.det(R) < 0:
        u = u.copy()
        u[:, -1] *= -1
        R = u @ vt
    out = np.eye(4)
    out[:3, :3] = R
    out[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    return out


def _http_body(url: str, timeout: float) -> bytes:
    """调试口在烟盒没认准时用 HTTP 500 退回，正文里仍有标定。"""
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            return resp.read()
    except urllib.error.HTTPError as exc:
        return exc.read()


def _http_text(url: str, timeout: float) -> str:
    return _http_body(url, timeout).decode("utf-8", "replace")


def _http_json(url: str, timeout: float) -> Optional[Dict[str, Any]]:
    try:
        data = json.loads(_http_text(url, timeout))
    except json.JSONDecodeError:
        return None
    return data if isinstance(data, dict) else None


def _fill_debug_calib(data: Dict[str, Any], page: str) -> Dict[str, Any]:
    """姿态接口失败时，从调试页正文补上内参、基线和安装角。"""
    out = dict(data)
    basis_m = re.search(
        r'"x_right_unit_xyz":\s*(\[[^\]]+\])\s*,\s*'
        r'"ground_forward_unit_xyz":\s*(\[[^\]]+\])\s*,\s*'
        r'"vertical_down_unit_xyz":\s*(\[[^\]]+\])',
        page,
    )
    cam = dict(((out.get("g1d_visualization") or {}).get("camera") or {}))
    basis = cam.get("basis") if isinstance(cam.get("basis"), dict) else {}
    if basis_m and any(k not in basis for k in ("x_right_unit_xyz", "ground_forward_unit_xyz", "vertical_down_unit_xyz")):
        basis = {
            "x_right_unit_xyz": json.loads(basis_m.group(1)),
            "ground_forward_unit_xyz": json.loads(basis_m.group(2)),
            "vertical_down_unit_xyz": json.loads(basis_m.group(3)),
        }
        cam["basis"] = basis
        vis = dict(out.get("g1d_visualization") or {})
        vis["camera"] = cam
        out["g1d_visualization"] = vis
    stereo = dict(out.get("stereo") or {})
    if not stereo.get("baseline_mm"):
        base_m = re.search(r'"baseline_mm":\s*([0-9.]+)\s*,\s*"stereo_reprojection_error_px"', page)
        if base_m:
            stereo["baseline_mm"] = float(base_m.group(1))
            out["stereo"] = stereo
    return out


def _debug_calib_ready(data: Dict[str, Any]) -> bool:
    left = data.get("intrinsics_assumption") or {}
    right = data.get("intrinsics_assumption_right") or {}
    if any(left.get(k) is None for k in ("fx", "fy", "cx", "cy")) or not left.get("dist_coeffs"):
        return False
    if any(right.get(k) is None for k in ("fx", "fy", "cx", "cy")) or not right.get("dist_coeffs"):
        return False
    try:
        if float((data.get("stereo") or {}).get("baseline_mm") or 0.0) < 10.0:
            return False
    except (TypeError, ValueError):
        return False
    basis = (((data.get("g1d_visualization") or {}).get("camera") or {}).get("basis") or {})
    return all(k in basis for k in ("x_right_unit_xyz", "ground_forward_unit_xyz", "vertical_down_unit_xyz"))


class DebugPortMixin:
    def _debug_base(self, preferred: str = "") -> str:
        """只解析主机，不请求 /debug（空参 /debug 会跑一次自动类别，冲掉 label）。"""
        hosts: List[str] = []
        raw = (preferred or "").strip().rstrip("/")
        if raw:
            hosts.append(raw.split("/debug")[0].split("/pose")[0])
        for host in _DEBUG_HOSTS:
            if host not in hosts:
                hosts.append(host)
        if not hosts:
            raise RuntimeError("没有配置调试口地址")
        return hosts[0]

    def _parse_pose_extra(self, body: Dict[str, Any]) -> Dict[str, str]:
        """从前端 body 解析要传给 /pose 的 query。"""
        from urllib.parse import parse_qsl, urlsplit

        extra: Dict[str, str] = {}
        raw_params = body.get("params")
        if isinstance(raw_params, dict):
            for k, v in raw_params.items():
                if v is None:
                    continue
                extra[str(k)] = str(v)
        qtext = str(body.get("query") or body.get("params_text") or "").strip()
        if qtext.startswith("?"):
            qtext = qtext[1:]
        if qtext.startswith("/pose?"):
            qtext = qtext[len("/pose?") :]
        if qtext.startswith("http://") or qtext.startswith("https://"):
            qtext = urlsplit(qtext).query
        if not qtext:
            return extra
        if qtext.startswith("{"):
            try:
                obj = json.loads(qtext)
            except json.JSONDecodeError as exc:
                raise ValueError(f"参数 JSON 无效：{exc}") from exc
            if not isinstance(obj, dict):
                raise ValueError("params JSON 必须是对象")
            for k, v in obj.items():
                if v is None:
                    continue
                extra[str(k)] = str(v)
            return extra
        for k, v in parse_qsl(qtext, keep_blank_values=True):
            extra[str(k)] = str(v)
        return extra

    def _pose_url_with_params(self, base: str, body: Dict[str, Any]) -> Tuple[str, str]:
        """只拼用户参数 + 新 t，不合并调试页旧 query（避免空参 /debug 的自动结果）。"""
        from urllib.parse import urlencode, urlsplit, urlunsplit

        extra = self._parse_pose_extra(body)
        query = dict(extra)
        query["t"] = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        # 用户若写了 t 则保留用户的
        if "t" in extra:
            query["t"] = extra["t"]
        new_query = urlencode(list(query.items()))
        base_parts = urlsplit(base)
        pose_url = urlunsplit((base_parts.scheme, base_parts.netloc, "/pose", new_query, ""))
        return pose_url, new_query

    def _grab_detect(self, body: Dict[str, Any]) -> Dict[str, Any]:
        """从调试口取左右目原图 + YOLO 掩膜标注图，供前端半透明叠显。"""
        import base64
        import time as _time

        try:
            base = self._debug_base(str(body.get("url") or ""))
        except RuntimeError as exc:
            return {"ok": False, "error": str(exc)}
        overlay = str(body.get("overlay") or "points").strip().lower()
        if overlay not in ("points", "candidates"):
            overlay = "points"
        ov_suffix = "points.jpg" if overlay == "points" else "candidates.jpg"
        stamp = _time.time_ns()
        pose: Dict[str, Any] = {}
        pose_query = ""
        try:
            pose_url, pose_query = self._pose_url_with_params(base, body)
        except ValueError as exc:
            return {"ok": False, "error": str(exc)}
        try:
            # 只打 /pose?label=...，不要先打空参 /debug
            got = _http_json(pose_url, 25)
            if isinstance(got, dict):
                pose = got
        except Exception as exc:
            return {
                "ok": False,
                "error": f"调试口 /pose 失败：{exc.__class__.__name__}: {exc}",
                "pose_url": pose_url,
                "pose_query": pose_query,
            }

        def _explain_non_jpeg(data: bytes, tried: str) -> str:
            text = (data or b"")[:400].decode("utf-8", errors="replace").strip()
            if text.startswith("{"):
                try:
                    obj = json.loads(text)
                    err = obj.get("error") or obj.get("error_code") or text
                    return f"{tried} → {err}"
                except Exception:
                    pass
            if not data:
                return f"{tried} 空响应"
            return f"{tried} 不是 JPEG（{text[:120] or data[:12]!r}）"

        def _jpeg(name: str) -> Tuple[Optional[bytes], str]:
            """叠图优先 /latest；没有则走 /debug/<name>?参数（会按参数重跑一帧）。

            注意：没有 /debug/latest/... 这种路径。
            """
            q = pose_query or f"t={stamp}"
            if "t=" not in q:
                q = f"{q}&t={stamp}"
            urls = [
                f"{base}/latest/{name}?t={stamp}",
                f"{base}/debug/{name}?{q}",
            ]
            errors: List[str] = []
            for url in urls:
                try:
                    data = _http_body(url, 20)
                except Exception as exc:
                    errors.append(f"{url}: {exc.__class__.__name__}: {exc}")
                    continue
                if data and data[:2] == b"\xff\xd8":
                    return data, ""
                errors.append(_explain_non_jpeg(data or b"", url.replace(base, "")))
            return None, "；".join(errors)

        left_raw, err = _jpeg("left_input.jpg")
        if left_raw is None:
            return {"ok": False, "error": f"左目原图失败：{err}", "pose_query": pose_query}
        left_mask, err = _jpeg(f"left_{ov_suffix}")
        if left_mask is None:
            return {"ok": False, "error": f"左目掩膜图失败：{err}", "pose_query": pose_query}
        right_raw, right_raw_err = _jpeg("right_input.jpg")
        right_mask, right_mask_err = _jpeg(f"right_{ov_suffix}")
        if right_raw is None or right_mask is None:
            return {
                "ok": False,
                "error": "右目图失败：" + "；".join(x for x in (right_raw_err, right_mask_err) if x),
                "pose_query": pose_query,
            }

        def _b64(data: bytes) -> str:
            return base64.b64encode(data).decode("ascii")

        yolo = ((pose.get("g1d_visualization") or {}).get("yolo") or {}) if pose else {}
        requested = pose.get("requested_yolo_label")
        label = pose.get("selected_yolo_label") or yolo.get("label")
        name = yolo.get("display_name") or label
        conf = pose.get("selected_yolo_confidence")
        if conf is None:
            conf = yolo.get("confidence")
        note = "左右目各：原图 + YOLO 掩膜，前端 50% 叠显。"
        if requested not in (None, "") and label not in (None, "") and str(requested) != str(label):
            note += f" 请求 label={requested}，实际选中 {label}。"
        if pose.get("ok") is False and pose.get("error"):
            note += f" 姿态接口：{pose.get('error')}。"
        left = {"image_jpeg": _b64(left_raw), "mask_jpeg": _b64(left_mask)}
        right = {"image_jpeg": _b64(right_raw), "mask_jpeg": _b64(right_mask)}
        return {
            "ok": True,
            "host": base,
            "pose_url": pose_url,
            "pose_query": pose_query,
            "overlay": overlay,
            "requested_label": requested,
            "label": label,
            "display_name": name,
            "confidence": None if conf is None else float(conf),
            "class_id": pose.get("selected_yolo_class_id") if pose else yolo.get("class_id"),
            "left": left,
            "right": right,
            # 兼容旧前端字段（等同左目）
            "image_jpeg": left["image_jpeg"],
            "mask_jpeg": left["mask_jpeg"],
            "pose_ok": bool(pose.get("ok")) if pose else None,
            "note": note,
        }

    def _fetch_debug_pose(self, preferred: str) -> Tuple[Dict[str, Any], str]:
        hosts: List[str] = []
        raw = preferred.strip().rstrip("/")
        if raw:
            hosts.append(raw.split("/debug")[0].split("/pose")[0])
        for host in _DEBUG_HOSTS:
            if host not in hosts:
                hosts.append(host)
        errors: List[str] = []
        for base in hosts:
            try:
                page = _http_text(base + "/debug", 4)
                found = re.search(r'href="(/pose\?[^"]+)"', page)
                data: Dict[str, Any] = {}
                pose_error = ""
                if found:
                    url = base + html.unescape(found.group(1))
                    pose = _http_json(url, 8)
                    if isinstance(pose, dict):
                        data = pose
                        pose_error = str(pose.get("error") or "")
                data = _fill_debug_calib(data, page)
                if _debug_calib_ready(data):
                    if pose_error:
                        data["_pose_error"] = pose_error
                    return data, base
                errors.append(f"{base} {pose_error or '调试页里没有内参、基线或安装角'}")
            except Exception as exc:
                detail = getattr(exc, "reason", None) or exc
                errors.append(f"{base} {exc.__class__.__name__}: {detail}")
        raise RuntimeError("调试口连不上：" + "；".join(errors))

    def _debug_calib(self, data: Dict[str, Any], hand_T: Optional[np.ndarray], size: Tuple[int, int]) -> Dict[str, Any]:
        if hand_T is None:
            raise ValueError("调试口没有相机在躯干里的位置，仍要先调用已保存的手眼结果，用它的平移")
        left = data.get("intrinsics_assumption") or {}
        right = data.get("intrinsics_assumption_right") or {}
        stereo = data.get("stereo") or {}
        cam = ((data.get("g1d_visualization") or {}).get("camera") or {})
        basis = cam.get("basis") or {}
        for eye, src in (("左", left), ("右", right)):
            if any(src.get(k) is None for k in ("fx", "fy", "cx", "cy")) or not src.get("dist_coeffs"):
                raise ValueError(f"调试口没有{eye}目内参")
        baseline = float(stereo.get("baseline_mm") or 0.0)
        if baseline < 10.0:
            raise ValueError("调试口没有双目基线")
        needed = ("x_right_unit_xyz", "ground_forward_unit_xyz", "vertical_down_unit_xyz")
        if any(k not in basis for k in needed):
            raise ValueError("调试口没有相机相对躯干的朝向")
        K1 = _kmat(left)
        K2 = _kmat(right)
        # 已保存的双目标定里，右相机在左相机的负 X 方向。调试口只给基线长度。
        stereo_T = np.array([-baseline, 0.0, 0.0], dtype=float)
        key = (
            round(float(left["fx"]), 3), round(float(right["fx"]), 3),
            round(baseline, 2), int(size[0]), int(size[1]),
        )
        if self._debug_matcher is None or self._debug_key != key:
            self._debug_matcher = StereoMatcher.from_matrices(
                K1, left["dist_coeffs"], K2, right["dist_coeffs"],
                np.eye(3), stereo_T, size,
            )
            self._debug_key = key
        T = _T_from_debug_basis(basis, np.asarray(hand_T, float)[:3, 3])
        ours = _optical_from_down_deg(hand_T)
        theirs = _optical_from_down_deg(T)
        stated = cam.get("camera_to_vertical_deg")
        stated_txt = "" if stated is None else f"，页面写的是 {float(stated):.1f}°"
        note = (
            f"调试口内参 fx {float(left['fx']):.1f}/{float(right['fx']):.1f}，基线 {baseline:.2f} mm。"
            f"光轴离竖直 {theirs:.1f}°{stated_txt}，手眼是 {ours:.1f}°。"
            "橙色是这套，原色是手眼。平移仍用手眼的，调试口没给相机位置。"
            "它也没给左右相机之间的旋转，右相机按平行、基线沿负 X。"
        )
        pose_error = str(data.get("_pose_error") or data.get("error") or "")
        if pose_error:
            note += f" 烟盒姿态接口返回：{pose_error}。标定仍从调试页读出。"
        return {"matcher": self._debug_matcher, "T": T, "note": note}
