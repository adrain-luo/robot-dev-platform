"""手眼求解接口：估内参 / 求外参、命名保存、调用（绑定配套内参）、删除。"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict

from robot_platform.calib.handeye import results as he_results, session as he_session
from robot_platform.calib.handeye import color_intrinsics as he_color_intr
from robot_platform.calib.handeye.config import load_config, save_config
from robot_platform.calib.handeye.detection import parse_board_size
from robot_platform.calib.handeye.solve_core import load_intrinsics, object_points_m, solve_eye, SolveOptions, stereo_cross_check
from robot_platform.control.urdf_fk import URDFRobotModel
from robot_platform.core import instance as instances
from robot_platform.perception import rgbd


class SolveMixin:
    def _solve(self, body: Dict[str, Any]) -> Dict[str, Any]:
        cfg = load_config()
        for key in ("board_size", "eye", "session"):
            if body.get(key):
                cfg[key] = body[key]
        if body.get("square_size_mm") not in (None, ""):
            cfg["square_size_mm"] = float(body["square_size_mm"])
        save_config(cfg)
        name = str(cfg.get("session") or "")
        if not name:
            raise ValueError("先选或新建一个采集会话")
        data_dir = he_session.session_dir(name)
        if not (data_dir / "joints").is_dir():
            raise FileNotFoundError(f"{name} 里没有 joints/")
        square = cfg.get("square_size_mm")
        if square is None or float(square) <= 0:
            raise ValueError("填棋盘格子边长（毫米）")
        board = parse_board_size(str(cfg.get("board_size") or "11x8"))
        eye = str(cfg.get("eye") or "both")
        log: list = []
        cam_info = he_session.camera_info(data_dir)
        if cam_info is not None:
            # RGB-D：只有彩色一路，不做双目标定；内参用相机卡片选定的（未选则出厂 camera_info）
            n_img = len(list((data_dir / "left").glob("*.jpg")))
            intr_name = str(cfg.get("cloud_color_intrinsics") or "")
            if intr_name:
                cam_info = he_color_intr.apply_to_info(cam_info, intr_name)["info"]
            fit = rgbd.write_session_intrinsics(data_dir, cam_info, float(square), n_img, source=intr_name)
            where = f"自标「{intr_name}」" if intr_name else "出厂"
            log.append(f"[内参] RGB-D {where}内参 fx={fit['left_fx']:.1f} fy={fit['left_fy']:.1f}，{n_img} 张彩色图")
            eye = "left"
        else:
            from robot_platform.calib.handeye.intrinsics import calibrate_session
            fit = calibrate_session(data_dir, board, float(square), log=log)
        yaml_path = Path(fit["yaml"])
        cfg["intrinsics"] = instances.active().to_stored(yaml_path)
        save_config(cfg)
        urdf = self.app.settings.urdf
        meta = he_session.load_meta(data_dir)
        base = str(meta.get("base_link") or self.app.settings.base_link or he_session.base_link())
        tip = str(meta.get("tip_link") or he_session.tip_link(meta.get("arm")))
        opt = SolveOptions(
            base_link=base,
            tip_link=tip,
            square_size_mm=float(square),
            intrinsics=str(yaml_path),
        )
        model = URDFRobotModel(str(urdf), opt.base_link, opt.tip_link)
        objp = object_points_m(board, opt.square_size_mm)
        eyes = ["left", "right"] if eye == "both" else [eye]
        results = {}
        for one in eyes:
            K, dist = load_intrinsics(yaml_path, one)
            res = solve_eye(one, data_dir, K, dist, objp, board, model, opt, log=log)
            if res is None:
                continue
            results[one] = res
            (data_dir / f"handeye_result_{one}.json").write_text(
                json.dumps(res, indent=2, ensure_ascii=False), encoding="utf-8"
            )
        if not results:
            raise RuntimeError("没有一眼算出结果。看采集是否够、棋盘是否检出。\n" + "\n".join(log[-12:]))
        cross = stereo_cross_check(results)
        combined = dict(results)
        if cross:
            combined.update(cross)
        if cross or cam_info is not None:
            (data_dir / "handeye_result.json").write_text(
                json.dumps(combined, indent=2, ensure_ascii=False), encoding="utf-8"
            )
        stt = self.status()
        stt["intrinsics_fit"] = fit
        stt["result"] = combined
        stt["log"] = log[-16:]
        return stt

    def _save_named(self, body: Dict[str, Any]) -> Dict[str, Any]:
        cfg = load_config()
        name = str(body.get("name") or "").strip()
        session = str(body.get("session") or cfg.get("session") or "")
        if not session:
            raise ValueError("先选一次已经求解的采集会话")
        data_dir = he_session.session_dir(session)
        doc = he_results.save_named(name, data_dir)
        self._bind_handeye_doc(cfg, doc)
        save_config(cfg)
        self._matcher = None
        self._matcher_key = ""
        stt = self.status()
        stt["intrinsics_fit"] = doc.get("intrinsics_fit")
        stt["result"] = doc.get("result")
        stt["active_result"] = doc["name"]
        return stt

    @staticmethod
    def _bind_handeye_doc(cfg: Dict[str, Any], doc: Dict[str, Any]) -> None:
        """手眼结果与其自带双目 yaml 成套写入配置，不允许拆开。"""
        cfg["active_result"] = doc["name"]
        if doc.get("session"):
            cfg["session"] = doc["session"]
        inst = instances.active()
        yaml_path = inst.from_stored(doc.get("intrinsics_yaml"))
        if yaml_path is None or not yaml_path.is_file():
            yaml_path = he_results.results_dir() / f"{doc['name']}.yaml"
        if yaml_path.is_file():
            cfg["intrinsics"] = inst.to_stored(yaml_path)

    def _load_named(self, body: Dict[str, Any]) -> Dict[str, Any]:
        doc = he_results.load_saved(str(body.get("name") or ""))
        cfg = load_config()
        self._bind_handeye_doc(cfg, doc)
        save_config(cfg)
        self._matcher = None
        self._matcher_key = ""
        stt = self.status()
        stt["intrinsics_fit"] = doc.get("intrinsics_fit")
        stt["result"] = doc.get("result")
        stt["active_result"] = doc["name"]
        return stt

    def _delete_named(self, body: Dict[str, Any]) -> Dict[str, Any]:
        name = str(body.get("name") or "")
        he_results.delete_saved(name)
        cfg = load_config()
        if cfg.get("active_result") == name:
            cfg["active_result"] = ""
            save_config(cfg)
        return self.status()
