// 11-detect.js —— 识别定位：取图、点位获取、校准、中心点提取
// 由 web/static/js/ 下各文件按文件名顺序拼成 /static/app.js（同一个脚本作用域）。

function renderDetect(det) {
  if (!det) return;
  const left = det.left_xyz || null;
  const right = det.right_xyz || null;
  const center = det.center_xyz || null;
  const key = [
    det.phase || "",
    left && left.length ? left.length + ":" + left[0] + ":" + left[left.length - 1] : "",
    right && right.length ? right.length + ":" + right[0] + ":" + right[right.length - 1] : "",
    center && center.length ? center.join(",") : "",
  ].join("|");
  if (key === window.detectPointKey) return;
  if (window.RobotView && window.RobotView.setDetectPoints
      && window.RobotView.setDetectPoints(left, right, center)) {
    window.detectPointKey = key;
  }
}

async function detectLocalize(path, body, busyText) {
  const note = $("detect-note");
  const btns = ["btn-detect-points", "btn-detect-calibrate", "btn-detect-center"]
    .map((id) => $(id)).filter(Boolean);
  if (note && busyText) note.textContent = busyText;
  btns.forEach((b) => { b.disabled = true; });
  try {
    const d = await post(path, body || {});
    renderDetect(d);
    if (!d.ok) {
      if (note) note.textContent = d.error || "失败";
      flash(d.error || "失败", false);
      return d;
    }
    if (note) note.textContent = d.message || d.phase || "完成";
    flash(d.message || "完成", true);
    return d;
  } catch (e) {
    if (note) note.textContent = String(e);
    flash(String(e), false);
    return null;
  } finally {
    btns.forEach((b) => { b.disabled = false; });
  }
}

if ($("btn-detect-grab")) {
  $("btn-detect-grab").onclick = async () => {
    const note = $("detect-note");
    const btn = $("btn-detect-grab");
    const params = ($("detect-params")?.value || "").trim();
    if (note) note.textContent = params ? `正在取图（${params}）…` : "正在从调试口取 YOLO 图…";
    if (btn) btn.disabled = true;
    try {
      const body = {};
      if (params) body.query = params;
      const d = await post("/api/detect/grab", body);
      if (!d.ok) {
        if (note) note.textContent = d.error || "取图失败";
        flash(d.error || "取图失败", false);
        return;
      }
      const left = d.left || { image_jpeg: d.image_jpeg, mask_jpeg: d.mask_jpeg };
      const right = d.right || {};
      const setEye = (side, eye) => {
        const base = $(`detect-${side}-base`);
        const mask = $(`detect-${side}-mask`);
        if (base && eye && eye.image_jpeg) base.src = "data:image/jpeg;base64," + eye.image_jpeg;
        if (mask && eye && eye.mask_jpeg) mask.src = "data:image/jpeg;base64," + eye.mask_jpeg;
      };
      setEye("left", left);
      setEye("right", right);
      const eyes = $("detect-eyes");
      if (eyes) eyes.hidden = false;
      const parts = [];
      if (d.requested_label) parts.push(`请求 ${d.requested_label}`);
      if (d.display_name || d.label) parts.push(`选中 ${d.display_name || d.label}`);
      if (d.confidence != null) parts.push(`置信度 ${Number(d.confidence).toFixed(3)}`);
      if (d.class_id != null) parts.push(`class ${d.class_id}`);
      if (d.pose_query) parts.push(`query ${d.pose_query}`);
      parts.push("左右目 · 各 50% 叠显");
      if (note) note.textContent = parts.join(" · ");
      const mismatch = d.requested_label && d.label && String(d.requested_label) !== String(d.label);
      flash(mismatch ? `已取左右图，但选中 ${d.label} ≠ 请求 ${d.requested_label}` : "已取左右 YOLO 图", !mismatch);
    } catch (e) {
      if (note) note.textContent = String(e);
      flash(String(e), false);
    } finally {
      if (btn) btn.disabled = false;
    }
  };
}

if ($("btn-detect-points")) {
  $("btn-detect-points").onclick = async () => {
    const params = ($("detect-params")?.value || "").trim();
    const body = {};
    if (params) body.query = params;
    await detectLocalize("/api/detect/points", body, "点位获取中（YOLO 四边形 → 10×10 网格 → SGBM）…");
  };
}

if ($("btn-detect-calibrate")) {
  $("btn-detect-calibrate").onclick = () =>
    detectLocalize("/api/detect/calibrate", {}, "校准中（贴平面）…");
}

if ($("btn-detect-center")) {
  $("btn-detect-center").onclick = () =>
    detectLocalize("/api/detect/center", {}, "提取中心点…");
}
