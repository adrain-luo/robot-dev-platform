// 03-body.js —— 躯体：腰（滑条 / 步进 / 到目标 / 停止）与立柱（读数、零位、行程、步进）
// 由 web/static/js/ 下各文件按文件名顺序拼成 /static/app.js（同一个脚本作用域）。

function renderLift(lift) {
  const el = $("lift-now");
  const deltaEl = $("lift-delta");
  if (!el) return;
  if (!lift) {
    el.textContent = "立柱高度 — 无状态";
    if (deltaEl) deltaEl.textContent = "";
    return;
  }
  const calib = lift.calib || {};
  if (lift.available && lift.height_m != null) {
    const live = lift.velocity_mps != null && Math.abs(lift.velocity_mps) > 0.001;
    const parts = [
      `${Number(lift.height_m).toFixed(4)} m`,
      `${Number(lift.height_mm).toFixed(1)} mm`,
    ];
    if (lift.stroke_mm != null) parts.push(`行程 ${Number(lift.stroke_mm).toFixed(1)} mm`);
    if (lift.joints) {
      const mt = lift.joints.LZ_mt_Joint;
      const it = lift.joints.LZ_it_Joint;
      if (mt != null && it != null) {
        parts.push(`LZ ${(Number(mt) * 1000).toFixed(0)}+${(Number(it) * 1000).toFixed(0)} mm`);
      }
    }
    if (lift.velocity_mps != null) {
      parts.push(`vz ${Number(lift.velocity_mms).toFixed(1)} mm/s${live ? " · 在动" : ""}`);
    }
    if (lift.age_s != null && Number.isFinite(lift.age_s)) {
      parts.push(`龄 ${Number(lift.age_s).toFixed(2)} s`);
    }
    el.textContent = "立柱高度 " + parts.join(" · ");
  } else {
    el.textContent = `立柱高度 — 等待 ${lift.topic || "rt/hispeed_state"}`;
  }
  if (deltaEl) {
    const zero = calib.height_zero_m;
    const zeroTxt = zero != null
      ? `当前零位 ${Number(zero).toFixed(4)} m（${Number(calib.height_zero_mm).toFixed(1)} mm）`
      : "尚未标定零位";
    deltaEl.textContent = `${zeroTxt}。降到机械最低后点「记为最低点」写入并应用到三维。`;
  }
}

if ($("btn-lift-zero")) {
  $("btn-lift-zero").onclick = async () => {
    try {
      const d = await post("/api/lift/set-zero", {});
      if (!d.ok) { flash(d.error || "记最低点失败", false); return; }
      if (d.lift) {
        window.lastLift = d.lift;
        renderLift(d.lift);
      }
      flash(d.message || "已记为最低点", true);
      poll();
    } catch (e) {
      flash(String(e), false);
    }
  };
}

function waistDeg(rad) {
  return Number.isFinite(rad) ? (rad * 180 / Math.PI).toFixed(2) + "°" : "—";
}

let liftSliderDragging = false;

let waistSliderDragging = false;

function renderWaist(data) {
  const note = $("waist-now");
  if (!note) return;
  const w = data.waist || {};
  const tgt = (data.ctrl && data.ctrl.waist_target_rad) || null;
  const axes = (w.joints || []).map((j) => ({ role: j.label.replace("waist_", ""), motor: j.motor, q: j.q_rad }));
  const axis = (role) => axes.find((a) => a.role === role) || {};
  const yawRad = axis("yaw").q;
  const pitchRad = axis("pitch").q;
  if (!axes.length) { note.textContent = "腰 —"; renderLiftCtrl(data.lift); return; }
  let text = "腰实测 " + axes.map((a) => `${WAIST_ROLE[a.role] || a.role} ${waistDeg(a.q)}（电机${a.motor}）`).join(" · ");
  if (tgt) text += "　指令 " + axes.map((a) => `${WAIST_ROLE[a.role] || a.role} ${waistDeg(Number(tgt[String(a.motor)]))}`).join(" · ");
  note.textContent = text;
  if (!waistSliderDragging && !window.waistTargetDirty) {
    const yawDeg = Number.isFinite(yawRad) ? yawRad * 180 / Math.PI : null;
    const pitchDeg = Number.isFinite(pitchRad) ? pitchRad * 180 / Math.PI : null;
    if (yawDeg != null) {
      if ($("waist-yaw") && document.activeElement !== $("waist-yaw")) $("waist-yaw").value = yawDeg.toFixed(1);
      if ($("waist-yaw-slider")) $("waist-yaw-slider").value = String(yawDeg);
    }
    if (pitchDeg != null) {
      if ($("waist-pitch") && document.activeElement !== $("waist-pitch")) $("waist-pitch").value = pitchDeg.toFixed(1);
      if ($("waist-pitch-slider")) $("waist-pitch-slider").value = String(pitchDeg);
    }
  }
  renderLiftCtrl(data.lift);
}

function renderLiftCtrl(lift) {
  const note = $("lift-ctrl-now");
  if (!note) return;
  if (!lift || !lift.available || lift.height_m == null) {
    note.textContent = "立柱 — 等待高度";
    return;
  }
  const ctrl = lift.ctrl || {};
  const stroke = lift.stroke_mm != null ? Number(lift.stroke_mm) : null;
  const maxMm = ctrl.stroke_max_mm != null ? Number(ctrl.stroke_max_mm) : 420;
  const slider = $("lift-slider");
  if (slider) {
    slider.max = String(Math.round(maxMm));
    if (!liftSliderDragging && stroke != null) {
      slider.value = String(Math.round(Math.max(0, Math.min(maxMm, stroke))));
    }
  }
  const targetEl = $("lift-target");
  if (targetEl && !liftSliderDragging && document.activeElement !== targetEl && stroke != null) {
    // 不覆盖用户正在编辑的目标；条形拖动时也不抢
    if (!window.liftTargetDirty) targetEl.value = String(Math.round(stroke));
  }
  const parts = [
    `行程 ${stroke != null ? stroke.toFixed(1) : "—"} mm`,
    `高度 ${Number(lift.height_m).toFixed(4)} m`,
  ];
  if (ctrl.busy && ctrl.target_m != null) {
    const zero = (lift.calib && lift.calib.height_zero_m != null)
      ? Number(lift.calib.height_zero_m)
      : (ctrl.zero_m != null ? Number(ctrl.zero_m) : null);
    const tStroke = zero != null ? (Number(ctrl.target_m) - zero) * 1000 : null;
    parts.push(`前往 ${tStroke != null ? tStroke.toFixed(1) : "—"} mm`);
  }
  if (ctrl.error) parts.push(ctrl.error);
  note.textContent = "立柱 " + parts.join(" · ");
}

function liftStrokeFromUi() {
  const n = parseFloat($("lift-target")?.value);
  return Number.isFinite(n) ? n : parseFloat($("lift-slider")?.value);
}

function syncLiftTargetFromSlider() {
  const sl = $("lift-slider");
  const tg = $("lift-target");
  if (sl && tg) tg.value = sl.value;
  window.liftTargetDirty = true;
}

if ($("lift-slider")) {
  $("lift-slider").addEventListener("pointerdown", () => { liftSliderDragging = true; });
  $("lift-slider").addEventListener("pointerup", async () => {
    syncLiftTargetFromSlider();
    liftSliderDragging = false;
    const mm = liftStrokeFromUi();
    if (!Number.isFinite(mm)) return;
    try {
      const d = await post("/api/lift/goto", { stroke_mm: mm });
      if (!d.ok) { flash(d.error || "立柱失败", false); return; }
      flash(d.message || `立柱前往 ${mm} mm`, true);
      window.liftTargetDirty = false;
      poll();
    } catch (e) { flash(String(e), false); }
  });
  $("lift-slider").addEventListener("input", syncLiftTargetFromSlider);
}

if ($("lift-target")) {
  $("lift-target").addEventListener("input", () => {
    window.liftTargetDirty = true;
    const sl = $("lift-slider");
    const v = parseFloat($("lift-target").value);
    if (sl && Number.isFinite(v)) sl.value = String(Math.round(v));
  });
}

function syncWaistTargetFromSliders() {
  if ($("waist-yaw-slider") && $("waist-yaw")) $("waist-yaw").value = $("waist-yaw-slider").value;
  if ($("waist-pitch-slider") && $("waist-pitch")) $("waist-pitch").value = $("waist-pitch-slider").value;
  window.waistTargetDirty = true;
}

function syncWaistSlidersFromInputs() {
  const y = parseFloat($("waist-yaw")?.value);
  const p = parseFloat($("waist-pitch")?.value);
  if ($("waist-yaw-slider") && Number.isFinite(y)) $("waist-yaw-slider").value = String(y);
  if ($("waist-pitch-slider") && Number.isFinite(p)) $("waist-pitch-slider").value = String(p);
  window.waistTargetDirty = true;
}

async function waistGotoFromUi(confirmGo) {
  const yaw = parseFloat($("waist-yaw")?.value);
  const pitch = parseFloat($("waist-pitch")?.value);
  if (!Number.isFinite(yaw) || !Number.isFinite(pitch)) { flash("腰目标无效", false); return; }
  if (confirmGo && !confirm(`腰将限速转到旋转 ${yaw}°、俯仰 ${pitch}°。扶住、守急停。继续？`)) return;
  try {
    await post("/api/waist", { yaw_deg: yaw, pitch_deg: pitch });
    flash(`腰前往 旋转 ${yaw}° · 俯仰 ${pitch}°`, true);
    window.waistTargetDirty = false;
    poll();
  } catch (e) { flash(String(e), false); }
}

async function waistStep(axis, sign) {
  const step = parseFloat($("waist-step")?.value);
  if (!Number.isFinite(step) || step <= 0) { flash("腰步长无效", false); return; }
  const body = axis === "yaw"
    ? { delta_yaw_deg: sign * step }
    : { delta_pitch_deg: sign * step };
  try {
    await post("/api/waist/step", body);
    flash(`腰${axis === "yaw" ? "旋转" : "俯仰"}${sign > 0 ? "+" : "−"}${step}°`, true);
    window.waistTargetDirty = false;
    poll();
  } catch (e) { flash(String(e), false); }
}

["waist-yaw-slider", "waist-pitch-slider"].forEach((id) => {
  const el = $(id);
  if (!el) return;
  el.addEventListener("pointerdown", () => { waistSliderDragging = true; });
  el.addEventListener("input", syncWaistTargetFromSliders);
  el.addEventListener("pointerup", async () => {
    syncWaistTargetFromSliders();
    waistSliderDragging = false;
    await waistGotoFromUi(false);
  });
});

["waist-yaw", "waist-pitch"].forEach((id) => {
  const el = $(id);
  if (!el) return;
  el.addEventListener("input", syncWaistSlidersFromInputs);
});

$("btn-waist-fill").onclick = () => {
  const w = window.lastWaist || {};
  if (!Number.isFinite(w.torso_Joint) || !Number.isFinite(w.Yaw_Joint)) {
    flash("还没有腰部读数", false);
    return;
  }
  $("waist-yaw").value = (w.torso_Joint * 180 / Math.PI).toFixed(2);
  $("waist-pitch").value = (w.Yaw_Joint * 180 / Math.PI).toFixed(2);
  syncWaistSlidersFromInputs();
  window.waistTargetDirty = true;
};

if ($("btn-waist-yaw-m")) $("btn-waist-yaw-m").onclick = () => waistStep("yaw", -1);

if ($("btn-waist-yaw-p")) $("btn-waist-yaw-p").onclick = () => waistStep("yaw", 1);

if ($("btn-waist-pitch-m")) $("btn-waist-pitch-m").onclick = () => waistStep("pitch", -1);

if ($("btn-waist-pitch-p")) $("btn-waist-pitch-p").onclick = () => waistStep("pitch", 1);

if ($("btn-waist-stop")) {
  $("btn-waist-stop").onclick = async () => {
    try {
      await post("/api/waist/stop", {});
      flash("腰已停在当前角", true);
      window.waistTargetDirty = false;
      poll();
    } catch (e) { flash(String(e), false); }
  };
}

async function liftStep(sign) {
  const step = parseFloat($("lift-step")?.value);
  if (!Number.isFinite(step) || step <= 0) { flash("步长无效", false); return; }
  try {
    const d = await post("/api/lift/step", { delta_mm: sign * step });
    if (!d.ok) { flash(d.error || "立柱步进失败", false); return; }
    flash(d.message || "立柱步进", true);
    window.liftTargetDirty = false;
    poll();
  } catch (e) { flash(String(e), false); }
}

if ($("btn-lift-up")) $("btn-lift-up").onclick = () => liftStep(1);

if ($("btn-lift-down")) $("btn-lift-down").onclick = () => liftStep(-1);

if ($("btn-lift-go")) {
  $("btn-lift-go").onclick = async () => {
    const mm = liftStrokeFromUi();
    if (!Number.isFinite(mm)) { flash("目标行程无效", false); return; }
    try {
      const d = await post("/api/lift/goto", { stroke_mm: mm });
      if (!d.ok) { flash(d.error || "立柱失败", false); return; }
      flash(d.message || `立柱前往 ${mm} mm`, true);
      window.liftTargetDirty = false;
      poll();
    } catch (e) { flash(String(e), false); }
  };
}

if ($("btn-lift-stop")) {
  $("btn-lift-stop").onclick = async () => {
    try {
      const d = await post("/api/lift/stop", {});
      flash(d.ok ? "立柱已停" : (d.error || "停止失败"), !!d.ok);
      poll();
    } catch (e) { flash(String(e), false); }
  };
}

$("btn-waist-go").onclick = () => waistGotoFromUi(true);
