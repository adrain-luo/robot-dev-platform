// 02-arm.js —— 手臂控制：关节滑条与目标（按卡片选的手）、控制状态 applyCtrl、接管 / 释放 / 急停 / 左右拖拽 / 重力 / 限速
// 由 web/static/js/ 下各文件按文件名顺序拼成 /static/app.js（同一个脚本作用域）。

const ARM_JOINTS = [
  { key: "sh_pitch", fallback: [-177, 153] },
  { key: "sh_roll", fallback: [-129, 129] },
  { key: "sh_yaw", fallback: [-150, 150] },
  { key: "elbow", fallback: [-60, 120] },
  { key: "wr_roll", fallback: [-113, 113] },
  { key: "wr_pitch", fallback: [-92.5, 92.5] },
  { key: "wr_yaw", fallback: [-92.5, 92.5] },
];

let armSliderDragging = false;

let armTargetDirty = false;

let lastArmJoints = null;  // 「手臂控制」所选那只手的实测 joints[]

function ensureGotoInputs() {
  const row = $("goto-inputs");
  if (!row || row.querySelector(".arm-joint")) return;
  ARM_JOINTS.forEach((spec, i) => {
    const [lo, hi] = spec.fallback;
    const wrap = document.createElement("div");
    wrap.className = "arm-joint";
    wrap.dataset.idx = String(i);
    wrap.innerHTML =
      `<div class="arm-lab">${spec.key}<em class="arm-motor">电机 —</em></div>` +
      `<input class="g-slider" type="range" min="${lo}" max="${hi}" step="0.5" value="0">` +
      `<input class="g-q" type="number" step="0.5" min="${lo}" max="${hi}" value="0">`;
    const slider = wrap.querySelector(".g-slider");
    const num = wrap.querySelector(".g-q");
    slider.addEventListener("pointerdown", () => { armSliderDragging = true; });
    slider.addEventListener("input", () => {
      num.value = slider.value;
      armTargetDirty = true;
    });
    slider.addEventListener("pointerup", async () => {
      armSliderDragging = false;
      await armGotoFromUi(false);
    });
    num.addEventListener("input", () => {
      const v = parseFloat(num.value);
      if (Number.isFinite(v)) slider.value = String(v);
      armTargetDirty = true;
    });
    num.addEventListener("change", async () => { await armGotoFromUi(false); });
    row.appendChild(wrap);
  });
}

function applyArmLimits(joints) {
  ensureGotoInputs();
  const rows = [...document.querySelectorAll("#goto-inputs .arm-joint")];
  rows.forEach((row, i) => {
    const j = joints && joints[i];
    const [flo, fhi] = ARM_JOINTS[i].fallback;
    const lo = j && j.limit_deg ? Number(j.limit_deg[0]) : flo;
    const hi = j && j.limit_deg ? Number(j.limit_deg[1]) : fhi;
    const slider = row.querySelector(".g-slider");
    const num = row.querySelector(".g-q");
    const motor = row.querySelector(".arm-motor");
    if (slider) { slider.min = String(lo); slider.max = String(hi); }
    if (num) { num.min = String(lo); num.max = String(hi); }
    if (motor) motor.textContent = j && j.motor != null ? `电机 ${j.motor}` : "电机 —";
  });
}

function fillQ(qRad) {
  ensureGotoInputs();
  [...document.querySelectorAll("#goto-inputs .arm-joint")].forEach((row, i) => {
    const rad = Number(qRad[i]);
    if (!Number.isFinite(rad)) return;
    const deg = rad * 180 / Math.PI;
    const slider = row.querySelector(".g-slider");
    const num = row.querySelector(".g-q");
    if (slider) slider.value = String(deg);
    if (num) num.value = deg.toFixed(1);
  });
  armTargetDirty = true;
}

function readArmTargetRad() {
  return [...document.querySelectorAll("#goto-inputs .arm-joint")].map((row) => {
    const v = parseFloat(row.querySelector(".g-q")?.value);
    return Number.isFinite(v) ? v * Math.PI / 180 : NaN;
  });
}

function syncArmSlidersFromLive(joints) {
  if (!joints || !joints.length) return;
  lastArmJoints = joints;
  applyArmLimits(joints);
  if (armSliderDragging || armTargetDirty) return;
  if ([...document.querySelectorAll("#goto-inputs .g-q")].some((el) => document.activeElement === el)) return;
  fillQ(joints.map((j) => j.q_rad));
  armTargetDirty = false;
}

async function armGotoFromUi(confirmGo) {
  const q = readArmTargetRad();
  if (q.some((x) => !Number.isFinite(x))) { flash("手臂目标角不完整", false); return; }
  const arm = armPick("motion");
  const busy = armBusyReason(arm);
  if (busy) { flash(busy, false); return; }
  if (confirmGo && !confirm(`将限速把${ARM_SIDE[arm]}臂走到目标关节角。扶住、守急停。继续？`)) return;
  try {
    const d = await post("/api/target", { q, arm });
    flash(d && d.detour ? `${ARM_SIDE[arm]}臂直线会碰，已绕行：${collisionPlanText(d.detour)}`
                        : `${ARM_SIDE[arm]}臂目标已更新`, true);
    armTargetDirty = false;
    poll();
  } catch (e) { flash(String(e), false); }
}

let lastCtrl = null;

let tcpJogBusy = false;

let speedDirty = false;

window.speedDirty = false;

// 限速滑条：拖动时只改显示，松手才发给后端
function showSpeed(v) {
  const x = Number(v);
  if (!Number.isFinite(x)) return;
  if ($("speed-val")) $("speed-val").textContent = x.toFixed(2);
  if ($("speed-deg")) $("speed-deg").textContent = (x * 180 / Math.PI).toFixed(1);
}
document.getElementById("speed").addEventListener("input", () => {
  window.speedDirty = true;
  showSpeed($("speed").value);
});

// 指令话题占用：有没有别的程序在往 rt/lowcmd / rt/arm_sdk 发令（hw/cmd_monitor.py）
function applyCmdBus(bus, engaged) {
  const el = $("bus-pill");
  if (!el) return;
  if (!bus) { el.hidden = true; return; }
  el.hidden = false;
  const topic = bus.topic || "指令话题";
  if (bus.foreign) {
    el.textContent = `${topic} 有别人在发 ${Math.round(bus.foreign_hz)} Hz`;
    el.className = "pill bad";
  } else {
    el.textContent = engaged ? `${topic} 仅本程序` : `${topic} 空闲`;
    el.className = "pill ok";
  }
  const lines = [`${topic}：本程序 ${bus.own_hz} Hz，别人 ${bus.foreign_hz} Hz`];
  if (bus.last_foreign_age_s != null) {
    let s = `最近一次别人的指令：${bus.last_foreign_age_s} s 前`;
    if (bus.last_kp_max != null) s += ` · kp 最大 ${Number(bus.last_kp_max).toFixed(0)}`;
    if (bus.last_weight != null) s += ` · 权重 ${Number(bus.last_weight).toFixed(2)}`;
    lines.push(s);
  }
  for (const h of bus.local_hints || []) lines.push(`本机疑似：${h.pid} ${h.cmd}`);
  if (bus.foreign) lines.push(engaged ? "接管中仍有别人发令：两边会打架，先停掉对方或急停释放" : "有别人在发令时不能接管");
  el.title = lines.join(String.fromCharCode(10));
}

function applyCtrl(ctrl) {
  if (!ctrl) return;
  applyCmdBus(ctrl.cmd_bus, !!ctrl.engaged);
  lastCtrl = ctrl;
  window.lastCtrl = ctrl;
  const allow = !!ctrl.allow_real_motion;
  // 急停锁存（运控核心）：一切新运动都被拒绝，直到「复位急停」
  const latched = !!(ctrl.motion && ctrl.motion.latched);
  const resetBtn = $("btn-estop-reset");
  if (resetBtn) {
    resetBtn.hidden = !latched;
    resetBtn.disabled = !allow;
  }
  // 接管过渡中（arm_sdk）或急停锁存：不能走点 / 拖拽 / 点动。须在下面第一次用到之前定义
  const settling = !!(ctrl.engaged && ctrl.ready === false) || latched;
  $("mode-badge").textContent = allow ? "可运动" : "只读";
  $("mode-badge").className = "badge " + (allow ? "real" : "ro");
  ["btn-engage", "btn-release", "btn-hold", "btn-waist-fill", "btn-waist-go", "btn-waist-yaw-m", "btn-waist-yaw-p", "btn-waist-pitch-m", "btn-waist-pitch-p", "btn-lift-up", "btn-lift-down", "btn-lift-go", "lift-slider", "lift-target", "lift-step", "speed", "grav-on", "btn-id-measure", "btn-id-apply", "btn-id-clear", "btn-he-run"].forEach((id) => {
    const el = $(id); if (el) el.disabled = !allow;
  });
  const mirror = $("btn-poseset-mirror");
  if (mirror) mirror.disabled = !window.currentSet;
  const arms = ctrl.arms || {};
  document.querySelectorAll(".btn-teach").forEach((btn) => {
    const a = arms[btn.dataset.arm] || {};
    btn.disabled = !allow || !ctrl.engaged || settling || !!a.owner;
    btn.classList.toggle("on", !!a.float);
    btn.textContent = `${a.float ? "结束" : "拖"}${ARM_SIDE[btn.dataset.arm]}`;
  });
  $("grav-on").checked = ctrl.gravity_ff !== false;
  const gd = ctrl.gravity_dir || {};
  if ($("grav-imu-wrap")) {
    $("grav-imu-wrap").hidden = !gd.available;
    $("grav-imu").checked = !!gd.enabled;
    $("grav-imu").disabled = !allow;
  }
  // 重力方向：开了 IMU 就显示前馈实际用的躯干倾角，读数不可信时显示原因
  const tiltText = (t) => {
    const p = Number(t?.pitch_deg || 0), r = Number(t?.roll_deg || 0);
    return `${p >= 0 ? "前倾" : "后仰"} ${Math.abs(p).toFixed(1)}° ${r >= 0 ? "右倾" : "左倾"} ${Math.abs(r).toFixed(1)}°`;
  };
  const gravText = gd.enabled
    ? (gd.error ? ` · 重力 IMU：${gd.error}` : ` · 重力按 IMU（躯干${tiltText(gd.used)}）`)
    : "";
  // 每只手一段：受控 / 拖拽中 / 被某任务占用 / 在走；前馈带上负载名
  const armText = (name) => {
    const a = arms[name] || {};
    let t = a.float ? "拖拽" : a.owner ? a.owner : a.moving ? "在走" : "锁住";
    if (a.payload_mass_kg != null) t += ` 载${Number(a.payload_mass_kg).toFixed(2)}kg`;
    return `${ARM_SIDE[name]}:${t}`;
  };
  // 经守护层发令：守护层急停 / 失联 / 被别人接管时，显示原因（不是"过渡中"）
  const gproblem = ctrl.engaged && ctrl.commander && ctrl.commander.kind === "guardian" ? ctrl.commander.problem : null;
  $("engage-state").textContent = allow
    ? latched ? `急停已锁定 · 两手保持当前角；确认安全后点「复位急停」`
    : gproblem ? `守护层 · ${gproblem}` : (ctrl.transition ? `过渡中 · ${ctrl.transition}` : ctrl.engaged
        ? `已接管 · ${["left", "right"].filter((n) => arms[n]).map(armText).join(" ")}${
            ctrl.waist_locked ? " · 腰保持" : ""}${gravText}${ctrl.loop_hz ? ` · ${Math.round(ctrl.loop_hz)} Hz` : ""}`
        : `未接管${gravText}`)
    : "只读（加 --allow-real-motion 后可接管）";
  const motionArm = arms[armPick("motion")] || {};
  document.querySelectorAll(".g-q, .g-slider").forEach((el) => {
    el.disabled = !allow || !!motionArm.float || !!motionArm.owner || settling;
  });
  const jogArm = arms[armPick("jog")] || {};
  document.querySelectorAll(".tcp-jog-btn").forEach((el) => {
    el.disabled = !allow || !ctrl.engaged || !!jogArm.float || !!jogArm.owner || tcpJogBusy || settling;
  });
  ["tcp-jog-step-mm", "tcp-jog-step-deg"].forEach((id) => {
    const el = $(id);
    if (el) el.disabled = !allow;
  });
  const waistNeed = !allow || !ctrl.engaged || !!ctrl.float || settling || !(window.ROBOT?.waist?.movable ?? true);
  ["btn-waist-go", "btn-waist-fill", "btn-waist-yaw-m", "btn-waist-yaw-p", "btn-waist-pitch-m", "btn-waist-pitch-p",
   "waist-yaw", "waist-pitch", "waist-step", "waist-yaw-slider", "waist-pitch-slider"].forEach((id) => {
    const el = $(id);
    if (el) el.disabled = waistNeed;
  });
  const waistStop = $("btn-waist-stop");
  if (waistStop) waistStop.disabled = !allow || !ctrl.engaged;
  // 立柱走 Agv 服务，不要求臂接管；停止按钮只读也可按
  ["btn-lift-up", "btn-lift-down", "btn-lift-go", "lift-slider", "lift-target", "lift-step"].forEach((id) => {
    const el = $(id);
    if (el) el.disabled = !allow;
  });
  const liftStop = $("btn-lift-stop");
  if (liftStop) liftStop.disabled = false;
  const sp = $("speed");
  if (ctrl.max_speed_rad_s != null && document.activeElement !== sp && !window.speedDirty) {
    const v = Number(ctrl.max_speed_rad_s);
    if (Number(sp.value) !== v) sp.value = v;
    showSpeed(v);
  }
}

$("btn-engage").onclick = async () => {
  const sdk = window.ROBOT?.transitions;
  const secs = (k, d) => Math.round(Number(window.ROBOT?.transition_s?.[k]) || d);
  const g = window.ROBOT?.guardian;
  const ask = g
    ? `将经本体守护层（${g.address}）接管：守护层退出高层运动模式后保持当前姿态。之后本页面卡住或断网，手臂原地保持、不会掉。请扶住机器人。继续？`
    : sdk
    ? `将经 rt/arm_sdk 接管腰与手臂（运控保持站立），约 ${secs("engage", 3)} 秒过渡后才能走点。请有人看护、守急停。继续？`
    : "将发布 rt/lowcmd 接管（会退出高层运动模式）。请扶住机器人。继续？";
  if (!confirm(ask)) return;
  try { await post("/api/engage", {}); flash("已接管：保持当前姿态", true); } catch (e) {
    // 守护层急停 / 故障锁存：现场确认安全后可以在这里复位并重新接管
    if (g && /复位/.test(String(e)) && confirm(`${String(e)}\n\n现场已确认安全，复位守护层并重新接管？`)) {
      try { await post("/api/guardian/reset", {}); flash("守护层已复位，已重新接管：保持当前姿态", true); } catch (e2) { flash(String(e2), false); }
      return;
    }
    flash(String(e), false);
  }
};

$("btn-release").onclick = async () => {
  const rel = Math.round(Number(window.ROBOT?.transition_s?.release) || 3);
  const g = window.ROBOT?.guardian;
  const ask = g
    ? `释放：守护层约 ${Math.round(Number(g.release_s) || 3)} 秒内把刚度降到 0，之后停止发令，手臂会慢慢垂下，请扶住。继续？`
    : window.ROBOT?.transitions
    ? `释放：约 ${rel} 秒内把手臂交还运控（手臂会回到运控自己的姿态），之后停止发令。继续？`
    : "释放后停止发令，手臂可能下垂，请扶住。继续？";
  if (!confirm(ask)) return;
  try { await post("/api/release", {}); flash("已释放", true); } catch (e) { flash(String(e), false); }
};

$("btn-hold").onclick = async () => {
  // 工作流在两步之间（例如正在识别、手臂没动）时，后端没有可停的运动：前端也要让它别再走下一步
  const wf = window.wfDetectGoto;
  if (wf && wf.busy) wf.abort = true;
  try { await post("/api/hold", {}); flash("急停已锁定：任务已停、两手保持当前角；确认现场安全后点「复位急停」", true); } catch (e) { flash(String(e), false); }
};

$("btn-estop-reset").onclick = async () => {
  const ask = "确认现场安全、手臂周围没有人和障碍，解除急停锁定？\n\n解除后手臂仍保持当前姿态，不会自己动；被急停打断的任务不会继续，要重新开始。";
  if (!confirm(ask)) return;
  try { await post("/api/estop/reset", {}); flash("急停已解除：手臂保持当前姿态", true); } catch (e) { flash(String(e), false); }
};

// 拖拽按手分开：一只手卸力拖拽时，另一只手照常受控
document.querySelectorAll(".btn-teach").forEach((btn) => {
  btn.onclick = async () => {
    const arm = btn.dataset.arm;
    const side = ARM_SIDE[arm];
    const on = !((window.lastCtrl?.arms || {})[arm] || {}).float;
    if (on && !confirm(`将让${side}臂卸力拖拽（kp=0，保留重力前馈），另一只手照常受控。请先扶住${side}臂。继续？`)) return;
    try {
      await post("/api/teach", { enabled: on, arm });
      flash(on ? `${side}臂拖拽中：手推手臂；急停可立刻变硬` : `${side}臂已结束拖拽并锁在当前角`, true);
    } catch (e) { flash(String(e), false); }
  };
});

$("grav-on").onchange = async () => {
  try {
    await post("/api/gravity", { enabled: $("grav-on").checked });
    flash($("grav-on").checked ? "重力前馈开（α=1）" : "重力前馈关", true);
  } catch (e) { flash(String(e), false); $("grav-on").checked = !$("grav-on").checked; }
};

if ($("grav-imu")) {
  $("grav-imu").onchange = async () => {
    const on = $("grav-imu").checked;
    if (window.lastCtrl?.engaged && !confirm(on
      ? "重力前馈改按躯干 IMU 的重力方向算，约 0.5 s 平滑过渡，手臂可能轻微动一下。扶住、守急停。继续？"
      : "重力前馈改回按躯干竖直算，约 0.5 s 平滑过渡。继续？")) {
      $("grav-imu").checked = !on;
      return;
    }
    try {
      await post("/api/gravity-imu", { enabled: on });
      flash(on ? "重力前馈按躯干 IMU 算" : "重力前馈按躯干竖直算", true);
    } catch (e) { flash(String(e), false); $("grav-imu").checked = !on; }
  };
}

$("speed").addEventListener("change", async () => {
  const v = parseFloat($("speed").value);
  try {
    await post("/api/speed", { max_speed_rad_s: v });
    window.speedDirty = false;
    flash(`限速 ${v.toFixed(2)} rad/s（${(v * 180 / Math.PI).toFixed(1)}°/s）`, true);
  } catch (e) { flash(String(e), false); window.speedDirty = false; }
});

// 「手臂控制」换手：滑条改跟那只手的实测角
document.addEventListener("arm-pick", (ev) => {
  if (ev.detail.key !== "motion") return;
  armTargetDirty = false;
  armSliderDragging = false;
  poll();
});
