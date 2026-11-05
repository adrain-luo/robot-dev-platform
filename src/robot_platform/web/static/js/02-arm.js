// 02-arm.js —— 手臂控制：关节滑条与目标、出发、控制状态 applyCtrl、接管 / 释放 / 急停 / 拖拽 / 重力 / 限速、切换控制臂
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

let lastArmJoints = null;

 // 控制臂实测 joints[]

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
  if (confirmGo && !confirm("将限速走到目标关节角。扶住、守急停。继续？")) return;
  try {
    await post("/api/target", { q });
    flash("手臂目标已更新", true);
    armTargetDirty = false;
    poll();
  } catch (e) { flash(String(e), false); }
}

let lastCtrl = null;

let tcpJogBusy = false;

let speedDirty = false;

window.speedDirty = false;

document.getElementById("speed").addEventListener("input", () => { window.speedDirty = true; });

function applyCtrl(ctrl) {
  if (!ctrl) return;
  lastCtrl = ctrl;
  window.lastCtrl = ctrl;
  const allow = !!ctrl.allow_real_motion;
  // 接管过渡中（arm_sdk）：不能走点 / 拖拽 / 点动。须在下面第一次用到之前定义
  const settling = !!(ctrl.engaged && ctrl.ready === false);
  $("mode-badge").textContent = allow ? "可运动" : "只读";
  $("mode-badge").className = "badge " + (allow ? "real" : "ro");
  ["btn-engage", "btn-release", "btn-hold", "btn-teach", "btn-speed", "btn-goto", "btn-fill", "btn-waist-fill", "btn-waist-go", "btn-waist-yaw-m", "btn-waist-yaw-p", "btn-waist-pitch-m", "btn-waist-pitch-p", "btn-lift-up", "btn-lift-down", "btn-lift-go", "lift-slider", "lift-target", "lift-step", "control-arm", "speed", "grav-on", "btn-id-measure", "btn-id-apply", "btn-id-clear", "btn-he-run"].forEach((id) => {
    const el = $(id); if (el) el.disabled = !allow;
  });
  const mirror = $("btn-poseset-mirror");
  if (mirror) mirror.disabled = !window.currentSet;
  if ($("btn-teach")) $("btn-teach").disabled = !allow || !ctrl.engaged || settling;
  $("btn-teach").textContent = ctrl.float ? "结束拖拽" : "拖拽";
  $("grav-on").checked = ctrl.gravity_ff !== false;
  $("engage-state").textContent = allow
    ? (ctrl.transition ? `过渡中 · ${ctrl.transition}` : ctrl.engaged
        ? `${ctrl.float ? "拖拽中" : "已接管"} · ${ctrl.arm}${ctrl.waist_locked ? " · 腰保持" : ""}${
            ctrl.payload_mass_kg != null ? ` · 载 ${Number(ctrl.payload_mass_kg).toFixed(2)} kg${ctrl.payload_name ? "(" + ctrl.payload_name + ")" : ""}` : ""
          }`
        : "未接管")
    : "只读（加 --allow-real-motion 后可接管）";
  document.querySelectorAll(".g-q, .g-slider").forEach((el) => { el.disabled = !allow || !!ctrl.float || settling; });
  $("btn-goto").disabled = !allow || !ctrl.engaged || !!ctrl.float || settling;
  document.querySelectorAll(".tcp-jog-btn").forEach((el) => {
    el.disabled = !allow || !ctrl.engaged || !!ctrl.float || tcpJogBusy || settling;
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
  }
  if (ctrl.arm) $("control-arm").value = ctrl.arm;
}

$("arm-select").onchange = () => { poll(); window.V3.draw(); };

$("btn-fill").onclick = () => {
  const view = $("control-arm")?.value || "right";
  fetch("/api/status?arm=" + view).then((r) => r.json()).then((d) => {
    if (d.ok) {
      applyArmLimits(d.joints);
      fillQ(d.joints.map((j) => j.q_rad));
      flash("已填入当前角", true);
    }
  }).catch((e) => flash(String(e), false));
};

$("btn-engage").onclick = async () => {
  const sdk = window.ROBOT?.transitions;
  const ask = sdk
    ? "将经 rt/arm_sdk 接管腰与手臂（运控保持站立），约 12 秒过渡后才能走点。请有人看护、守急停。继续？"
    : "将发布 rt/lowcmd 接管（会退出高层运动模式）。请扶住机器人。继续？";
  if (!confirm(ask)) return;
  try { await post("/api/engage", {}); flash("已接管：保持当前姿态", true); } catch (e) { flash(String(e), false); }
};

$("btn-release").onclick = async () => {
  const ask = window.ROBOT?.transitions
    ? "释放：约 5 秒内把手臂交还运控，之后停止发令。继续？"
    : "释放后停止发令，手臂可能下垂，请扶住。继续？";
  if (!confirm(ask)) return;
  try { await post("/api/release", {}); flash("已释放", true); } catch (e) { flash(String(e), false); }
};

$("btn-hold").onclick = async () => {
  try { await post("/api/hold", {}); flash("急停：退出拖拽，保持当前角", true); } catch (e) { flash(String(e), false); }
};

$("btn-teach").onclick = async () => {
  const on = $("btn-teach").textContent !== "结束拖拽";
  if (on && !confirm("将卸力拖拽（kp=0，保留重力前馈）。请先扶住手臂。继续？")) return;
  try {
    await post("/api/teach", { enabled: on });
    flash(on ? "拖拽中：手推手臂；急停可立刻变硬" : "已结束拖拽并锁在当前角", true);
  } catch (e) { flash(String(e), false); }
};

$("grav-on").onchange = async () => {
  try {
    await post("/api/gravity", { enabled: $("grav-on").checked });
    flash($("grav-on").checked ? "重力前馈开（α=1）" : "重力前馈关", true);
  } catch (e) { flash(String(e), false); $("grav-on").checked = !$("grav-on").checked; }
};

$("speed").addEventListener("keydown", (e) => {
  if (e.key === "Enter") { e.preventDefault(); $("btn-speed").click(); }
});

$("btn-speed").onclick = async () => {
  try {
    await post("/api/speed", { max_speed_rad_s: parseFloat($("speed").value) });
    window.speedDirty = false;
    flash("已设限速", true);
  } catch (e) { flash(String(e), false); }
};

$("btn-goto").onclick = () => armGotoFromUi(true);

$("control-arm").onchange = async () => {
  try { await post("/api/control-arm", { arm: $("control-arm").value }); flash("控制臂已切换", true); }
  catch (e) { flash(String(e), false); }
};
