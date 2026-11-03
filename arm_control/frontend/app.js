const $ = (id) => document.getElementById(id);

const sub3 = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add3 = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const scale3 = (a, k) => [a[0] * k, a[1] * k, a[2] * k];
const dot3 = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross3 = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm3 = (a) => { const n = Math.hypot(...a) || 1; return [a[0] / n, a[1] / n, a[2] / n]; };

const V3 = {
  canvas: null, ctx: null, scene: null, yaw: -2.3, pitch: 0.4, dist: 1.6, target: [0.15, 0, 0.15], drag: null,
  reset() { this.yaw = -2.3; this.pitch = 0.4; this.dist = 1.6; this.target = [0.15, 0, 0.15]; this.draw(); },
  project(p) {
    const cy = Math.cos(this.yaw), sy = Math.sin(this.yaw), cp = Math.cos(this.pitch), sp = Math.sin(this.pitch);
    const eye = [this.target[0] + this.dist * cp * cy, this.target[1] + this.dist * cp * sy, this.target[2] + this.dist * sp];
    const f = norm3(sub3(this.target, eye)), r = norm3(cross3(f, [0, 0, 1])), u = cross3(r, f);
    const d = sub3(p, eye), x = dot3(d, r), y = dot3(d, u), z = dot3(d, f);
    if (z < 0.05) return null;
    const W = this.canvas.width, H = this.canvas.height, fl = 0.9 * H;
    return [W / 2 + (x / z) * fl, H / 2 - (y / z) * fl, z];
  },
  line(a, b, color, width = 1, dash = []) {
    const A = this.project(a), B = this.project(b); if (!A || !B) return;
    const c = this.ctx; c.strokeStyle = color; c.lineWidth = width; c.setLineDash(dash);
    c.beginPath(); c.moveTo(A[0], A[1]); c.lineTo(B[0], B[1]); c.stroke(); c.setLineDash([]);
  },
  dot(p, color, r = 3, label) {
    const P = this.project(p); if (!P) return;
    const c = this.ctx; c.fillStyle = color; c.beginPath(); c.arc(P[0], P[1], r, 0, Math.PI * 2); c.fill();
    if (label) { c.font = "11px system-ui"; c.fillText(label, P[0] + 7, P[1] - 5); }
  },
  chain(pts, color, width) {
    for (let i = 1; i < pts.length; i++) this.line(pts[i - 1], pts[i], color, width);
    pts.forEach((p, i) => this.dot(p, i === 0 ? "#2f6feb" : color, i === 0 ? 5 : 3.5));
  },
  drawArm(sc, color, label) {
    if (!sc || !sc.links) return;
    this.chain(sc.links, color, 4);
    if (sc.cmd_links) {
      for (let i = 1; i < sc.cmd_links.length; i++) this.line(sc.cmd_links[i - 1], sc.cmd_links[i], "#6b7280", 2, [4, 4]);
    }
    if (sc.target_links) {
      for (let i = 1; i < sc.target_links.length; i++) this.line(sc.target_links[i - 1], sc.target_links[i], "#60a5fa", 2, [6, 4]);
    }
    (sc.pose_tips || []).forEach((p, i) => p && this.dot(p, "rgba(110,231,168,0.75)", 2.5, i === 0 ? "姿态集" : undefined));
    const T = sc.wrist_T;
    if (T) {
      const o = [T[0][3], T[1][3], T[2][3]], L = 0.06;
      this.line(o, add3(o, [T[0][0] * L, T[1][0] * L, T[2][0] * L]), "#e5484d", 1.5);
      this.line(o, add3(o, [T[0][1] * L, T[1][1] * L, T[2][1] * L]), "#46a758", 1.5);
      this.line(o, add3(o, [T[0][2] * L, T[1][2] * L, T[2][2] * L]), "#3e63dd", 1.5);
      this.dot(o, color, 4, label);
    }
    if (sc.payload_point && sc.payload_mass > 1e-6) {
      const T = sc.wrist_T;
      const o = T ? [T[0][3], T[1][3], T[2][3]] : sc.links[sc.links.length - 1];
      this.line(o, sc.payload_point, "#e879f9", 1.5);
      const r = 4 + 4 * Math.min(sc.payload_mass, 1.5);
      this.dot(sc.payload_point, "#e879f9", r, `${Number(sc.payload_mass).toFixed(2)} kg`);
      this.line(sc.payload_point, add3(sc.payload_point, [0, 0, -0.04]), "#e879f9", 2);
    }
  },
  draw() {
    if (window.RobotView && window.RobotView.sync(this.scene)) return;
  },
  init() {
    this.canvas = $("scene3d");
    const start = () => window.RobotView && window.RobotView.attach(this.canvas);
    if (window.RobotView) start();
    else window.addEventListener("robot-view-ready", start, { once: true });
  },
};
window.V3 = V3;

function fmt(v, n = 2) {
  if (v == null || Number.isNaN(v)) return "—";
  return Number(v).toFixed(n);
}

function appendJointRows(tb, joints, { section } = {}) {
  if (section) {
    const hr = document.createElement("tr");
    hr.className = "j-section";
    hr.innerHTML = `<td colspan="5" class="j-sub">${section}</td>`;
    tb.appendChild(hr);
  }
  for (const j of joints || []) {
    const tr = document.createElement("tr");
    const live = Math.abs(j.dq) > 0.02;
    tr.innerHTML =
      `<td class="j-name">${j.label}</td>` +
      `<td class="j-q">${fmt(j.q_deg)}°</td>` +
      `<td class="j-sub">电机 ${j.motor}</td>` +
      `<td class="j-sub ${live ? "dq-live" : ""}">dq ${fmt(j.dq, 3)}</td>` +
      `<td class="j-sub">τ ${fmt(j.tau_est)}</td>`;
    tb.appendChild(tr);
  }
}
const WAIST_ROLE = { yaw: "旋转", pitch: "俯仰", roll: "侧倾" };
window.ROBOT = null;

// 机型能力（/api/robot）：标题、腰与立柱控件按它显示或隐藏
function applyRobotCaps(caps) {
  window.ROBOT = caps;
  const title = `${caps.label} 臂控`;
  document.title = title;
  if ($("app-title")) $("app-title").textContent = title;
  const show = (id, on) => { const el = $(id); if (el) el.style.display = on ? "" : "none"; };
  const w = caps.waist;
  const desc = w ? w.joints.map((j) => `电机 ${j.motor} ${WAIST_ROLE[j.role] || j.role}`).join(" / ") : "无";
  if ($("joints-note")) $("joints-note").textContent = `臂：视图所选（双臂时显示控制臂）。腰：${desc}。`;
  show("lift-zero-block", !!caps.lift);
  show("lift-ctrl", !!caps.lift);
  show("waist-ctrl", !!(w && w.movable));
  if ($("waist-note")) {
    if (!w) $("waist-note").textContent = "这台没有腰。";
    else if (!w.movable) $("waist-note").textContent = "腰：只在接管时保持当前角，不开放转动（站立时转腰会影响平衡）。";
    else {
      const lim = w.joints.map((j) => `${WAIST_ROLE[j.role] || j.role} ${j.limit_deg[0].toFixed(1)}°～${j.limit_deg[1].toFixed(1)}°`).join("，");
      $("waist-note").textContent = `腰：接管后与手臂共用限速。${lim}。条形拖到目标松手后走。`;
    }
  }
  if (w) {
    for (const j of w.joints) {
      for (const id of [`waist-${j.role}-slider`, `waist-${j.role}`]) {
        const el = $(id);
        if (el) { el.min = String(j.limit_deg[0].toFixed(1)); el.max = String(j.limit_deg[1].toFixed(1)); }
      }
    }
  }
  if (caps.lift) {
    for (const id of ["lift-slider", "lift-target"]) {
      const el = $(id);
      if (el) el.max = String(Math.round(caps.lift.stroke_max_mm));
    }
  }
}
fetch("/api/robot", { cache: "no-store" }).then((r) => r.json()).then(applyRobotCaps).catch(() => {});

function renderTable(snap, waist) {
  const tb = $("joints-table").querySelector("tbody");
  tb.innerHTML = "";
  if (snap && snap.joints && snap.joints.length) {
    appendJointRows(tb, snap.joints, { section: `臂 · ${snap.arm || ""}` });
  }
  if (waist && waist.joints && waist.joints.length) {
    appendJointRows(tb, waist.joints, { section: `腰 · ${waist.joints.map((j) => `${j.motor} ${WAIST_ROLE[j.label.replace("waist_", "")] || j.label}`).join(" / ")}` });
  }
}

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

function setPill(ok, text, cls) {
  const el = $("pill");
  el.textContent = text;
  el.className = "pill " + (cls || (ok ? "ok" : "bad"));
}

function flash(text, ok) {
  const el = $("msg");
  el.className = ok ? "ok" : "err";
  el.textContent = text;
}

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
let lastArmJoints = null; // 控制臂实测 joints[]

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

async function post(path, body) {
  const res = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body || {}),
  });
  const data = await res.json();
  if (!data.ok) throw new Error(data.error || res.statusText);
  return data;
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
  const settling = !!(ctrl.engaged && ctrl.ready === false);
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


let inflight = false;
async function poll() {
  if (inflight) return;
  inflight = true;
  try {
    const res = await fetch("/api/scene", { cache: "no-store" });
    const data = await res.json();
    const right = data.arms && data.arms.right;
    const left = data.arms && data.arms.left;
    const err = data.error || "";
    if (!res.ok || data.ok === false) {
      flash(err || "读取失败", false);
      const dds = /No message|Timeout|timeout/.test(err);
      setPill(false, dds ? "lowstate 失败" : "接口失败");
      if (!right && !left) return;
    } else {
      const age = Math.min(right?.age_s ?? 99, left?.age_s ?? 99);
      const fresh = age < 0.5;
      setPill(fresh, fresh ? `lowstate OK  ${age.toFixed(3)} s` : `信号陈旧  ${age.toFixed(2)} s`, fresh ? "ok" : "warn");
    }
    $("meta").textContent = `${data.iface || ""} · ${data.topic || ""} · mode_machine=${right?.mode_machine ?? "—"}`;
    if (data.ctrl) applyCtrl(data.ctrl);
    if (typeof applyIdentify === "function" && data.identify) {
      try { applyIdentify(data.identify); } catch (e) { flash(String(e), false); }
    }
    if (data.handeye && typeof applyHeRun === "function") {
      try { applyHeRun(data.handeye); } catch (e) { flash(String(e), false); }
    }
    const modeLabel = data.ctrl?.float ? "实测 · 拖拽卸力" : (data.ctrl?.engaged ? "实测 · 已接管" : (data.mode === "real" ? "实测 · 未接管" : "实测 · 只读"));
    if (window.currentSet && window.currentSet.poses) {
      const tips = window.currentSet.poses.map((p) => p.tip_xyz).filter(Boolean);
      const armSc = window.currentSet.arm === "left" ? left?.scene : right?.scene;
      if (armSc) armSc.pose_tips = tips;
    }
    const joints = {};
    for (const arm of [right, left]) {
      for (const j of (arm && arm.joints) || []) joints[j.name] = j.q_rad;
    }
    Object.assign(joints, data.waist || {});
    Object.assign(joints, data.body || {});
    if (data.lift && data.lift.joints) Object.assign(joints, data.lift.joints);
    window.lastWaist = data.waist || {};
    renderWaist(data);
    window.lastLift = data.lift || null;
    renderLift(data.lift);
    renderTcp(data.tcp, { left: left && left.scene, right: right && right.scene });
    renderSpatial(data.spatial);
    renderWorkPose(data.work_pose, data.tcp);
    renderBoard(data.board);
    renderDetect(data.detect);
    if (data.runtime) renderRuntimeConfig(data.runtime);
    V3.scene = { right: right?.scene, left: left?.scene, joints, modeLabel };
    V3.draw();
    const view = $("arm-select").value;
    const ctrlArm = data.ctrl?.arm || "right";
    const tableArm = view === "left" ? left : view === "right" ? right : (ctrlArm === "left" ? left : right);
    renderTable(tableArm, data.waist);
    const ctrlSnap = ctrlArm === "left" ? left : right;
    if (ctrlSnap && ctrlSnap.joints) syncArmSlidersFromLive(ctrlSnap.joints);
    const note = $("arm-ctrl-now");
    if (note && ctrlSnap && ctrlSnap.joints) {
      const live = ctrlSnap.joints.map((j) => `${j.label} ${Number(j.q_deg).toFixed(1)}°`).join(" · ");
      note.textContent = `控制臂 ${ctrlArm} · ${live}`;
    }
  } catch (e) {
    flash(String(e), false);
    setPill(false, "服务断开");
  } finally {
    inflight = false;
  }
}

ensureGotoInputs();
V3.init();

function bindBook(book) {
  const marks = [...book.querySelectorAll(".bookmark")];
  const pages = [...book.querySelectorAll(".page")];
  function openPage(name) {
    const current = marks.find((m) => m.classList.contains("active"))?.dataset.page;
    if (book.classList.contains("open") && current === name) {
      book.classList.remove("open");
      marks.forEach((m) => {
        m.classList.remove("active");
        m.setAttribute("aria-pressed", "false");
      });
      return;
    }
    book.classList.add("open");
    marks.forEach((m) => {
      const on = m.dataset.page === name;
      m.classList.toggle("active", on);
      m.setAttribute("aria-pressed", on ? "true" : "false");
    });
    pages.forEach((p) => {
      p.classList.toggle("active", p.dataset.page === name);
    });
    if (typeof window.refillSelects === "function") {
      setTimeout(() => window.refillSelects(), 0);
    }
  }
  marks.forEach((m) => m.addEventListener("click", () => openPage(m.dataset.page)));
}
document.querySelectorAll(".book").forEach(bindBook);

document.querySelectorAll(".subfold-toggle").forEach((btn) => {
  btn.addEventListener("click", (ev) => {
    ev.stopPropagation();
    const fold = btn.closest(".subfold");
    if (fold) fold.classList.toggle("open");
  });
});

document.querySelectorAll(".float").forEach((card) => {
  const btn = card.querySelector(".float-toggle");
  if (!btn) return;
  btn.addEventListener("click", () => {
    const opening = !card.classList.contains("open");
    card.classList.toggle("open");
    if (opening && typeof window.refillSelects === "function") {
      setTimeout(() => window.refillSelects(), 0);
    }
    if (opening && card.id === "panel-he" && typeof window.refreshHandeye === "function") {
      window.refreshHandeye();
    }
    if (card.id === "panel-cam") {
      if (opening) refreshCamPreview();
      else {
        const img = $("cam-preview");
        if (img && img.dataset.obj) {
          URL.revokeObjectURL(img.dataset.obj);
          delete img.dataset.obj;
          img.removeAttribute("src");
        }
      }
    }
  });
});
setInterval(poll, 100);
poll();

let stillDirty = false;
let pinnedVal = null;
let valList = [];
let lastIdentify = null;
let valWasRunning = false;

function viewFromPin(p) {
  const rows = p.rows || [];
  return {
    kind: "validate",
    running: false,
    session: p.session,
    phase: "",
    progress: [rows.length, rows.length],
    band: p.friction_band || p.band || {},
    summary: p.summary || {},
    rows,
    pinned: true,
    file: p.file,
    finished: p.finished,
  };
}

function renderVal(id) {
  if (!$("val-table")) return;
  const running = !!(id && id.kind === "validate" && id.running);
  const src = (!running && pinnedVal) ? viewFromPin(pinnedVal) : (id || {});
  const [a, b] = src.progress || [0, 0];
  if (running) {
    $("val-progress").textContent = `${src.phase || ""} ${a}/${b}`;
  } else if (src.pinned) {
    $("val-progress").textContent = src.session || "";
  } else {
    $("val-progress").textContent = "";
  }
  const fileEl = $("val-file");
  if (fileEl) {
    const file = src.file || (src.session ? `arm_payload_gravity/data/validate/${src.session}.json` : "");
    fileEl.textContent = file;
  }
  const band = src.band || {};
  if (band.default_deg) {
    const where = band.fallback ? "无双向数据，常数兜底" : `${band.n_sessions}次双向 / ${band.n_poses}姿态`;
    $("val-band").textContent = `摩擦带（${where}）默认 [${band.default_deg.map((x) => Number(x).toFixed(2)).join(", ")}] °`;
  }
  const sum = src.summary || {};
  const host = $("val-summary");
  host.replaceChildren();
  for (const [k, v] of Object.entries(sum)) {
    const box = document.createElement("div");
    box.className = "scheme";
    const title = document.createElement("b");
    title.textContent = k;
    const ang = document.createElement("span");
    ang.innerHTML = `关节均 <strong>${Number(v.err_max_deg_mean).toFixed(2)}°</strong> · 最差 <strong>${Number(v.err_max_deg_worst).toFixed(2)}°</strong>`;
    const ee = document.createElement("span");
    ee.innerHTML = `末端均 <strong>${Number(v.ee_mm_mean).toFixed(1)} mm</strong> · 最差 <strong>${Number(v.ee_mm_worst).toFixed(1)} mm</strong>`;
    box.append(title, ang, ee);
    host.append(box);
  }
  const tb = $("val-table").querySelector("tbody");
  tb.innerHTML = (src.rows || []).map((r) =>
    `<tr><td>${r.mode}</td><td>${r.pose}</td><td>${Number(r.err_max_deg).toFixed(2)}</td>` +
    `<td>${Number(r.ee_mm).toFixed(1)}</td><td>${r.corr_applied || 0}${r.corr_reverted ? "↩" : ""}</td>` +
    `<td>${Number(r.total_s).toFixed(1)}${r.timed_out ? "⚠" : ""}</td></tr>`
  ).join("");
}

function filteredValList() {
  if (!window.currentSet) return valList;
  const rows = valList.filter((s) => s.arm === window.currentSet.arm && s.poseset === window.currentSet.name);
  return rows.length ? rows : valList;
}

function fillValSelect() {
  const sel = $("val-session");
  if (!sel || document.activeElement === sel) return;
  const keep = (pinnedVal && pinnedVal.session) || sel.value;
  const rows = filteredValList();
  while (sel.options.length) sel.remove(0);
  sel.add(new Option(rows.length ? "打开已保存的对比…" : "还没有对比记录", ""));
  for (const s of rows) {
    const side = s.arm === "left" ? "左" : (s.arm === "right" ? "右" : "?");
    const rel = `data/validate/${s.name}.json`;
    const opt = new Option(`${rel} · ${side} ${s.poseset || ""}`, s.name);
    opt.title = s.file || rel;
    sel.add(opt);
  }
  if (keep && [...sel.options].some((o) => o.value === keep)) sel.value = keep;
}

async function refreshValList() {
  const d = await fetch("/api/validate/list", { cache: "no-store" }).then((r) => r.json());
  if (!d.ok) return;
  valList = d.list || [];
  fillValSelect();
}

async function openVal(name) {
  if (!name) return;
  const d = await fetch("/api/validate/result?name=" + encodeURIComponent(name), { cache: "no-store" }).then((r) => r.json());
  if (!d.ok) throw new Error(d.error || "打不开对比记录");
  pinnedVal = d.result;
  const meta = valList.find((s) => s.name === name);
  if (meta && meta.file) pinnedVal.file = meta.file;
  fillValSelect();
  renderVal(lastIdentify || {});
}

function applyIdentify(id) {
  if (!id) return;
  lastIdentify = id;
  const running = !!(id.kind === "validate" && id.running);
  if (valWasRunning && !running && id.session) {
    pinnedVal = {
      session: id.session,
      rows: id.rows || [],
      summary: id.summary || {},
      friction_band: id.band || {},
    };
    refreshValList().catch(() => {});
  }
  valWasRunning = running;
  const [a, b] = id.progress || [0, 0];
  $("id-progress").textContent = id.running ? `${id.phase || ""} ${a}/${b}` : (id.session ? `会话 ${id.session}` : "");
  $("id-log").textContent = (id.log || []).join("\n");
  const sol = id.solution;
  const result = $("id-result");
  if (sol) {
    result.className = "stats";
    result.replaceChildren();
    const com = (sol.com_m || []).map((x) => Number(x).toFixed(3)).join(", ");
    const items = [
      ["质量", `${Number(sol.mass_kg).toFixed(3)} kg`],
      ["质心 m", com],
      ["条件数", Number(sol.regressor_cond).toFixed(1)],
      ["残差 N·m", `${Number(sol.residual_rms_before_nm).toFixed(2)} → ${Number(sol.residual_rms_after_nm).toFixed(2)}`],
      ["估下垂", `${Number(sol.sag_est_before_deg).toFixed(2)}° → ${Number(sol.sag_est_after_deg).toFixed(2)}°`],
    ];
    for (const [k, v] of items) {
      const d = document.createElement("div");
      d.className = "stat";
      const em = document.createElement("em");
      em.textContent = k;
      const strong = document.createElement("strong");
      strong.textContent = v;
      d.append(em, strong);
      result.append(d);
    }
    const files = [];
    if (id.session_dir) files.push(id.session_dir);
    if (id.payload_file) files.push(id.payload_file);
    if (files.length) {
      const foot = document.createElement("p");
      foot.className = "foot";
      foot.textContent = files.join("\n");
      result.append(foot);
    }
  } else if (id.error) {
    result.className = "";
    result.textContent = id.error;
  }
  const allow = !!window.lastCtrl?.allow_real_motion && !!window.lastCtrl?.engaged && !window.lastCtrl?.float;
  const measure = $("btn-id-measure");
  const stop = $("btn-id-stop");
  const valStart = $("btn-val-start");
  if (measure) measure.disabled = !allow || !!id.running;
  if (stop) stop.disabled = !id.running;
  if (valStart) valStart.disabled = !allow || !!id.running;
  renderVal(id);
  renderPayloadTable(id);
  renderSessionSelect(id.sessions || [], id.session);
  const st = id.still;
  if (st && !stillDirty) {
    const map = { "still-dq": "still_dq_rad_s", "still-drift": "still_drift_rad", "still-win": "still_window_s", "still-to": "settle_timeout_s" };
    for (const [idEl, key] of Object.entries(map)) {
      const el = $(idEl);
      if (!el || document.activeElement === el) continue;
      if (st[key] != null && Number(el.value) !== Number(st[key])) el.value = st[key];
    }
  }
}

let poseList = [];

function refillSelects() {
  renderPoseList(poseList);
  if (lastIdentify) {
    renderPayloadTable(lastIdentify);
    renderSessionSelect(lastIdentify.sessions || [], lastIdentify.session);
  }
  // 打开卡片时若还没拉过列表，不要用空 valList 冲掉服务端注入的选项
  if (valList.length) fillValSelect();
  else refreshValList().catch(() => {});
}
window.refillSelects = refillSelects;
refreshValList().catch(() => {});
refreshPoseList().catch((e) => {
  const m = document.getElementById("msg");
  if (!m) return;
  m.className = "err";
  m.textContent = "姿态集列表没有加载: " + e;
});

function posesetName() {
  return $("poseset-select").value;
}

function writeOptions(sel, placeholder, rows, keep) {
  if (!sel) return;
  const prev = keep != null ? keep : sel.value;
  while (sel.options.length) sel.remove(0);
  sel.add(new Option(placeholder, ""));
  for (const row of rows) sel.add(new Option(row.label, row.value));
  if (prev && [...sel.options].some((o) => o.value === prev)) sel.value = prev;
}

const CFG_MUTATORS = [
  "btn-cfg-stereo", "btn-cfg-grav-left", "btn-cfg-grav-right",
  "btn-cfg-tcp-left", "btn-cfg-tcp-right",
  "cfg-he", "cfg-grav-left", "cfg-grav-right", "cfg-tcp-left", "cfg-tcp-right",
  "btn-he-solve", "btn-id-apply",
];
const CFG_SELECT_IDS = [
  "cfg-he", "cfg-grav-left", "cfg-grav-right", "cfg-tcp-left", "cfg-tcp-right",
  "cfg-wp-left", "cfg-wp-right", "cfg-pose-ik-mode",
];
const cfgDirty = Object.create(null);

function markCfgDirty(id) {
  cfgDirty[id] = true;
}
function clearCfgDirty(id) {
  if (id) delete cfgDirty[id];
  else CFG_SELECT_IDS.forEach((k) => delete cfgDirty[k]);
}
function cfgSelectBusy(id) {
  const sel = $(id);
  return !!cfgDirty[id] || (sel && document.activeElement === sel);
}

CFG_SELECT_IDS.forEach((id) => {
  const el = $(id);
  if (el) el.addEventListener("change", () => markCfgDirty(id));
});

function shortPath(p) {
  if (!p) return "—";
  const parts = String(p).split("/");
  return parts.length >= 2 ? parts.slice(-2).join("/") : p;
}

function fmtGrav(side) {
  if (!side || !side.name && side.mass_kg == null) return "未配置";
  const m = side.mass_kg != null ? `${Number(side.mass_kg).toFixed(3)} kg` : "";
  return `${side.name || "（无名）"}${m ? " · " + m : ""}`;
}

function renderRuntimeConfig(rt) {
  if (!rt) return;
  window.lastRuntime = rt;
  const locked = !!rt.locked;
  window.configLocked = locked;
  const st = rt.stereo || {};
  const g = rt.gravity || {};
  const tcp = rt.tcp || {};
  const badge = $("cfg-lock-badge");
  if (badge) {
    badge.textContent = locked ? "已锁定" : "";
    badge.classList.toggle("locked", locked);
  }
  const now = $("cfg-now");
  if (now) {
    const applied = g.applied_name
      ? `控制臂当前前馈「${g.applied_name}」${g.applied_mass_kg != null ? ` ${Number(g.applied_mass_kg).toFixed(3)} kg` : ""}`
      : "控制臂当前无负载前馈";
    now.classList.toggle("locked", locked);
    now.textContent =
      `${locked ? "🔒 已锁定\n" : ""}` +
      `手眼：${st.active_result || "—"}\n` +
      `双目 yaml：${shortPath(st.intrinsics)}\n` +
      applied;
  }
  const wp = rt.work_pose || {};
  const nowL = $("cfg-now-left");
  if (nowL) {
    const show = wp.locked && wp.arm === "left" ? (wp.active_name || "无") : "无";
    nowL.textContent = `重力：${fmtGrav(g.left)}\nTCP：${tcp.left?.name || "—"}\n工作姿态锁定：${show}`;
  }
  const nowR = $("cfg-now-right");
  if (nowR) {
    const show = wp.locked && wp.arm === "right" ? (wp.active_name || "无") : "无";
    nowR.textContent = `重力：${fmtGrav(g.right)}\nTCP：${tcp.right?.name || "—"}\n工作姿态锁定：${show}`;
  }
  const bound = $("cfg-stereo-bound");
  if (bound) {
    const pending = cfgDirty["cfg-he"] && $("cfg-he")?.value;
    const pendingYaml = pending
      ? (st.handeye_saved || []).find((s) => s.name === pending)?.intrinsics_yaml
      : "";
    const show = pendingYaml || st.intrinsics;
    bound.textContent = pending
      ? `双目 yaml：${shortPath(show)}（已选「${pending}」，点应用后生效）`
      : `双目 yaml：${shortPath(st.intrinsics)}（随手眼绑定，不可单独改）`;
  }
  const heSel = $("cfg-he");
  if (heSel && !cfgSelectBusy("cfg-he")) {
    writeOptions(
      heSel,
      (rt.stereo?.handeye_saved || []).length ? "选择手眼结果…" : "还没有保存的手眼",
      (rt.stereo?.handeye_saved || []).map((s) => ({
        value: s.name,
        label: `${s.name}${s.baseline_mm != null ? ` · ${Number(s.baseline_mm).toFixed(1)} mm` : ""}${s.intrinsics_yaml ? " · 含双目" : ""}`,
      })),
      rt.stereo?.active_result || ""
    );
  }
  for (const [selId, arm] of [["cfg-grav-left", "left"], ["cfg-grav-right", "right"]]) {
    const sel = $(selId);
    if (!sel || cfgSelectBusy(selId)) continue;
    const rows = (rt.gravity?.profiles || [])
      .filter((p) => !p.arm || p.arm === arm)
      .map((p) => ({
        value: p.name,
        label: `${p.name}${p.mass_kg != null ? ` · ${Number(p.mass_kg).toFixed(2)} kg` : ""}`,
      }));
    const keep = arm === "left" ? rt.gravity?.left?.name : rt.gravity?.right?.name;
    writeOptions(sel, rows.length ? `选择${arm === "left" ? "左" : "右"}臂补偿…` : "还没有补偿文件", rows, keep || "");
  }
  for (const [selId, arm] of [["cfg-tcp-left", "left"], ["cfg-tcp-right", "right"]]) {
    const sel = $(selId);
    if (!sel || cfgSelectBusy(selId)) continue;
    const rows = (rt.tcp?.saved || [])
      .filter((s) => s.arm === arm)
      .map((s) => ({
        value: s.name,
        label: `${s.name}${s.residual_max_mm != null ? ` · max ${Number(s.residual_max_mm).toFixed(1)} mm` : ""}`,
      }));
    const keep = arm === "left" ? rt.tcp?.active_left : rt.tcp?.active_right;
    writeOptions(
      sel,
      rows.length ? `选择${arm === "left" ? "左" : "右"}臂 TCP…` : `还没有${arm === "left" ? "左" : "右"}臂 TCP`,
      rows,
      keep || ""
    );
  }
  // 工作姿态：按该侧当前 TCP 列姿态；配置锁定时仍可改（不上锁）
  for (const [selId, arm] of [["cfg-wp-left", "left"], ["cfg-wp-right", "right"]]) {
    const sel = $(selId);
    if (!sel || cfgSelectBusy(selId)) continue;
    const tcpName = arm === "left"
      ? ($("cfg-tcp-left")?.value || rt.tcp?.active_left || "")
      : ($("cfg-tcp-right")?.value || rt.tcp?.active_right || "");
    const all = wp.saved_all || window.lastWorkPoseAll || [];
    const rows = all
      .filter((s) => s.tcp_name === tcpName && (!s.arm || s.arm === arm))
      .map((s) => ({ value: s.name, label: s.note ? `${s.name} · ${s.note}` : s.name }));
    const keep = (wp.locked && wp.arm === arm && wp.tcp_name === tcpName) ? (wp.active_name || "") : "";
    writeOptions(
      sel,
      tcpName
        ? (rows.length ? `选择工作姿态（${tcpName}）…` : `「${tcpName}」还没有工作姿态`)
        : "先选该侧 TCP",
      rows,
      keep
    );
  }
  const allowMotion = !!window.lastCtrl?.allow_real_motion;
  CFG_MUTATORS.forEach((id) => {
    const el = $(id);
    if (!el) return;
    if (locked) {
      el.disabled = true;
      return;
    }
    if (id.startsWith("cfg-") || id.startsWith("btn-cfg-")) {
      el.disabled = false;
      return;
    }
    if (id === "btn-he-solve") {
      el.disabled = false;
      return;
    }
    if (id === "btn-id-apply") {
      el.disabled = !allowMotion;
    }
  });
  // 工作姿态 / IK 策略：永远可改（不受配置锁定）
  ["cfg-wp-left", "cfg-wp-right", "btn-cfg-wp-left", "btn-cfg-wp-right",
    "cfg-pose-ik-mode", "btn-cfg-pose-ik-mode"].forEach((id) => {
    const el = $(id);
    if (!el) return;
    if (id.startsWith("btn-cfg-wp")) el.disabled = !allowMotion;
    else el.disabled = false;
  });
  const ikSel = $("cfg-pose-ik-mode");
  if (ikSel && !cfgSelectBusy("cfg-pose-ik-mode")) {
    const mode = wp.ik_mode === "single" ? "single" : "robust";
    ikSel.value = mode;
  }
  const lockBtn = $("btn-cfg-lock");
  const unlockBtn = $("btn-cfg-unlock");
  if (lockBtn) lockBtn.disabled = locked;
  if (unlockBtn) unlockBtn.disabled = !locked;
}

function renderPoseList(list) {
  const sel = $("poseset-select");
  if (!sel) return;
  const rows = (list || []).map((s) => {
    const side = s.arm === "left" ? "左" : "右";
    const cond = s.cond != null ? ` · cond ${s.cond}` : "";
    return { value: s.name, label: `${s.name} · ${side} · ${s.n}点${cond}` };
  });
  writeOptions(sel, rows.length ? "选择姿态集…" : "还没有姿态集", rows, posesetName());
}

function renderChips() {
  const box = $("pose-chips");
  box.innerHTML = "";
  $("pose-cond").textContent = "";
  if (!window.currentSet) return;
  const cond = window.currentSet.regressor_cond;
  $("pose-cond").textContent = window.currentSet.arm === "left" ? "左臂" : "右臂"
    + (cond != null ? ` · 回归 cond ${cond}` : "")
    + (window.currentSet.mirrored_from ? ` · 镜像自 ${window.currentSet.mirrored_from.name}` : "");
  (window.currentSet.poses || []).forEach((p, i) => {
    const wrap = document.createElement("span");
    wrap.className = "chip";
    const b = document.createElement("button");
    b.textContent = p.name || `p${i}`;
    b.title = (p.q || []).map((q) => Number(q).toFixed(2)).join(", ");
    b.onclick = () => gotoPose(i);
    const x = document.createElement("button");
    x.textContent = "×";
    x.title = "删除此姿态";
    x.onclick = (e) => { e.stopPropagation(); deletePose(i); };
    wrap.appendChild(b);
    wrap.appendChild(x);
    box.appendChild(wrap);
  });
}

async function refreshPoseList(selectName) {
  const d = await fetch("/api/posesets").then((r) => r.json());
  if (!d.ok) throw new Error(d.error);
  poseList = d.list || [];
  renderPoseList(poseList);
  if (selectName) $("poseset-select").value = selectName;
  if (posesetName()) await loadCurrentSet();
  else { window.currentSet = null; renderChips(); }
}

async function loadCurrentSet() {
  const name = posesetName();
  if (!name) { window.currentSet = null; renderChips(); return; }
  const d = await fetch("/api/posesets?name=" + encodeURIComponent(name)).then((r) => r.json());
  if (!d.ok) throw new Error(d.error);
  window.currentSet = d.set;
  renderChips();
  try {
    await refreshValList();
    const rows = filteredValList();
    const keep = pinnedVal && rows.some((s) => s.name === pinnedVal.session);
    if (!keep && rows[0] && !(lastIdentify && lastIdentify.kind === "validate" && lastIdentify.running)) {
      await openVal(rows[0].name);
    } else {
      fillValSelect();
    }
  } catch (_) { /* ignore */ }
}

async function gotoPose(i) {
  if (!window.currentSet) return;
  fillQ(window.currentSet.poses[i].q);
  const ctrl = window.lastCtrl;
  if (!ctrl?.allow_real_motion || !ctrl.engaged || ctrl.float) {
    flash("已填入该姿态角（需接管且非拖拽才能走过去）", true);
    return;
  }
  if (ctrl.arm !== window.currentSet.arm) {
    flash(`姿态集是${window.currentSet.arm === "left" ? "左" : "右"}臂，请先切控制臂`, false);
    return;
  }
  try {
    await post("/api/posesets/goto", { name: window.currentSet.name, index: i });
    flash(`正在去 ${window.currentSet.poses[i].name}`, true);
  } catch (e) { flash(String(e), false); }
}

async function deletePose(i) {
  if (!window.currentSet) return;
  if (!confirm(`删除 ${window.currentSet.poses[i].name}？`)) return;
  try {
    const d = await post("/api/posesets/delete-pose", { name: window.currentSet.name, index: i });
    window.currentSet = d.set;
    renderChips();
    await refreshPoseList(window.currentSet.name);
  } catch (e) { flash(String(e), false); }
}

async function recordPose() {
  if (!posesetName()) { flash("先新建或选择姿态集", false); return; }
  try {
    const d = await post("/api/posesets/record", { name: posesetName() });
    window.currentSet = d.set;
    renderChips();
    await refreshPoseList(window.currentSet.name);
    flash(`已记录 ${d.pose.name}（${window.currentSet.arm === "left" ? "左" : "右"}臂实测）`, true);
  } catch (e) { flash(String(e), false); }
}

try {
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
  if (!confirm("将发布 rt/lowcmd 接管（可能释放高层运动模式）。请扶住机器人。继续？")) return;
  try { await post("/api/engage", {}); flash("已接管：保持当前姿态", true); } catch (e) { flash(String(e), false); }
};
$("btn-release").onclick = async () => {
  if (!confirm("释放后停止发令，手臂可能下垂，请扶住。继续？")) return;
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
$("btn-still").onclick = async () => {
  try {
    await post("/api/identify/still", {
      still_dq_rad_s: parseFloat($("still-dq").value),
      still_drift_rad: parseFloat($("still-drift").value),
      still_window_s: parseFloat($("still-win").value),
      settle_timeout_s: parseFloat($("still-to").value),
    });
    stillDirty = false;
    flash("已设静止判定", true);
  } catch (e) { flash(String(e), false); }
};
["still-dq", "still-drift", "still-win", "still-to"].forEach((id) => {
  $(id).addEventListener("input", () => { stillDirty = true; });
});
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
function tcpOnWrist(tcp, arms) {
  const offset = tcp && tcp.result && tcp.result.tcp_m;
  const scene = tcp && tcp.arm === "left" ? arms && arms.left : arms && arms.right;
  const T = scene && scene.wrist_T;
  if (!offset || !T) return null;
  return [0, 1, 2].map((i) => T[i][3] + T[i][0] * offset[0] + T[i][1] * offset[1] + T[i][2] * offset[2]);
}
function renderWorkPose(wp, tcp) {
  if (!wp) return;
  window.lastWorkPose = wp;
  if (wp.saved_all) window.lastWorkPoseAll = wp.saved_all;
  const badge = $("wp-lock-badge");
  if (badge) {
    badge.textContent = wp.locked ? "已锁定" : "";
    badge.classList.toggle("locked", !!wp.locked);
  }
  const now = $("wp-now");
  if (now) {
    now.textContent = wp.locked && wp.active_name
      ? `锁定：${wp.active_name}${wp.note ? " · " + wp.note : ""}`
      : "锁定：无";
  }
  const hint = $("wp-tcp-hint");
  const tcpName = (tcp && tcp.active_name) || "";
  if (hint) {
    hint.textContent = tcpName
      ? `TCP：${tcpName}${tcp.arm ? `（${tcp.arm === "left" ? "左" : "右"}）` : ""}`
      : "TCP：—（先在 TCP 卡片调用一套）";
  }
  const rows = (wp.saved || []).map((s) => ({
    value: s.name,
    label: s.note ? `${s.name} · ${s.note}` : s.name,
  }));
  const placeholder = tcpName
    ? (rows.length ? "选择工作姿态…" : "这个 TCP 还没有工作姿态")
    : "先调用 TCP";
  const preferred = (wp.locked && wp.active_name) || "";
  const sel = $("wp-select");
  if (sel && document.activeElement !== sel) {
    writeOptions(sel, placeholder, rows, preferred || sel.value || "");
  }
  const wfSel = $("wf-dg-wp");
  if (wfSel && document.activeElement !== wfSel) {
    writeOptions(wfSel, placeholder, rows, preferred || wfSel.value || "");
  }
}

async function workPosePost(path, body) {
  const d = await post(path, body || {});
  renderWorkPose(d, { active_name: d.tcp_name || window.lastTcpName, arm: d.arm });
  if (d.runtime) renderRuntimeConfig(d.runtime);
  return d;
}

function renderTcpTable(tcp) {
  const body = $("tcp-table-body");
  const empty = $("tcp-table-empty");
  if (!body) return;
  const saved = (tcp && tcp.saved) || [];
  window.lastTcpSaved = saved;
  const activeL = (tcp && tcp.active_left) || "";
  const activeR = (tcp && tcp.active_right) || "";
  body.innerHTML = "";
  if (empty) empty.classList.toggle("show", saved.length === 0);
  saved.forEach((s) => {
    const name = s.name || "";
    const arm = s.arm === "left" || s.arm === "right" ? s.arm : "";
    const side = arm === "left" ? "左" : arm === "right" ? "右" : "—";
    const on = (arm === "left" && name === activeL) || (arm === "right" && name === activeR);
    const tr = document.createElement("tr");
    tr.dataset.name = name;
    tr.dataset.arm = arm;
    tr.innerHTML = `
      <td class="tcp-name"></td>
      <td>${side}</td>
      <td class="tcp-note"></td>
      <td><button type="button" class="tcp-act tcp-apply${on ? " on" : ""}" data-act="apply" title="${on ? "当前应用" : "设为该手当前 TCP"}">${on ? "✓" : "○"}</button></td>
      <td><button type="button" class="tcp-act tcp-mirror" data-act="mirror" ${arm ? "" : "disabled"}>镜像</button></td>
      <td><button type="button" class="tcp-act tcp-del" data-act="delete">删除</button></td>
    `;
    tr.querySelector(".tcp-name").textContent = name;
    tr.querySelector(".tcp-note").textContent = s.note || "—";
    body.appendChild(tr);
  });
}

function renderTcp(tcp, arms) {
  const note = $("tcp-now");
  if (!note || !tcp) return;
  window.lastTcpName = tcp.active_name || "";
  renderTcpTable(tcp);
  const jogNow = $("tcp-jog-now");
  if (jogNow) {
    if (tcp.active_name) {
      const side = tcp.arm === "left" ? "左臂" : tcp.arm === "right" ? "右臂" : "";
      jogNow.textContent = `当前 TCP「${tcp.active_name}」${side ? ` · ${side}` : ""} · 躯干系点动`;
    } else {
      jogNow.textContent = "还没调用 TCP（先在标定册 TCP 卡片调用）";
    }
  }
  const side = tcp.arm === "left" ? "左臂" : tcp.arm === "right" ? "右臂" : "";
  let text = tcp.active_name ? `会话已加载「${tcp.active_name}」。` : "";
  text += side ? `${side}已记 ${tcp.n} 个姿态` : "还没有姿态";
  if (tcp.note) text += `。${tcp.note}`;
  const r = tcp.result;
  if (r) {
    const mm = (arr) => arr.map((v) => Number(v).toFixed(1)).join(", ");
    const res = r.residual_mm || {};
    text += `。工作点相对腕 [${mm(r.tcp_mm)}] mm。固定点 [${mm(r.fixed_mm)}] mm。残差平均 ${Number(res.mean).toFixed(1)}、最大 ${Number(res.max).toFixed(1)} mm。`;
    if (r.warning) text += r.warning;
  }
  note.textContent = text;
  if (!arms) return;
  const live = tcpOnWrist(tcp, arms);
  const key = live ? live.map((v) => Number(v).toFixed(4)).join(",") : "";
  if (key === window.tcpPointKey) return;
  if (window.RobotView && window.RobotView.setFixedPoint && window.RobotView.setFixedPoint(live || null)) {
    window.tcpPointKey = key;
  }
}
async function tcpPost(path, body) {
  const d = await post(path, body || {});
  renderTcp(d);
  if (d.work_pose) renderWorkPose(d.work_pose, d);
  if (d.runtime) renderRuntimeConfig(d.runtime);
  return d;
}

async function tcpApplySaved(name) {
  if (!name) return;
  try {
    const d = await tcpPost("/api/tcp/load", { name });
    flash(`已应用「${name}」为${d.arm === "left" ? "左" : "右"}臂当前 TCP`, true);
  } catch (e) { flash(String(e), false); }
}

async function tcpMirrorSaved(name) {
  if (!name) return;
  const row = (window.lastTcpSaved || []).find((s) => s.name === name);
  const dst = row && row.arm === "left" ? "右" : "左";
  if (!confirm(`把「${name}」关于躯干 y=0 镜像成${dst}臂？新名为「${name}_${dst}」，不含工作姿态。`)) return;
  try {
    const d = await tcpPost("/api/tcp/mirror", { name });
    flash(d.message || `已镜像为「${d.mirrored_name}」`, true);
  } catch (e) {
    if (String(e).includes("已存在") && confirm(`对侧「${name}_${dst}」已存在，覆盖？`)) {
      try {
        const d = await tcpPost("/api/tcp/mirror", { name, force: true });
        flash(d.message || `已覆盖镜像「${d.mirrored_name}」`, true);
      } catch (e2) { flash(String(e2), false); }
    } else flash(String(e), false);
  }
}

async function tcpDeleteSaved(name) {
  if (!name) return;
  if (!confirm(`删除 TCP「${name}」？将取消该手应用，并删除其下全部工作姿态。`)) return;
  try {
    const d = await tcpPost("/api/tcp/delete", { name });
    flash(d.message || "已删除", true);
  } catch (e) { flash(String(e), false); }
}

if ($("tcp-table-body")) {
  $("tcp-table-body").addEventListener("click", (ev) => {
    const btn = ev.target.closest("button[data-act]");
    if (!btn) return;
    const tr = btn.closest("tr");
    const name = tr?.dataset?.name || "";
    if (!name) return;
    if (btn.dataset.act === "apply") tcpApplySaved(name);
    else if (btn.dataset.act === "mirror") tcpMirrorSaved(name);
    else if (btn.dataset.act === "delete") tcpDeleteSaved(name);
  });
}

if ($("btn-wp-save")) {
  $("btn-wp-save").onclick = async () => {
    const name = $("wp-name").value.trim() || $("wp-select").value;
    if (!name) { flash("先给工作姿态起个名字", false); return; }
    if (!window.lastTcpName) { flash("先调用一套 TCP", false); return; }
    const exists = [...($("wp-select")?.options || [])].some((o) => o.value === name);
    if (exists && !confirm(`覆盖工作姿态「${name}」？`)) return;
    try {
      await workPosePost("/api/work-pose/save", {
        name,
        note: $("wp-note")?.value.trim() || "",
      });
      if ($("wp-name")) $("wp-name").value = "";
      flash(`已记下工作姿态「${name}」`, true);
    } catch (e) { flash(String(e), false); }
  };
}
if ($("btn-wp-load")) {
  $("btn-wp-load").onclick = async () => {
    const name = $("wp-select")?.value;
    if (!name) { flash("先选一个工作姿态", false); return; }
    if (!window.lastTcpName) { flash("先调用一套 TCP", false); return; }
    if (window.lastCtrl && window.lastCtrl.float) { flash("先点急停退出拖拽", false); return; }
    if (!confirm(`将转到「${name}」并锁定腕部方向（尽量保持当前 TCP 位置）。扶住、守急停。继续？`)) return;
    const btn = $("btn-wp-load");
    if (btn) btn.disabled = true;
    flash("正在转到工作姿态…", true);
    try {
      const d = await workPosePost("/api/work-pose/load", {
        name,
        tcp_name: window.lastTcpName,
      });
      flash(
        d.message
          || `已锁定「${name}」· 位置 ${Number(d.measured_miss_mm || 0).toFixed(1)} mm · 姿态 ${Number(d.measured_ori_deg || 0).toFixed(1)}°`,
        true,
      );
    } catch (e) { flash(String(e), false); }
    finally { if (btn) btn.disabled = false; }
  };
}
if ($("btn-wp-clear")) {
  $("btn-wp-clear").onclick = async () => {
    try {
      await workPosePost("/api/work-pose/clear", {});
      flash("已解锁工作姿态", true);
    } catch (e) { flash(String(e), false); }
  };
}
if ($("btn-wp-del")) {
  $("btn-wp-del").onclick = async () => {
    const name = $("wp-select")?.value;
    if (!name) { flash("先选一个工作姿态", false); return; }
    if (!window.lastTcpName) { flash("先调用一套 TCP", false); return; }
    if (!confirm(`删除工作姿态「${name}」？`)) return;
    try {
      await workPosePost("/api/work-pose/delete", { name, tcp_name: window.lastTcpName });
      flash("已删除", true);
    } catch (e) { flash(String(e), false); }
  };
}
$("btn-tcp-record").onclick = async () => {
  try {
    const d = await tcpPost("/api/tcp/record");
    const side = d.arm === "left" ? "左" : "右";
    flash(`已记${side}臂第 ${d.n} 个姿态`, true);
  } catch (e) { flash(String(e), false); }
};
$("btn-tcp-undo").onclick = async () => {
  try { await tcpPost("/api/tcp/undo"); } catch (e) { flash(String(e), false); }
};
$("btn-tcp-clear").onclick = async () => {
  try { await tcpPost("/api/tcp/clear"); flash("已清空 TCP 记录", true); }
  catch (e) { flash(String(e), false); }
};
$("btn-tcp-solve").onclick = async () => {
  try {
    const d = await tcpPost("/api/tcp/solve");
    const res = (d.result && d.result.residual_mm) || {};
    flash(`TCP 已算出，最大残差 ${Number(res.max).toFixed(1)} mm`, true);
  } catch (e) { flash(String(e), false); }
};
$("btn-tcp-save").onclick = async () => {
  const name = $("tcp-name").value.trim();
  if (!name) { flash("填一个名字", false); return; }
  const exists = (window.lastTcpSaved || []).some((s) => s.name === name);
  if (exists && !confirm(`覆盖已保存的「${name}」？`)) return;
  try {
    await tcpPost("/api/tcp/save", { name, note: $("tcp-note").value.trim() });
    $("tcp-name").value = "";
    flash(`已保存「${name}」`, true);
  } catch (e) { flash(String(e), false); }
};
function renderSpatial(pt) {
  const note = $("pt-now");
  if (!note || !pt) return;
  const sel = $("pt-select");
  if (sel && pt.saved && document.activeElement !== sel) {
    // 优先保留下拉框当前选择，避免 poll 用旧 active_name 把刚点的项刷回去
    const names = new Set((pt.saved || []).map((s) => s.name));
    const keep = (sel.value && names.has(sel.value))
      ? sel.value
      : (pt.active_name || "");
    writeOptions(sel, pt.saved.length ? "选择空间点…" : "还没有空间点",
      pt.saved.map((s) => {
        const src = s.source === "detect_center" ? "绿点"
          : s.source === "tcp_fixed" ? "固定点"
          : s.source === "tcp_now" ? "爪尖"
          : (s.note || "");
        const label = src ? `${s.name} · ${src}` : s.name;
        return { value: s.name, label };
      }),
      keep);
  }
  let text = pt.active_name ? `已选「${pt.active_name}」` : "还没有空间点";
  if (pt.point_mm) text += ` [${pt.point_mm.map((v) => Number(v).toFixed(1)).join(", ")}] mm`;
  if (pt.source) text += ` · ${pt.source}`;
  if (pt.note) text += `。${pt.note}`;
  note.textContent = text;
  window.lastSpatialPointM = pt.point_m || null;
  const p = pt.point_m;
  const key = p ? p.map((v) => Number(v).toFixed(4)).join(",") : "";
  if (key !== window.spatialPointKey) {
    if (window.RobotView && window.RobotView.setSpatialPoint && window.RobotView.setSpatialPoint(p || null)) {
      window.spatialPointKey = key;
    }
  }
  syncPtAimNote();
}
function readPtOffsetMm() {
  const n = (id) => {
    const v = parseFloat($(id)?.value);
    return Number.isFinite(v) ? v : 0;
  };
  return { x: n("pt-off-x"), y: n("pt-off-y"), z: n("pt-off-z") };
}
function syncPtAimNote() {
  const el = $("pt-aim-note");
  if (!el) return;
  const off = readPtOffsetMm();
  const base = window.lastSpatialPointM;
  const hasOff = Math.abs(off.x) + Math.abs(off.y) + Math.abs(off.z) > 1e-9;
  if (!base || base.length < 3) {
    el.textContent = hasOff
      ? `偏置 Δ[${off.x}, ${off.y}, ${off.z}] mm（先选空间点）`
      : "瞄准：与空间点重合";
    if (window.RobotView?.setSpatialAim) window.RobotView.setSpatialAim(null);
    return;
  }
  const aim = [
    Number(base[0]) + off.x / 1000,
    Number(base[1]) + off.y / 1000,
    Number(base[2]) + off.z / 1000,
  ];
  if (!hasOff) {
    el.textContent = "瞄准：与空间点重合";
    if (window.RobotView?.setSpatialAim) window.RobotView.setSpatialAim(null);
    return;
  }
  el.textContent =
    `瞄准 [${(aim[0] * 1000).toFixed(1)}, ${(aim[1] * 1000).toFixed(1)}, ${(aim[2] * 1000).toFixed(1)}] mm` +
    ` = 空间点 + Δ[${off.x}, ${off.y}, ${off.z}]`;
  if (window.RobotView?.setSpatialAim) window.RobotView.setSpatialAim(aim);
}
async function pointPost(path, body) {
  const d = await post(path, body || {});
  renderSpatial(d);
  return d;
}
$("btn-pt-fixed").onclick = async () => {
  const name = $("pt-name").value.trim();
  if (!name) { flash("先给空间点起个名字", false); return; }
  const exists = [...$("pt-select").options].some((o) => o.value === name);
  if (exists && !confirm(`覆盖空间点「${name}」？`)) return;
  try {
    await pointPost("/api/points/save-fixed", { name, note: $("pt-note").value.trim() });
    $("pt-name").value = "";
    flash(`已记下「${name}」`, true);
  } catch (e) { flash(String(e), false); }
};
$("btn-pt-now").onclick = async () => {
  const name = $("pt-name").value.trim();
  if (!name) { flash("先给空间点起个名字", false); return; }
  const exists = [...$("pt-select").options].some((o) => o.value === name);
  if (exists && !confirm(`覆盖空间点「${name}」？`)) return;
  try {
    await pointPost("/api/points/save-now", { name, note: $("pt-note").value.trim() });
    $("pt-name").value = "";
    flash(`已记下「${name}」`, true);
  } catch (e) { flash(String(e), false); }
};
if ($("btn-pt-center")) {
  $("btn-pt-center").onclick = async () => {
    const name = $("pt-name").value.trim();
    if (!name) { flash("先给空间点起个名字", false); return; }
    const exists = [...$("pt-select").options].some((o) => o.value === name);
    if (exists && !confirm(`覆盖空间点「${name}」？`)) return;
    try {
      const d = await pointPost("/api/points/save-center", {
        name,
        note: $("pt-note").value.trim(),
      });
      if (!d.ok) { flash(d.error || "记下中心点失败", false); return; }
      $("pt-name").value = "";
      flash(d.message || `已记下中心点「${name}」`, true);
    } catch (e) { flash(String(e), false); }
  };
}
$("btn-pt-load").onclick = async () => {
  const name = $("pt-select").value;
  if (!name) { flash("先选一个空间点", false); return; }
  try {
    const d = await pointPost("/api/points/load", { name });
    if ($("pt-note") && document.activeElement !== $("pt-note")) $("pt-note").value = d.note || "";
    flash(`已调用「${name}」`, true);
  } catch (e) { flash(String(e), false); }
};
$("btn-pt-del").onclick = async () => {
  const name = $("pt-select").value;
  if (!name) { flash("先选一个空间点", false); return; }
  if (!confirm(`删除空间点「${name}」？`)) return;
  try { await pointPost("/api/points/delete", { name }); flash("已删除", true); }
  catch (e) { flash(String(e), false); }
};
function renderBoard(board) {
  const log = $("board-log");
  if (!log || !board) return;
  const lines = board.log || [];
  log.textContent = lines.join("\n") || (board.phase || "");
  const xyz = board.point_xyz;
  const key = xyz && xyz.length ? xyz.length + ":" + xyz[0] + ":" + xyz[xyz.length - 1] : "";
  if (key === window.boardPointKey) return;
  if (window.RobotView && window.RobotView.setBoardPoints && window.RobotView.setBoardPoints(xyz || null)) {
    window.boardPointKey = key;
  }
}
async function boardPost(path, body) {
  const res = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body || {}),
  });
  const data = await res.json();
  renderBoard(data);
  if (!data.ok) throw new Error(data.error || res.statusText);
  return data;
}
$("btn-board-look").onclick = async () => {
  try {
    const d = await boardPost("/api/board/look", {
      board: $("board-size").value.trim() || "9x6",
      ideal: $("board-ideal").checked,
    });
    flash(`看到 ${d.n} 个角点`, true);
  } catch (e) { flash(String(e), false); }
};
$("btn-board-start").onclick = async () => {
  if (window.lastCtrl && window.lastCtrl.float) { flash("先点急停退出拖拽", false); return; }
  if (!confirm("将按限速逐个触摸棋盘角点。扶住、守急停。继续？")) return;
  try {
    await boardPost("/api/board/start", {
      above_cm: parseFloat($("board-above").value) || 5,
      down_cm: parseFloat($("board-down").value) || 4,
      up_cm: parseFloat($("board-up").value) || 4,
    });
    flash("开始触摸", true);
  } catch (e) { flash(String(e), false); }
};
$("btn-board-stop").onclick = async () => {
  try { await boardPost("/api/board/stop", {}); flash("已停止", true); }
  catch (e) { flash(String(e), false); }
};
["pt-off-x", "pt-off-y", "pt-off-z"].forEach((id) => {
  if ($(id)) $(id).addEventListener("input", syncPtAimNote);
});
if ($("btn-pt-off-clear")) {
  $("btn-pt-off-clear").onclick = () => {
    ["pt-off-x", "pt-off-y", "pt-off-z"].forEach((id) => { if ($(id)) $(id).value = "0"; });
    syncPtAimNote();
  };
}
$("btn-pt-goto").onclick = async () => {
  const name = $("pt-select").value;
  if (!name) { flash("先选一个空间点", false); return; }
  if (window.lastCtrl && window.lastCtrl.float) { flash("先点急停退出拖拽", false); return; }
  const off = readPtOffsetMm();
  const refine = !($("pt-refine") && !$("pt-refine").checked);
  const offTxt = `Δ[${off.x}, ${off.y}, ${off.z}] mm`;
  const refineTxt = refine ? "到位后误差补偿（最多 2 步）" : "仅开环走到瞄准点";
  if (!confirm(`将按限速用当前控制臂的 TCP 碰到「${name}」+ ${offTxt}。${refineTxt}。扶住、守急停。继续？`)) return;
  const btn = $("btn-pt-goto");
  if (btn) btn.disabled = true;
  flash(`正在走点（${offTxt}）${refine ? " / 误差补偿" : ""}…`, true);
  try {
    const d = await pointPost("/api/points/goto", {
      name,
      offset_mm: [off.x, off.y, off.z],
      refine,
    });
    if (!d.ok) { flash(d.error || "碰点失败", false); return; }
    const side = d.goto_arm === "left" ? "左臂" : d.goto_arm === "right" ? "右臂" : "";
    const tcp = d.goto_tcp ? ` · ${d.goto_tcp}` : "";
    const plan = Number(d.plan_miss_mm ?? d.miss_mm);
    const meas = Number(d.measured_miss_mm);
    const before = Number(d.measured_before_mm);
    if (!Number.isFinite(meas)) {
      flash(
        `服务端没返回实测误差（规划 ${Number.isFinite(plan) ? plan.toFixed(2) : "?"} mm）。请重启臂控后再试。`,
        false,
      );
      return;
    }
    const to = d.settle_timed_out ? "（停稳超时）" : "";
    const nRef = Number(d.refine_passes) || 0;
    let text = `已到位（${side}${tcp}）${to}`;
    if (Array.isArray(d.offset_mm)) {
      text += ` · 偏置 Δ[${d.offset_mm.map((v) => Number(v).toFixed(1)).join(", ")}]`;
    }
    text += `。规划 ${plan.toFixed(2)} mm`;
    if (Number.isFinite(before) && nRef > 0) {
      text += ` · 修正前实测 ${before.toFixed(2)} mm → 修正 ${nRef} 步后 ${meas.toFixed(2)} mm`;
    } else {
      text += ` · 电机实测相对瞄准点还差 ${meas.toFixed(2)} mm`;
    }
    if (Array.isArray(d.measured_tcp_mm) && d.measured_tcp_mm.length >= 3) {
      text += ` [${d.measured_tcp_mm.map((v) => Number(v).toFixed(1)).join(", ")}]`;
    }
    flash(text, true);
  } catch (e) { flash(String(e), false); }
  finally { if (btn) btn.disabled = false; }
};
async function tcpJogStep(mode, axis, sign) {
  if (tcpJogBusy) return;
  if (!window.lastTcpName) { flash("先调用一套 TCP", false); return; }
  if (window.lastCtrl && window.lastCtrl.float) { flash("先点急停退出拖拽", false); return; }
  if (window.lastCtrl && !window.lastCtrl.engaged) { flash("先接管再点动 TCP", false); return; }
  const stepMm = Number($("tcp-jog-step-mm")?.value);
  const stepDeg = Number($("tcp-jog-step-deg")?.value);
  const body = { mode, axis, sign: Number(sign) };
  if (mode === "translate") {
    if (!Number.isFinite(stepMm) || stepMm <= 0) { flash("平移步长无效", false); return; }
    body.step_mm = stepMm;
  } else {
    if (!Number.isFinite(stepDeg) || stepDeg <= 0) { flash("旋转步长无效", false); return; }
    body.step_deg = stepDeg;
  }
  tcpJogBusy = true;
  document.querySelectorAll(".tcp-jog-btn").forEach((b) => { b.disabled = true; });
  try {
    const d = await post("/api/tcp/jog", body);
    if (d.work_pose) {
      renderWorkPose(d.work_pose, { active_name: window.lastTcpName, arm: d.arm });
    }
    if (d.runtime) renderRuntimeConfig(d.runtime);
    flash(d.message || "已发出 TCP 点动", true);
  } catch (e) {
    flash(String(e), false);
  } finally {
    tcpJogBusy = false;
    const ctrl = window.lastCtrl;
    const allow = !!(ctrl && ctrl.allow_real_motion);
    document.querySelectorAll(".tcp-jog-btn").forEach((b) => {
      b.disabled = !allow || !ctrl?.engaged || !!ctrl?.float;
    });
  }
}
document.querySelectorAll(".tcp-jog-btn").forEach((btn) => {
  btn.addEventListener("click", () => {
    tcpJogStep(btn.dataset.jog, btn.dataset.axis, btn.dataset.sign);
  });
});

async function cfgPost(path, body, okMsg, dirtyIds) {
  const d = await post(path, body || {});
  (dirtyIds || []).forEach(clearCfgDirty);
  if (d.runtime) renderRuntimeConfig(d.runtime);
  else if (d.stereo || d.tcp || d.gravity) renderRuntimeConfig(d);
  if (String(path).indexOf("work-pose") >= 0) {
    renderWorkPose(d, { active_name: d.tcp_name || window.lastTcpName, arm: d.arm });
    if (window.lastRuntime) {
      window.lastRuntime.work_pose = d;
      renderRuntimeConfig(window.lastRuntime);
    }
  }
  flash(okMsg || d.message || "已应用", true);
  return d;
}
$("btn-cfg-stereo").onclick = async () => {
  const name = $("cfg-he").value;
  if (!name) { flash("先选手眼结果", false); return; }
  try {
    await cfgPost(
      "/api/runtime-config/stereo",
      { active_result: name },
      `已应用手眼「${name}」（双目 yaml 已一并绑定）`,
      ["cfg-he"]
    );
  } catch (e) { flash(String(e), false); }
};
$("btn-cfg-grav-left").onclick = async () => {
  const name = $("cfg-grav-left").value;
  if (!name) { flash("先选左臂补偿", false); return; }
  try { await cfgPost("/api/runtime-config/gravity", { arm: "left", name }, `左臂重力已设为「${name}」`, ["cfg-grav-left"]); }
  catch (e) { flash(String(e), false); }
};
$("btn-cfg-grav-right").onclick = async () => {
  const name = $("cfg-grav-right").value;
  if (!name) { flash("先选右臂补偿", false); return; }
  try { await cfgPost("/api/runtime-config/gravity", { arm: "right", name }, `右臂重力已设为「${name}」`, ["cfg-grav-right"]); }
  catch (e) { flash(String(e), false); }
};
$("btn-cfg-tcp-left").onclick = async () => {
  const name = $("cfg-tcp-left").value;
  if (!name) { flash("先选左臂 TCP", false); return; }
  try { await cfgPost("/api/runtime-config/tcp", { arm: "left", name }, `左臂 TCP 已设为「${name}」`, ["cfg-tcp-left"]); }
  catch (e) { flash(String(e), false); }
};
$("btn-cfg-tcp-right").onclick = async () => {
  const name = $("cfg-tcp-right").value;
  if (!name) { flash("先选右臂 TCP", false); return; }
  try { await cfgPost("/api/runtime-config/tcp", { arm: "right", name }, `右臂 TCP 已设为「${name}」`, ["cfg-tcp-right"]); }
  catch (e) { flash(String(e), false); }
};
async function cfgApplyWorkPose(arm) {
  const tcpSel = arm === "left" ? $("cfg-tcp-left") : $("cfg-tcp-right");
  const wpSel = arm === "left" ? $("cfg-wp-left") : $("cfg-wp-right");
  const tcpName = tcpSel?.value || "";
  const name = wpSel?.value || "";
  if (!tcpName) { flash("先选该侧 TCP", false); return; }
  if (!name) { flash("先选工作姿态", false); return; }
  if (window.lastCtrl && window.lastCtrl.float) { flash("先点急停退出拖拽", false); return; }
  if (!confirm(`将转到工作姿态「${name}」（TCP「${tcpName}」）并锁定方向。扶住、守急停。继续？`)) return;
  try {
    await cfgPost(
      "/api/runtime-config/work-pose",
      { arm, tcp_name: tcpName, name },
      `已锁定工作姿态「${name}」`,
      [arm === "left" ? "cfg-wp-left" : "cfg-wp-right"],
    );
  } catch (e) { flash(String(e), false); }
}
if ($("btn-cfg-wp-left")) $("btn-cfg-wp-left").onclick = () => cfgApplyWorkPose("left");
if ($("btn-cfg-wp-right")) $("btn-cfg-wp-right").onclick = () => cfgApplyWorkPose("right");
if ($("btn-cfg-pose-ik-mode")) {
  $("btn-cfg-pose-ik-mode").onclick = async () => {
    const mode = $("cfg-pose-ik-mode")?.value || "robust";
    try {
      await cfgPost(
        "/api/runtime-config/pose-ik-mode",
        { mode },
        mode === "single" ? "已切到旧·单次局部 IK" : "已切到新·多初值 IK",
        ["cfg-pose-ik-mode"],
      );
    } catch (e) { flash(String(e), false); }
  };
}
// TCP 下拉变更时刷新该侧工作姿态列表
["cfg-tcp-left", "cfg-tcp-right"].forEach((id) => {
  const el = $(id);
  if (!el) return;
  el.addEventListener("change", () => {
    if (window.lastRuntime) renderRuntimeConfig(window.lastRuntime);
  });
});
$("btn-cfg-lock").onclick = async () => {
  try { await cfgPost("/api/runtime-config/lock", { locked: true }, "配置已锁定"); }
  catch (e) { flash(String(e), false); }
};
$("btn-cfg-unlock").onclick = async () => {
  try { await cfgPost("/api/runtime-config/lock", { locked: false }, "配置已解锁"); }
  catch (e) { flash(String(e), false); }
};

$("btn-waist-go").onclick = () => waistGotoFromUi(true);
$("btn-goto").onclick = () => armGotoFromUi(true);
$("control-arm").onchange = async () => {
  try { await post("/api/control-arm", { arm: $("control-arm").value }); flash("控制臂已切换", true); }
  catch (e) { flash(String(e), false); }
};
$("poseset-select").onchange = () => loadCurrentSet().catch((e) => flash(String(e), false));
$("btn-poseset-new").onclick = async () => {
  const name = $("poseset-new").value.trim();
  if (!name) { flash("填一个姿态集名字", false); return; }
  const arm = $("poseset-arm").value;
  try {
    const d = await post("/api/posesets/create", { name, arm });
    $("poseset-new").value = "";
    await refreshPoseList(d.set.name);
    flash(`已建「${d.set.name}」${arm === "left" ? "左" : "右"}臂`, true);
  } catch (e) { flash(String(e), false); }
};
$("btn-poseset-del").onclick = async () => {
  const name = posesetName();
  if (!name) return;
  if (!confirm(`删除姿态集「${name}」？`)) return;
  try {
    await post("/api/posesets/delete", { name });
    window.currentSet = null;
    await refreshPoseList();
    flash("已删集", true);
  } catch (e) { flash(String(e), false); }
};
$("btn-poseset-record").onclick = () => recordPose();
$("btn-poseset-mirror").onclick = async () => {
  if (!window.currentSet) { flash("先选姿态集", false); return; }
  const dst = window.currentSet.arm === "left" ? "右" : "左";
  if (!confirm(`把「${window.currentSet.name}」关于 y=0 镜像成${dst}臂姿态集？关节角会改符号，不是原样复制。`)) return;
  try {
    const d = await post("/api/posesets/mirror", { src: window.currentSet.name });
    await refreshPoseList(d.set.name);
    flash(`已镜像为「${d.set.name}」${dst}臂 ${d.set.poses.length} 点`, true);
  } catch (e) {
    if (String(e).includes("已存在") && confirm("对侧集已存在，覆盖？")) {
      try {
        const d = await post("/api/posesets/mirror", { src: window.currentSet.name, force: true });
        await refreshPoseList(d.set.name);
        flash(`已覆盖镜像「${d.set.name}」`, true);
      } catch (e2) { flash(String(e2), false); }
    } else flash(String(e), false);
  }
};
window.addEventListener("keydown", (e) => {
  if (e.code !== "Space" && e.key !== " ") return;
  const t = e.target;
  if (t && (t.tagName === "INPUT" || t.tagName === "TEXTAREA" || t.tagName === "SELECT")) return;
  e.preventDefault();
  recordPose();
});
$("btn-id-measure").onclick = async () => {
  if (!posesetName()) { flash("先选姿态集", false); return; }
  if (!confirm("将按当前限速走完姿态集（双向逼近）。扶住、守急停。继续？")) return;
  try {
    await post("/api/identify/measure", { name: posesetName() });
    flash("开始测量", true);
  } catch (e) { flash(String(e), false); }
};
$("btn-id-stop").onclick = async () => {
  try { await post("/api/identify/stop", {}); flash("已请求停止", true); }
  catch (e) { flash(String(e), false); }
};
$("btn-id-solve").onclick = async () => {
  const session = $("id-session") && $("id-session").value;
  if (!session) { flash("先选一条测量会话（或先测量当前集）", false); return; }
  try {
    const d = await post("/api/identify/solve", { source: $("id-source").value, session });
    applyIdentify(d.identify);
    flash("已用会话 " + session + " 重新求解", true);
  } catch (e) { flash(String(e), false); }
};
function renderSessionSelect(list, current) {
  const sel = $("id-session");
  if (!sel || document.activeElement === sel) return;
  let rows = list || [];
  if (window.currentSet) {
    const filtered = rows.filter((s) => s.arm === window.currentSet.arm && s.poseset === window.currentSet.name);
    if (filtered.length) rows = filtered;
  }
  writeOptions(sel, rows.length ? "选择测量会话再求解…" : "还没有测量会话（先点测量当前集）", rows.map((s) => {
    const side = s.arm === "left" ? "左" : (s.arm === "right" ? "右" : "?");
    const bi = s.bidirectional ? "双向" : "单向";
    return { value: s.name, label: `${s.name} · ${side} ${s.poseset || ""} ${s.n}样 ${bi}` };
  }), current || sel.value);
}

function renderPayloadTable(id) {
  const body = $("payload-table-body");
  const empty = $("payload-table-empty");
  if (!body) return;
  const saved = (id && id.profiles) || [];
  window.lastPayloadSaved = saved;
  const activeL = (id && id.applied_left) || "";
  const activeR = (id && id.applied_right) || "";
  body.innerHTML = "";
  if (empty) empty.classList.toggle("show", saved.length === 0);
  saved.forEach((p) => {
    const name = p.name || "";
    const arm = p.arm === "left" || p.arm === "right" ? p.arm : "";
    const side = arm === "left" ? "左" : arm === "right" ? "右" : "—";
    const mass = p.mass_kg != null && Number.isFinite(Number(p.mass_kg))
      ? `${Number(p.mass_kg).toFixed(2)} kg`
      : "—";
    const on = (arm === "left" && name === activeL) || (arm === "right" && name === activeR);
    const tr = document.createElement("tr");
    tr.dataset.name = name;
    tr.innerHTML = `
      <td class="pay-name"></td>
      <td>${side}</td>
      <td class="pay-meta"></td>
      <td class="pay-meta pay-note"></td>
      <td><button type="button" class="pay-act pay-apply${on ? " on" : ""}" data-act="apply" title="${on ? "该手当前补偿" : "设为该手补偿"}">${on ? "✓" : "○"}</button></td>
      <td><button type="button" class="pay-act pay-del" data-act="delete">删除</button></td>
    `;
    tr.querySelector(".pay-name").textContent = name;
    tr.querySelector(".pay-meta").textContent = mass;
    tr.querySelector(".pay-note").textContent = p.note || "—";
    body.appendChild(tr);
  });
}

async function payloadApplySaved(name) {
  if (!name) return;
  try {
    const d = await post("/api/identify/payload/apply", { name });
    applyIdentify(d.identify);
    if (d.runtime) renderRuntimeConfig(d.runtime);
    window.lastPayloadPick = name;
    flash(d.message || `已应用「${name}」`, true);
  } catch (e) { flash(String(e), false); }
}

async function payloadDeleteSaved(name) {
  if (!name) return;
  if (!confirm(`删除补偿「${name}」？若正在应用将去掉负载项。`)) return;
  try {
    const d = await post("/api/identify/payload/delete", { name });
    applyIdentify(d.identify);
    if (d.runtime) renderRuntimeConfig(d.runtime);
    if (window.lastPayloadPick === name) window.lastPayloadPick = "";
    flash(d.message || "已删除", true);
  } catch (e) { flash(String(e), false); }
}

if ($("payload-table-body")) {
  $("payload-table-body").addEventListener("click", (ev) => {
    const btn = ev.target.closest("button[data-act]");
    if (!btn) return;
    const tr = btn.closest("tr");
    const name = tr?.dataset?.name || "";
    if (!name) return;
    if (btn.dataset.act === "apply") payloadApplySaved(name);
    else if (btn.dataset.act === "delete") payloadDeleteSaved(name);
  });
}

$("btn-id-apply").onclick = async () => {
  try {
    const name = (
      $("payload-new")?.value.trim()
      || (window.currentSet && window.currentSet.name)
      || ""
    ).trim();
    if (!name) { flash("填一个名字再存并应用", false); return; }
    const exists = (window.lastPayloadSaved || []).some((p) => p.name === name);
    if (exists && !confirm(`覆盖已保存的「${name}」？`)) return;
    const d = await post("/api/identify/apply", { name });
    applyIdentify(d.identify);
    if (d.runtime) renderRuntimeConfig(d.runtime);
    window.lastPayloadPick = d.payload?.name || name;
    if ($("payload-new")) $("payload-new").value = "";
    flash(`已存并应用「${d.payload.name}」`, true);
  } catch (e) { flash(String(e), false); }
};
$("btn-id-clear").onclick = async () => {
  try {
    const d = await post("/api/identify/clear", {});
    if (d.identify) applyIdentify(d.identify);
    flash("已去掉负载项，只剩 URDF 臂自重", true);
  } catch (e) { flash(String(e), false); }
};
$("btn-payload-save").onclick = async () => {
  const name = (
    $("payload-new")?.value.trim()
    || (window.currentSet && window.currentSet.name)
    || ""
  ).trim();
  if (!name) { flash("填一个另存名字", false); return; }
  const exists = (window.lastPayloadSaved || []).some((p) => p.name === name);
  if (exists && !confirm(`覆盖已保存的「${name}」？`)) return;
  try {
    const d = await post("/api/identify/payload/save", { name });
    applyIdentify(d.identify);
    if ($("payload-new")) $("payload-new").value = "";
    flash(`已保存「${name}」（未改当前前馈）`, true);
  } catch (e) { flash(String(e), false); }
};
$("btn-val-start").onclick = async () => {
  if (!posesetName()) { flash("先选姿态集", false); return; }
  const modes = [];
  if ($("val-base").checked) modes.push("baseline");
  if ($("val-pay").checked) modes.push("payload");
  if ($("val-corr").checked) modes.push("payload_correct");
  if (!modes.length) { flash("至少勾一种方案", false); return; }
  const payloadName = (lastIdentify && lastIdentify.applied_name)
    || window.lastPayloadPick
    || "";
  if ((modes.includes("payload") || modes.includes("payload_correct")) && !payloadName) {
    flash("补偿方案需要先在末端负载管理里应用一套补偿", false); return;
  }
  if (!confirm("将按所选方案走完整姿态集（同一限速）。扶住、守急停。继续？")) return;
  try {
    await post("/api/validate/start", {
      name: posesetName(),
      modes,
      payload: payloadName,
      correction_iters: parseInt($("val-iters").value, 10) || 2,
    });
    flash("开始对比", true);
  } catch (e) { flash(String(e), false); }
};
$("val-session").onchange = async () => {
  const name = $("val-session").value;
  if (!name) return;
  try { await openVal(name); }
  catch (e) { flash(String(e), false); }
};

function heForm() {
  return {
    session: $("he-session").value,
    square_size_mm: $("he-square").value,
    eye: $("he-eye").value,
    board_size: $("he-board").value.trim() || "11x8",
    camera_host: $("he-host").value.trim(),
  };
}
function renderHeTable(st) {
  const body = $("he-table-body");
  const empty = $("he-table-empty");
  if (!body) return;
  const saved = (st && st.saved) || [];
  window.lastHeSaved = saved;
  const active = (st && st.active_result) || "";
  body.innerHTML = "";
  if (empty) empty.classList.toggle("show", saved.length === 0);
  saved.forEach((s) => {
    const name = s.name || "";
    const side = s.arm === "left" ? "左" : s.arm === "right" ? "右" : "—";
    const base = s.baseline_mm != null && Number.isFinite(Number(s.baseline_mm))
      ? `${Number(s.baseline_mm).toFixed(1)} mm`
      : "—";
    const on = !!name && name === active;
    const tr = document.createElement("tr");
    tr.dataset.name = name;
    tr.innerHTML = `
      <td class="he-name"></td>
      <td>${side}</td>
      <td class="he-meta"></td>
      <td><button type="button" class="he-act he-apply${on ? " on" : ""}" data-act="apply" title="${on ? "当前应用" : "设为当前手眼"}">${on ? "✓" : "○"}</button></td>
      <td><button type="button" class="he-act he-del" data-act="delete">删除</button></td>
    `;
    tr.querySelector(".he-name").textContent = name;
    tr.querySelector(".he-meta").textContent = base;
    body.appendChild(tr);
  });
}

function renderHe(st) {
  if (!st) return;
  renderHeTable(st);
  const sel = $("he-session");
  if (sel && document.activeElement !== sel) {
    writeOptions(sel, (st.sessions || []).length ? "选择采集会话…" : "还没有采集会话",
      (st.sessions || []).map((s) => {
        const side = s.arm === "left" ? "左" : "右";
        const ps = s.poseset ? ` · ${s.poseset}` : "";
        return { value: s.name, label: `${s.name} · ${side} ${s.n}张${ps}` };
      }),
      st.session || sel.value);
  }
  if ($("he-count") && !st.running) $("he-count").textContent = st.session ? `${st.count || 0}张` : "";
  const msg = $("he-msg");
  if (msg && document.activeElement !== $("he-host")) {
    msg.textContent = st.error || st.camera_error || (st.running ? (st.phase || "") : "");
  }
  const sq = $("he-square");
  if (sq && document.activeElement !== sq && st.square_size_mm != null && sq.value === "") sq.value = st.square_size_mm;
  if (st.eye && document.activeElement !== $("he-eye")) $("he-eye").value = st.eye;
  if (st.board_size && document.activeElement !== $("he-board")) $("he-board").value = st.board_size;
  if (st.camera_host && document.activeElement !== $("he-host") && $("he-host").value === "") $("he-host").value = st.camera_host;
  const box = $("he-result");
  if (!box) return;
  const res = st.result;
  if (!res) {
    if (!st.intrinsics_fit) return;
  }
  const lines = [];
  const fit = st.intrinsics_fit;
  if (fit && fit.left_fx != null) {
    const rms = fit.left_rms_px != null
      ? `<span class="note">重投影 左 ${Number(fit.left_rms_px).toFixed(3)} / 右 ${Number(fit.right_rms_px).toFixed(3)} / 双目 ${Number(fit.stereo_rms_px).toFixed(3)} px · 基线 ${Number(fit.baseline_mm).toFixed(1)} mm</span>`
      : `<span class="note">基线 ${Number(fit.baseline_mm).toFixed(1)} mm</span>`;
    const n = fit.num_pairs != null ? ` · ${fit.num_pairs} 对` : "";
    lines.push(`<div class="stat"><em>本次内参${n}</em><strong>左 fx ${Number(fit.left_fx).toFixed(1)} · 右 fx ${Number(fit.right_fx).toFixed(1)}</strong>${rms}</div>`);
  }
  if (res) {
    for (const eye of ["left", "right"]) {
      const r = res[eye];
      if (!r) continue;
      const t = r.t_cam2base_m || [];
      const tr = r.residual_translation_mm || {};
      lines.push(`<div class="stat"><em>${eye === "left" ? "左目" : "右目"} → ${r.base_link || "torso_link"}</em><strong>${t.map((x) => Number(x).toFixed(4)).join(", ")} m</strong><span class="note">平移残差 均 ${Number(tr.mean).toFixed(2)} / 最大 ${Number(tr.max).toFixed(2)} mm · 内点 ${r.num_inliers}/${r.num_samples}</span></div>`);
    }
    if (res.stereo_baseline_mm != null) {
      lines.push(`<div class="stat"><em>右目相对左目</em><strong>${Number(res.stereo_baseline_mm).toFixed(2)} mm</strong></div>`);
    }
  }
  if (lines.length) box.innerHTML = lines.join("");
}

async function heApplySaved(name) {
  if (!name) return;
  try {
    const d = await post("/api/handeye/load", { name });
    renderHe(d);
    if (d.runtime) renderRuntimeConfig(d.runtime);
    flash(`已应用手眼「${name}」`, true);
  } catch (e) { flash(String(e), false); }
}

async function heDeleteSaved(name) {
  if (!name) return;
  if (!confirm(`删除手眼「${name}」？若正在应用将取消，并删掉配套 yaml。`)) return;
  try {
    const d = await post("/api/handeye/delete", { name });
    renderHe(d);
    if (d.runtime) renderRuntimeConfig(d.runtime);
    const box = $("he-result");
    if (box) box.innerHTML = "";
    flash("已删除", true);
  } catch (e) { flash(String(e), false); }
}

if ($("he-table-body")) {
  $("he-table-body").addEventListener("click", (ev) => {
    const btn = ev.target.closest("button[data-act]");
    if (!btn) return;
    const tr = btn.closest("tr");
    const name = tr?.dataset?.name || "";
    if (!name) return;
    if (btn.dataset.act === "apply") heApplySaved(name);
    else if (btn.dataset.act === "delete") heDeleteSaved(name);
  });
}
async function refreshHandeye() {
  const res = await fetch("/api/handeye", { cache: "no-store" });
  const data = await res.json();
  if (!data.ok) throw new Error(data.error || "手眼状态失败");
  renderHe(data);
  return data;
}
window.refreshHandeye = refreshHandeye;
function applyHeRun(st) {
  if (!st) return;
  if (st.running) {
    const p = st.progress || [0, 0];
    if ($("he-count")) $("he-count").textContent = `${p[0]}/${p[1]}`;
    const msg = $("he-msg");
    if (msg) msg.textContent = st.phase || "";
  }
  if (window.heWasRunning && !st.running) {
    refreshHandeye().catch((e) => flash(String(e), false));
  }
  window.heWasRunning = !!st.running;
}
window.applyHeRun = applyHeRun;
let camPreviewBusy = false;
function camPreviewWanted() {
  const card = $("panel-cam");
  if (!card || !card.classList.contains("open")) return false;
  const book = card.closest(".book");
  if (book && !book.classList.contains("open")) return false;
  const page = card.closest(".page");
  if (page && !page.classList.contains("active")) return false;
  if (typeof document.hidden === "boolean" && document.hidden) return false;
  return true;
}
async function refreshCamPreview() {
  if (!camPreviewWanted() || camPreviewBusy) return;
  const img = $("cam-preview");
  if (!img) return;
  camPreviewBusy = true;
  try {
    const res = await fetch("/api/handeye/preview.jpg?t=" + Date.now(), { cache: "no-store" });
    if (!camPreviewWanted()) return; // 拉取中途若已收起，丢弃
    const msg = $("cam-msg");
    if (!res.ok) {
      let text = "双目没有画面";
      try { const data = await res.json(); if (data.error) text = data.error; } catch (e) { /* 非 JSON */ }
      if (msg) msg.textContent = text;
      return;
    }
    const blob = await res.blob();
    const url = URL.createObjectURL(blob);
    if (img.dataset.obj) URL.revokeObjectURL(img.dataset.obj);
    img.dataset.obj = url;
    img.src = url;
    if (msg) msg.textContent = "";
  } catch (e) {
    const msg = $("cam-msg");
    if (msg) msg.textContent = String(e);
  } finally {
    camPreviewBusy = false;
  }
}
setInterval(refreshCamPreview, 700);
document.addEventListener("visibilitychange", () => {
  if (!document.hidden && camPreviewWanted()) refreshCamPreview();
});
function syncRenderNote(extra) {
  const el = $("render-note");
  if (!el || !window.RobotView || !window.RobotView.cloudStatus) return;
  const st = window.RobotView.cloudStatus();
  const frame = st.frame === "scene" ? "场景" : "视觉";
  const stack = st.accumulate ? "固定" : "单帧";
  const dist = Number(st.maxDistM);
  const distTxt = Number.isFinite(dist) && dist > 0 ? `≤${dist.toFixed(1)} m` : "不限距";
  let text = `${frame} · ${stack} · ${distTxt} · ${st.layers} 层 · ${st.points}`;
  if (st.fullPoints && st.fullPoints !== st.points) text += `/${st.fullPoints}`;
  text += " 点";
  if (st.hasCompare) text += " · 含调试层";
  if (extra) text += ` · ${extra}`;
  el.textContent = text;
}

function applyRenderControlsFromUi() {
  if (!window.RobotView) return;
  const frame = $("render-frame")?.value || "vision";
  const stack = $("render-stack")?.value || "single";
  if (window.RobotView.setCloudFrame) window.RobotView.setCloudFrame(frame);
  if (window.RobotView.setCloudAccumulate) window.RobotView.setCloudAccumulate(stack === "fixed");
  const distEl = $("cloud-max-dist");
  if (distEl && window.RobotView.setCloudMaxDist) {
    window.RobotView.setCloudMaxDist(parseFloat(distEl.value) || 2);
  }
  syncRenderNote();
}

function syncCloudMaxDistLabel() {
  const sl = $("cloud-max-dist");
  const lab = $("cloud-max-dist-val");
  if (!sl || !lab) return;
  lab.textContent = Number(sl.value).toFixed(1);
}

if ($("cloud-max-dist")) {
  syncCloudMaxDistLabel();
  $("cloud-max-dist").addEventListener("input", () => {
    syncCloudMaxDistLabel();
    const m = parseFloat($("cloud-max-dist").value) || 2;
    if (window.RobotView && window.RobotView.setCloudMaxDist) {
      window.RobotView.setCloudMaxDist(m);
      syncRenderNote();
    }
  });
}

if ($("render-frame")) {
  $("render-frame").onchange = () => {
    applyRenderControlsFromUi();
    flash($("render-frame").value === "scene" ? "点云钉在场景" : "点云跟随躯干", true);
  };
}
if ($("render-stack")) {
  $("render-stack").onchange = () => {
    applyRenderControlsFromUi();
    flash($("render-stack").value === "fixed" ? "固定场景：匹配会叠加" : "单帧场景：匹配会替换", true);
  };
}
if ($("btn-cloud-clear")) {
  $("btn-cloud-clear").onclick = () => {
    if (!window.RobotView || !window.RobotView.clearClouds) {
      flash("三维还没准备好", false);
      return;
    }
    window.RobotView.clearClouds();
    syncRenderNote("已清除");
    flash("点云已清除（棋盘/红黄点保留）", true);
  };
}
if ($("btn-cloud-optimize")) {
  $("btn-cloud-optimize").onclick = () => {
    if (!window.RobotView || !window.RobotView.optimizeClouds) {
      flash("三维还没准备好", false);
      return;
    }
    const r = window.RobotView.optimizeClouds();
    if (!r.ok) { flash(r.error || "优化失败", false); return; }
    syncRenderNote(`滤除 ${r.dropped}，剩 ${r.kept}`);
    flash(`点云优化：去掉 ${r.dropped} 个噪点，剩 ${r.kept}`, true);
  };
}
window.addEventListener("robot-view-ready", () => {
  applyRenderControlsFromUi();
  syncRenderNote();
});

async function matchCloud(body, slot) {
  const msg = $("cam-match");
  if (msg) msg.textContent = slot === "compare" ? "正在用调试口的内外参匹配…" : "正在匹配…";
  try {
    applyRenderControlsFromUi();
    const d = await post("/api/handeye/match", body);
    const img = $("cam-disp");
    if (img && d.disparity_jpeg) img.src = "data:image/jpeg;base64," + d.disparity_jpeg;
    const drawn = d.cloud && window.RobotView && window.RobotView.setCloud(d.cloud.xyz, d.cloud.rgb, slot);
    const dep = d.depth_m || {};
    const st = window.RobotView && window.RobotView.cloudStatus ? window.RobotView.cloudStatus() : null;
    let text = `有效 ${d.valid} 点，画面上 ${d.saved} 点。深度中位 ${Number(dep.median).toFixed(2)} m。`
      + (drawn ? " 已画到中央模型上。" : " 三维画面还没准备好。");
    if (st) {
      text += st.accumulate ? ` 固定场景第 ${st.layers} 层。` : " 单帧已替换。";
      text += st.frame === "scene" ? " 挂接：场景。" : " 挂接：视觉。";
    }
    if (d.note) text += " " + d.note;
    if (msg) msg.textContent = text;
    syncRenderNote(drawn ? "刚匹配" : null);
  } catch (e) {
    if (msg) msg.textContent = String(e);
  }
}
$("btn-cam-match").onclick = () => matchCloud({}, "main");
$("btn-cam-match-debug").onclick = () => matchCloud({ source: "debug" }, "compare");
if ($("btn-render-match")) $("btn-render-match").onclick = () => matchCloud({}, "main");

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
/* —— 工作流：识别-末端到位 ——
 * next: 下一步要跑的索引 0..3；跑完后 finished=true，按钮变「完成」。
 * 「完成」只复位；「停止」仅在 busy 时可用，急停本卡进行中动作后复位。
 * 「自动继续」从当前 next 接着跑到结束。
 */
window.wfDetectGoto = {
  next: 0,
  finished: false,
  center_m: null,
  busy: false,
  abort: false,
};
const WF_DG_STEP_COUNT = 4;
const WF_DG_PHASES = ["待开始", "① 姿态已锁", "② 已识别", "③ 已抬到位", "已完成"];
const WF_DG_STEP_NAMES = ["① 工作姿态", "② 识别中心", "③ 偏置+提升", "④ 仅偏置"];

function wfDgReadOffset() {
  const n = (id, fallback = 0) => {
    const v = Number($(id)?.value);
    return Number.isFinite(v) ? v : fallback;
  };
  return {
    ox: n("wf-dg-ox"),
    oy: n("wf-dg-oy"),
    oz: n("wf-dg-oz"),
    hz: n("wf-dg-hz", 20),
  };
}

function wfDgSyncAimNote() {
  const el = $("wf-dg-aim-note");
  if (!el) return;
  const { ox, oy, oz, hz } = wfDgReadOffset();
  el.textContent =
    `第三步 Δ[${ox}, ${oy}, ${oz + hz}] = 偏置+提升 · 第四步 Δ[${ox}, ${oy}, ${oz}] = 仅偏置`;
  const c = window.wfDetectGoto.center_m;
  if (c && window.RobotView?.setSpatialAim) {
    window.RobotView.setSpatialAim([
      c[0] + ox / 1000,
      c[1] + oy / 1000,
      c[2] + oz / 1000,
    ]);
  }
}

function wfDgSetNote(text) {
  const el = $("wf-dg-note");
  if (el) el.textContent = text;
}

function wfDgSyncUi() {
  const st = window.wfDetectGoto;
  const btn = $("btn-wf-dg-step");
  const btnAll = $("btn-wf-dg-all");
  const btnStop = $("btn-wf-dg-stop");
  const badge = $("wf-dg-phase");
  if (btn) {
    if (st.finished) btn.textContent = "完成";
    else if (st.next === 0) btn.textContent = "开始";
    else btn.textContent = "下一步";
    btn.disabled = !!st.busy;
  }
  if (btnAll) btnAll.disabled = !!st.busy || !!st.finished;
  // 进行中可急停；已跑过至少一步（含已完成）也可点停止复位，不必硬点「下一步」
  if (btnStop) btnStop.disabled = !(st.busy || st.next > 0 || st.finished);
  if (badge) {
    if (st.finished) badge.textContent = WF_DG_PHASES[4];
    else badge.textContent = WF_DG_PHASES[Math.min(st.next, 3)];
  }
}

function wfDgResetToStart(noteText) {
  const st = window.wfDetectGoto;
  st.next = 0;
  st.finished = false;
  st.abort = false;
  st.center_m = null;
  if (window.RobotView?.setWorkflowPoint) window.RobotView.setWorkflowPoint(null);
  if (window.RobotView?.setSpatialAim) window.RobotView.setSpatialAim(null);
  wfDgSyncAimNote();
  wfDgSyncUi();
  if (noteText) wfDgSetNote(noteText);
}

function wfDgCheckAbort() {
  if (window.wfDetectGoto.abort) {
    const err = new Error("__wf_abort__");
    err.wfAbort = true;
    throw err;
  }
}

function wfDgCenterFromDetect(d) {
  if (Array.isArray(d?.center_m) && d.center_m.length >= 3) {
    return d.center_m.map(Number);
  }
  const xyz = d?.center_xyz;
  if (Array.isArray(xyz) && xyz.length >= 3) {
    return [Number(xyz[0]), Number(xyz[1]), Number(xyz[2])];
  }
  return null;
}

function wfDgFormatGoto(d, tag) {
  const plan = Number(d.plan_miss_mm ?? d.miss_mm);
  const meas = Number(d.measured_miss_mm);
  const nRef = Number(d.refine_passes) || 0;
  const before = Number(d.measured_before_mm);
  let text = `${tag}`;
  if (Array.isArray(d.offset_mm)) {
    text += ` Δ[${d.offset_mm.map((v) => Number(v).toFixed(1)).join(", ")}]`;
  }
  text += ` · 规划 ${Number.isFinite(plan) ? plan.toFixed(2) : "?"} mm`;
  if (Number.isFinite(meas)) {
    if (Number.isFinite(before) && nRef > 0) {
      text += ` · 修正前 ${before.toFixed(2)} → ${nRef} 步后 ${meas.toFixed(2)} mm`;
    } else {
      text += ` · 实测 ${meas.toFixed(2)} mm`;
    }
  }
  return text;
}

async function wfDgStepPose() {
  const name = ($("wf-dg-wp")?.value || "").trim();
  if (!name) throw new Error("先选择工作姿态");
  if (!window.lastTcpName) throw new Error("先调用一套 TCP");
  if (window.lastCtrl && window.lastCtrl.float) throw new Error("先点急停退出拖拽");
  if (window.lastCtrl && !window.lastCtrl.engaged) throw new Error("先接管再调用工作姿态");
  wfDgCheckAbort();
  wfDgSetNote("① 调用并锁定工作姿态…");
  const d = await workPosePost("/api/work-pose/load", {
    name,
    tcp_name: window.lastTcpName,
  });
  wfDgCheckAbort();
  if (!d.ok) throw new Error(d.error || "工作姿态失败");
  const text =
    d.message
    || `① 已锁定「${name}」· 位置 ${Number(d.measured_miss_mm || 0).toFixed(1)} mm · 姿态 ${Number(d.measured_ori_deg || 0).toFixed(1)}°`;
  wfDgSetNote(text);
  flash(text, true);
  return d;
}

async function wfDgStepDetect() {
  const params = ($("wf-dg-params")?.value || "").trim();
  const body = {};
  if (params) body.query = params;
  wfDgCheckAbort();
  wfDgSetNote("② 点位获取…");
  const p = await detectLocalize("/api/detect/points", body, "工作流：点位获取中…");
  wfDgCheckAbort();
  if (!p || !p.ok) throw new Error(p?.error || "点位获取失败");
  wfDgSetNote("② 校准…");
  const c = await detectLocalize("/api/detect/calibrate", {}, "工作流：校准中…");
  wfDgCheckAbort();
  if (!c || !c.ok) throw new Error(c?.error || "校准失败");
  wfDgSetNote("② 中心点提取…");
  const ctr = await detectLocalize("/api/detect/center", {}, "工作流：提取中心…");
  wfDgCheckAbort();
  if (!ctr || !ctr.ok) throw new Error(ctr?.error || "中心点提取失败");
  const center = wfDgCenterFromDetect(ctr);
  if (!center || center.some((v) => !Number.isFinite(v))) {
    throw new Error("没有得到有效中心点");
  }
  window.wfDetectGoto.center_m = center;
  if (window.RobotView?.setWorkflowPoint) window.RobotView.setWorkflowPoint(center);
  wfDgSyncAimNote();
  const mm = center.map((v) => (v * 1000).toFixed(1));
  const text = `② 完成 · 浅蓝临时点 [${mm.join(", ")}] mm`;
  wfDgSetNote(text);
  flash(text, true);
  return center;
}

async function wfDgStepGoto(kind) {
  const center = window.wfDetectGoto.center_m;
  if (!center) throw new Error("还没有识别中心，先跑识别步骤");
  if (window.lastCtrl && window.lastCtrl.float) throw new Error("先点急停退出拖拽");
  const { ox, oy, oz, hz } = wfDgReadOffset();
  const offset_mm = kind === "lift" ? [ox, oy, oz + hz] : [ox, oy, oz];
  const refine = !($("wf-dg-refine") && !$("wf-dg-refine").checked);
  const tag = kind === "lift" ? "③ 偏置+提升" : "④ 仅偏置";
  wfDgCheckAbort();
  wfDgSetNote(`${tag} 走点中…`);
  const d = await post("/api/points/goto-xyz", {
    point_m: center,
    offset_mm,
    refine,
  });
  wfDgCheckAbort();
  if (!d.ok) throw new Error(d.error || `${tag} 失败`);
  const text = wfDgFormatGoto(d, tag);
  wfDgSetNote(text);
  flash(text, true);
  return d;
}

async function wfDgRunOne(index) {
  if (index === 0) return wfDgStepPose();
  if (index === 1) return wfDgStepDetect();
  if (index === 2) return wfDgStepGoto("lift");
  if (index === 3) return wfDgStepGoto("final");
  throw new Error("没有这一步");
}

function wfDgMarkAdvance() {
  const st = window.wfDetectGoto;
  st.next += 1;
  if (st.next >= WF_DG_STEP_COUNT) {
    st.next = WF_DG_STEP_COUNT;
    st.finished = true;
  }
  wfDgSyncUi();
}

function wfDgIsAbortError(e) {
  if (window.wfDetectGoto.abort) return true;
  if (!e) return false;
  if (e.wfAbort || e.message === "__wf_abort__") return true;
  const msg = String(e.message || e);
  return msg.includes("已停止") || msg.includes("到位前已释放");
}

async function wfDgWithBusy(runner) {
  if (window.wfDetectGoto.busy) return;
  window.wfDetectGoto.busy = true;
  window.wfDetectGoto.abort = false;
  wfDgSyncUi();
  try {
    await runner();
  } catch (e) {
    if (wfDgIsAbortError(e)) {
      wfDgResetToStart("已停止并复位");
      flash("工作流已停止", true);
      return;
    }
    flash(String(e.message || e), false);
    wfDgSetNote(String(e.message || e));
  } finally {
    window.wfDetectGoto.busy = false;
    wfDgSyncUi();
  }
}

async function wfDgRunStep() {
  const st = window.wfDetectGoto;
  if (st.busy) return;
  if (st.finished) {
    wfDgResetToStart("已复位，可重新开始");
    flash("已复位", true);
    return;
  }
  const index = st.next;
  if (index === 0) {
    const name = ($("wf-dg-wp")?.value || "").trim();
    if (!name) { flash("先选择工作姿态", false); return; }
    if (!confirm(`第一步：调用并锁定「${name}」。扶住、守急停。继续？`)) return;
  } else if (index === 2) {
    const { ox, oy, oz, hz } = wfDgReadOffset();
    if (!confirm(`第三步：TCP → 中心 + 偏置+提升 Δ[${ox}, ${oy}, ${oz + hz}] mm。继续？`)) return;
  } else if (index === 3) {
    const { ox, oy, oz } = wfDgReadOffset();
    if (!confirm(`第四步：TCP → 中心 + 偏置 Δ[${ox}, ${oy}, ${oz}] mm。继续？`)) return;
  }
  await wfDgWithBusy(async () => {
    await wfDgRunOne(index);
    wfDgCheckAbort();
    wfDgMarkAdvance();
    if (window.wfDetectGoto.finished) {
      flash("工作流完成，点「完成」可复位", true);
    }
  });
}

async function wfDgRunAuto() {
  const st = window.wfDetectGoto;
  if (st.busy || st.finished) return;
  const from = st.next;
  if (from >= WF_DG_STEP_COUNT) return;
  if (from === 0) {
    const name = ($("wf-dg-wp")?.value || "").trim();
    if (!name) { flash("先选择工作姿态", false); return; }
  }
  const remain = WF_DG_STEP_NAMES.slice(from).join(" → ");
  if (!confirm(`自动继续（从当前进度）：${remain}。扶住、守急停。继续？`)) return;
  await wfDgWithBusy(async () => {
    while (window.wfDetectGoto.next < WF_DG_STEP_COUNT && !window.wfDetectGoto.finished) {
      wfDgCheckAbort();
      const i = window.wfDetectGoto.next;
      await wfDgRunOne(i);
      wfDgCheckAbort();
      wfDgMarkAdvance();
    }
    if (window.wfDetectGoto.finished) {
      wfDgSetNote(($("wf-dg-note")?.textContent || "") + " · 自动继续结束");
      flash("自动继续完成，点「完成」可复位", true);
    }
  });
}

async function wfDgStop() {
  const st = window.wfDetectGoto;
  if (!(st.busy || st.next > 0 || st.finished)) return;
  if (st.busy) {
    st.abort = true;
    wfDgSetNote("正在停止…");
    try {
      await post("/api/hold", {});
    } catch (e) {
      /* 急停请求失败也继续走前端复位；busy 结束时会按 abort 复位 */
    }
    return;
  }
  // 已停在某步之间 / 已完成：直接复位，等同「完成」
  wfDgResetToStart("已停止并复位");
  flash("工作流已停止", true);
}

if ($("btn-wf-dg-step")) {
  wfDgSyncUi();
  wfDgSyncAimNote();
  ["wf-dg-ox", "wf-dg-oy", "wf-dg-oz", "wf-dg-hz"].forEach((id) => {
    if ($(id)) $(id).addEventListener("input", wfDgSyncAimNote);
  });
  $("btn-wf-dg-step").onclick = () => wfDgRunStep();
  if ($("btn-wf-dg-all")) $("btn-wf-dg-all").onclick = () => wfDgRunAuto();
  if ($("btn-wf-dg-stop")) $("btn-wf-dg-stop").onclick = () => wfDgStop();
}

$("btn-he-run").onclick = async () => {
  if (!posesetName()) { flash("先在姿态集里选一套已经记好的点", false); return; }
  const arm = window.currentSet && window.currentSet.arm;
  const side = arm === "left" ? "左臂" : arm === "right" ? "右臂" : "这一套";
  if (!confirm(`将按当前限速走完「${posesetName()}」（${side}），每个点停稳后存图。扶住、守急停。继续？`)) return;
  try {
    const d = await post("/api/handeye/run", { name: posesetName(), ...heForm() });
    applyHeRun(d);
    flash("开始按姿态集采集", true);
  } catch (e) { flash(String(e), false); }
};
$("btn-he-stop").onclick = async () => {
  try { await post("/api/handeye/stop", {}); flash("已请求停止", true); }
  catch (e) { flash(String(e), false); }
};
$("btn-he-capture").onclick = async () => {
  try {
    const d = await post("/api/handeye/capture", heForm());
    renderHe(d);
    flash(`已存 ${d.session} / ${String(d.index).padStart(4, "0")}`, true);
  } catch (e) { flash(String(e), false); }
};
$("he-session").onchange = async () => {
  const name = $("he-session").value;
  if (!name) return;
  try { renderHe(await post("/api/handeye/session", { name })); }
  catch (e) { flash(String(e), false); }
};
$("btn-he-solve").onclick = async () => {
  const f = heForm();
  if (!f.session) { flash("先选一次采集会话", false); return; }
  if (!f.square_size_mm) { flash("填棋盘格子边长（毫米）", false); return; }
  try {
    flash("先估内参，再做手眼…", true);
    const d = await post("/api/handeye/solve", f);
    renderHe(d);
    flash("求解完成，结果写在该会话目录", true);
  } catch (e) { flash(String(e), false); }
};
$("btn-he-save").onclick = async () => {
  const name = $("he-calib-name").value.trim();
  if (!name) { flash("填一个名字", false); return; }
  const exists = (window.lastHeSaved || []).some((s) => s.name === name);
  if (exists && !confirm(`覆盖已保存的「${name}」？`)) return;
  try {
    const d = await post("/api/handeye/save", { name, session: $("he-session").value });
    $("he-calib-name").value = "";
    renderHe(d);
    if (d.runtime) renderRuntimeConfig(d.runtime);
    flash(`已保存并应用「${name}」`, true);
  } catch (e) { flash(String(e), false); }
};
} catch (e) {
  const m = document.getElementById("msg");
  if (m) { m.className = "err"; m.textContent = "页面脚本停在: " + e; }
}

