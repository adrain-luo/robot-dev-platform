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
    const c = this.ctx, cv = this.canvas, w = cv.clientWidth, h = cv.clientHeight;
    if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
    c.clearRect(0, 0, w, h);
    const g = c.createRadialGradient(w * 0.55, h * 0.42, 30, w * 0.5, h * 0.55, Math.max(w, h) * 0.72);
    g.addColorStop(0, "#1a2330");
    g.addColorStop(1, "#0c1016");
    c.fillStyle = g;
    c.fillRect(0, 0, w, h);
    for (let x = -1; x <= 1.001; x += 0.25) {
      this.line([x, -1, -0.6], [x, 1, -0.6], "#243044");
      this.line([-1, x, -0.6], [1, x, -0.6], "#243044");
    }
    this.line([0, 0, 0], [0.3, 0, 0], "#e5484d", 2);
    this.line([0, 0, 0], [0, 0.3, 0], "#46a758", 2);
    this.line([0, 0, 0], [0, 0, 0.3], "#3e63dd", 2);
    c.fillStyle = "#c5ced8"; c.font = "12px system-ui";
    const ax = this.project([0.33, 0, 0]); if (ax) c.fillText("X 前", ax[0], ax[1]);
    const ay = this.project([0, 0.33, 0]); if (ay) c.fillText("Y 左", ay[0], ay[1]);
    const az = this.project([0, 0, 0.33]); if (az) c.fillText("Z 上", az[0], az[1]);
    this.line([0, 0, 0.35], [0, 0, -0.5], "#3a3f48", 6);
    this.line([0, -0.2, 0.3], [0, 0.2, 0.3], "#3a3f48", 6);
    const sc = this.scene; if (!sc) return;
    const show = $("arm-select").value;
    if (show !== "left") this.drawArm(sc.right, "#f4efe6", "右腕");
    if (show !== "right") this.drawArm(sc.left, "#7dcec4", "左腕");
  },
  init() {
    this.canvas = $("scene3d"); this.ctx = this.canvas.getContext("2d");
    const cv = this.canvas;
    cv.addEventListener("mousedown", (e) => {
      this.drag = { x: e.clientX, y: e.clientY, pan: e.button === 2 || e.shiftKey };
      e.preventDefault();
    });
    window.addEventListener("mouseup", () => { this.drag = null; });
    window.addEventListener("mousemove", (e) => {
      if (!this.drag) return;
      const dx = e.clientX - this.drag.x, dy = e.clientY - this.drag.y;
      this.drag.x = e.clientX; this.drag.y = e.clientY;
      if (this.drag.pan) {
        const cy = Math.cos(this.yaw), sy = Math.sin(this.yaw), k = this.dist * 0.0015;
        this.target = add3(add3(this.target, scale3([-sy, cy, 0], -dx * k)), [0, 0, dy * k]);
      } else {
        this.yaw -= dx * 0.008;
        this.pitch = Math.max(-1.5, Math.min(1.5, this.pitch + dy * 0.008));
      }
      this.draw();
    });
    cv.addEventListener("wheel", (e) => {
      this.dist = Math.max(0.4, Math.min(8, this.dist * (e.deltaY > 0 ? 1.1 : 0.9)));
      this.draw(); e.preventDefault();
    }, { passive: false });
    cv.addEventListener("contextmenu", (e) => e.preventDefault());
    cv.addEventListener("dblclick", () => this.reset());
    new ResizeObserver(() => this.draw()).observe(cv);
    this.draw();
  },
};
window.V3 = V3;

function fmt(v, n = 2) {
  if (v == null || Number.isNaN(v)) return "—";
  return Number(v).toFixed(n);
}

function renderTable(snap) {
  const tb = $("joints-table").querySelector("tbody");
  tb.innerHTML = "";
  if (!snap) return;
  for (const j of snap.joints) {
    const tr = document.createElement("tr");
    const live = Math.abs(j.dq) > 0.02;
    tr.innerHTML = `<td class="j-name">${j.label}</td><td class="j-q">${fmt(j.q_deg)}°</td>` +
      `<td class="j-sub">电机 ${j.motor}</td>` +
      `<td class="j-sub ${live ? "dq-live" : ""}">dq ${fmt(j.dq, 3)}</td><td class="j-sub">τ ${fmt(j.tau_est)}</td>`;
    tb.appendChild(tr);
  }
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

const LABELS = ["sh_pitch", "sh_roll", "sh_yaw", "elbow", "wr_roll", "wr_pitch", "wr_yaw"];
function ensureGotoInputs() {
  const row = $("goto-inputs");
  if (row.querySelector("input")) return;
  LABELS.forEach((lab) => {
    const label = document.createElement("label");
    label.className = "field";
    label.innerHTML = `<span>${lab}</span><input class="g-q" type="number" step="0.01">`;
    row.appendChild(label);
  });
}

function fillQ(q) {
  ensureGotoInputs();
  [...document.querySelectorAll(".g-q")].forEach((el, i) => { el.value = Number(q[i]).toFixed(3); });
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
  ["btn-engage", "btn-release", "btn-hold", "btn-teach", "btn-speed", "btn-goto", "btn-fill", "control-arm", "speed", "grav-on", "btn-id-measure", "btn-id-apply", "btn-id-clear", "btn-payload-use"].forEach((id) => {
    const el = $(id); if (el) el.disabled = !allow;
  });
  const mirror = $("btn-poseset-mirror");
  if (mirror) mirror.disabled = !window.currentSet;
  if ($("btn-teach")) $("btn-teach").disabled = !allow || !ctrl.engaged;
  $("btn-teach").textContent = ctrl.float ? "结束拖拽" : "拖拽";
  $("grav-on").checked = ctrl.gravity_ff !== false;
  $("engage-state").textContent = allow
    ? (ctrl.engaged
        ? `${ctrl.float ? "拖拽中" : "已接管"} · ${ctrl.arm}${ctrl.waist_locked ? " · 腰回零锁" : ""}${
            ctrl.payload_mass_kg != null ? ` · 载 ${Number(ctrl.payload_mass_kg).toFixed(2)} kg${ctrl.payload_name ? "(" + ctrl.payload_name + ")" : ""}` : ""
          }`
        : "未接管")
    : "只读（加 --allow-real-motion 后可接管）";
  document.querySelectorAll(".g-q").forEach((el) => { el.disabled = !allow || !!ctrl.float; });
  $("btn-goto").disabled = !allow || !ctrl.engaged || !!ctrl.float;
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
    if (!data.ok) {
      flash(data.error || "读取失败", false);
      setPill(false, "lowstate 失败");
      return;
    }
    const right = data.arms.right, left = data.arms.left;
    const age = Math.min(right?.age_s ?? 99, left?.age_s ?? 99);
    const fresh = age < 0.5;
    setPill(fresh, fresh ? `lowstate OK  ${age.toFixed(3)} s` : `信号陈旧  ${age.toFixed(2)} s`, fresh ? "ok" : "warn");
    $("meta").textContent = `${data.iface} · ${data.topic} · mode_machine=${right?.mode_machine ?? "—"}`;
    applyCtrl(data.ctrl);
    if (typeof applyIdentify === "function") applyIdentify(data.identify);
    const modeLabel = data.ctrl?.float ? "实测 · 拖拽卸力" : (data.ctrl?.engaged ? "实测 · 已接管" : (data.mode === "real" ? "实测 · 未接管" : "实测 · 只读"));
    if (window.currentSet && window.currentSet.poses) {
      const tips = window.currentSet.poses.map((p) => p.tip_xyz).filter(Boolean);
      const armSc = window.currentSet.arm === "left" ? left?.scene : right?.scene;
      if (armSc) armSc.pose_tips = tips;
    }
    V3.scene = {
      right: right?.scene,
      left: left?.scene,
      modeLabel,
    };
    V3.draw();
    const view = $("arm-select").value;
    const ctrlArm = data.ctrl?.arm || "right";
    const tableArm = view === "left" ? left : view === "right" ? right : (ctrlArm === "left" ? left : right);
    renderTable(tableArm);
  } catch (e) {
    flash(String(e), false);
    setPill(false, "服务断开");
  } finally {
    inflight = false;
  }
}

ensureGotoInputs();
V3.init();
setInterval(poll, 100);
poll();
