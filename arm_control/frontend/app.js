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
    const T = sc.wrist_T;
    if (T) {
      const o = [T[0][3], T[1][3], T[2][3]], L = 0.06;
      this.line(o, add3(o, [T[0][0] * L, T[1][0] * L, T[2][0] * L]), "#e5484d", 1.5);
      this.line(o, add3(o, [T[0][1] * L, T[1][1] * L, T[2][1] * L]), "#46a758", 1.5);
      this.line(o, add3(o, [T[0][2] * L, T[1][2] * L, T[2][2] * L]), "#3e63dd", 1.5);
      this.dot(o, color, 4, label);
    }
  },
  draw() {
    const c = this.ctx, cv = this.canvas, w = cv.clientWidth, h = cv.clientHeight;
    if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
    c.clearRect(0, 0, w, h);
    for (let g = -1; g <= 1.001; g += 0.25) {
      this.line([g, -1, -0.6], [g, 1, -0.6], "#1e2229");
      this.line([-1, g, -0.6], [1, g, -0.6], "#1e2229");
    }
    this.line([0, 0, 0], [0.3, 0, 0], "#e5484d", 2);
    this.line([0, 0, 0], [0, 0.3, 0], "#46a758", 2);
    this.line([0, 0, 0], [0, 0, 0.3], "#3e63dd", 2);
    c.fillStyle = "#9aa3ad"; c.font = "11px system-ui";
    const ax = this.project([0.33, 0, 0]); if (ax) c.fillText("X 前", ax[0], ax[1]);
    const ay = this.project([0, 0.33, 0]); if (ay) c.fillText("Y 左", ay[0], ay[1]);
    const az = this.project([0, 0, 0.33]); if (az) c.fillText("Z 上", az[0], az[1]);
    this.line([0, 0, 0.35], [0, 0, -0.5], "#3a3f48", 6);
    this.line([0, -0.2, 0.3], [0, 0.2, 0.3], "#3a3f48", 6);
    const sc = this.scene; if (!sc) return;
    const show = $("arm-select").value;
    if (show !== "left") this.drawArm(sc.right, "#e8eaed", "右腕");
    if (show !== "right") this.drawArm(sc.left, "#67e8f9", "左腕");
    c.font = "12px system-ui"; c.fillStyle = "#9aa3ad";
    c.fillText(sc.modeLabel || "torso_link", 8, 16);
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
    tr.innerHTML = `<td>${j.label}</td><td>${j.motor}</td><td>${fmt(j.q_deg)}</td>` +
      `<td class="${live ? "dq-live" : ""}">${fmt(j.dq, 3)}</td><td>${fmt(j.tau_est)}</td>`;
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
    label.innerHTML = `${lab} <input class="g-q" type="number" step="0.01">`;
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

function applyCtrl(ctrl) {
  if (!ctrl) return;
  const allow = !!ctrl.allow_real_motion;
  $("mode-badge").textContent = allow ? "可运动" : "只读";
  $("mode-badge").className = "badge " + (allow ? "real" : "ro");
  ["btn-engage", "btn-release", "btn-hold", "btn-teach", "btn-speed", "btn-goto", "btn-fill", "control-arm", "speed", "grav-on"].forEach((id) => {
    const el = $(id); if (el) el.disabled = !allow;
  });
  if ($("btn-teach")) $("btn-teach").disabled = !allow || !ctrl.engaged;
  $("btn-teach").textContent = ctrl.float ? "结束拖拽" : "拖拽";
  $("grav-on").checked = ctrl.gravity_ff !== false;
  $("engage-state").textContent = allow
    ? (ctrl.engaged ? (ctrl.float ? `拖拽中 · ${ctrl.arm}` : `已接管 · ${ctrl.arm}`) : "未接管")
    : "只读（加 --allow-real-motion 后可接管）";
  document.querySelectorAll(".g-q").forEach((el) => { el.disabled = !allow || !!ctrl.float; });
  $("btn-goto").disabled = !allow || !ctrl.engaged || !!ctrl.float;
  if (ctrl.max_speed_rad_s) $("speed").value = ctrl.max_speed_rad_s;
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
    const modeLabel = data.ctrl?.float ? "实测 · 拖拽卸力" : (data.ctrl?.engaged ? "实测 · 已接管" : (data.mode === "real" ? "实测 · 未接管" : "实测 · 只读"));
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
$("arm-select").onchange = () => { poll(); V3.draw(); };
$("btn-fill").onclick = () => {
  const view = $("arm-select").value === "left" ? "left" : $("control-arm").value;
  fetch("/api/status?arm=" + view).then((r) => r.json()).then((d) => {
    if (d.ok) fillQ(d.joints.map((j) => j.q_rad));
  });
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
$("btn-speed").onclick = async () => {
  try {
    await post("/api/speed", { max_speed_rad_s: parseFloat($("speed").value) });
    flash("已设限速", true);
  } catch (e) { flash(String(e), false); }
};
$("btn-goto").onclick = async () => {
  const q = [...document.querySelectorAll(".g-q")].map((el) => parseFloat(el.value));
  if (q.some((x) => !Number.isFinite(x))) { flash("目标角不完整", false); return; }
  if (!confirm("将限速走到目标关节角。扶住、守急停。继续？")) return;
  try { await post("/api/target", { q }); flash("已设目标", true); } catch (e) { flash(String(e), false); }
};
$("control-arm").onchange = async () => {
  try { await post("/api/control-arm", { arm: $("control-arm").value }); flash("控制臂已切换", true); }
  catch (e) { flash(String(e), false); }
};
setInterval(poll, 100);
poll();
