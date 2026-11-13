// 00-util.js —— 公共工具：向量运算、三维视图辅助 V3、数值格式、提示条、POST 封装、下拉框填充
// 由 web/static/js/ 下各文件按文件名顺序拼成 /static/app.js（同一个脚本作用域）。

// 加载期间脚本出错：在页面顶部提示（99-main.js 末尾撤掉，之后的错误照常进控制台）
function showLoadError(e) {
  const m = document.getElementById("msg");
  if (m) { m.className = "err"; m.textContent = "页面脚本停在: " + (e.error || e.message || e); }
}
window.addEventListener("error", showLoadError);

const $ = (id) => document.getElementById(id);
const escapeHtml = (s) => String(s).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));

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

function writeOptions(sel, placeholder, rows, keep) {
  if (!sel) return;
  const prev = keep != null ? keep : sel.value;
  while (sel.options.length) sel.remove(0);
  sel.add(new Option(placeholder, ""));
  for (const row of rows) sel.add(new Option(row.label, row.value));
  if (prev && [...sel.options].some((o) => o.value === prev)) sel.value = prev;
}

// ---------------------------------------------------------------- 选手（左 / 右）
// 两只手都在控制循环里，没有全局「控制臂」：需要指定手的卡片各放一个
// <span class="arm-pick" data-pick="键"></span>，选择按键存在浏览器里（只是本机的偏好）。
const ARM_PICK_STORE = "rdp.armPick.";
const ARM_SIDE = { left: "左", right: "右" };

function armPick(key) {
  const el = document.querySelector(`.arm-pick[data-pick="${key}"]`);
  return (el && el.dataset.arm) || "right";
}

function setArmPick(key, arm) {
  const el = document.querySelector(`.arm-pick[data-pick="${key}"]`);
  if (!el || (arm !== "left" && arm !== "right")) return;
  el.dataset.arm = arm;
  el.querySelectorAll("button").forEach((b) => {
    const on = b.dataset.arm === arm;
    b.classList.toggle("active", on);
    b.setAttribute("aria-pressed", on ? "true" : "false");
  });
  try { localStorage.setItem(ARM_PICK_STORE + key, arm); } catch (_) { /* 无痕模式等：不记 */ }
}

function initArmPicks() {
  document.querySelectorAll(".arm-pick").forEach((el) => {
    const key = el.dataset.pick;
    if (el.querySelector("button")) return;
    for (const arm of ["left", "right"]) {
      const b = document.createElement("button");
      b.type = "button";
      b.dataset.arm = arm;
      b.textContent = ARM_SIDE[arm];
      b.title = `${ARM_SIDE[arm]}臂`;
      b.addEventListener("click", (ev) => {
        ev.stopPropagation();
        setArmPick(key, arm);
        el.dispatchEvent(new CustomEvent("arm-pick", { bubbles: true, detail: { key, arm } }));
      });
      el.appendChild(b);
    }
    let saved = null;
    try { saved = localStorage.getItem(ARM_PICK_STORE + key); } catch (_) { saved = null; }
    setArmPick(key, saved === "left" ? "left" : "right");
  });
}

// 这只手现在能不能接受新目标：已接管、不在拖拽、没被任务占用
function armBusyReason(arm) {
  const ctrl = window.lastCtrl;
  const side = ARM_SIDE[arm] || arm;
  if (!ctrl || !ctrl.engaged) return "先接管";
  const a = (ctrl.arms || {})[arm] || {};
  if (a.float) return `${side}臂在拖拽，先退出拖拽`;
  if (a.owner) return `${side}臂正被「${a.owner}」使用`;
  return "";
}

// 同上，但先取一次最新状态再判断。连续几步的流程用：上一步刚结束时，轮询缓存里可能还是
// 上一步占着手的旧状态（例：③ 碰点刚返回，缓存仍显示「空间点碰点」，④ 会被误拦）
async function armBusyReasonFresh(arm) {
  try {
    const res = await fetch("/api/status", { cache: "no-store" });
    const d = await res.json();
    if (d && d.ctrl && typeof applyCtrl === "function") applyCtrl(d.ctrl);
  } catch (e) {
    /* 取不到就按缓存判断，后端走点时还会再查一次占用 */
  }
  return armBusyReason(arm);
}
