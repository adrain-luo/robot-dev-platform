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
    const hud = (show !== "left" && sc.right?.payload_mass) ? sc.right
      : (show !== "right" && sc.left?.payload_mass ? sc.left : null);
    if (hud && hud.payload_mass > 1e-6) {
      const com = (hud.payload_com_m || []).map((v) => Number(v).toFixed(3)).join(", ");
      const nm = hud.payload_name ? `「${hud.payload_name}」 ` : "";
      c.fillStyle = "#e879f9";
      c.fillText(`末端负载 ${nm}${Number(hud.payload_mass).toFixed(3)} kg  ·  质心(腕连杆) [${com}] m`, 8, 34);
    }
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

let currentSet = null;
let lastCtrl = null;
let speedDirty = false;
let stillDirty = false;

function applyCtrl(ctrl) {
  if (!ctrl) return;
  lastCtrl = ctrl;
  const allow = !!ctrl.allow_real_motion;
  $("mode-badge").textContent = allow ? "可运动" : "只读";
  $("mode-badge").className = "badge " + (allow ? "real" : "ro");
  ["btn-engage", "btn-release", "btn-hold", "btn-teach", "btn-speed", "btn-goto", "btn-fill", "control-arm", "speed", "grav-on", "btn-id-measure", "btn-id-apply", "btn-id-clear", "btn-payload-use"].forEach((id) => {
    const el = $(id); if (el) el.disabled = !allow;
  });
  $("btn-poseset-mirror").disabled = !currentSet;
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
  if (ctrl.max_speed_rad_s != null && document.activeElement !== sp && !speedDirty) {
    const v = Number(ctrl.max_speed_rad_s);
    if (Number(sp.value) !== v) sp.value = v;
  }
  if (ctrl.arm) $("control-arm").value = ctrl.arm;
}

function applyIdentify(id) {
  if (!id) return;
  const [a, b] = id.progress || [0, 0];
  $("id-progress").textContent = id.running ? `${id.phase || ""} ${a}/${b}` : (id.session ? `会话 ${id.session}` : "");
  $("id-log").textContent = (id.log || []).join("\n");
  const sol = id.solution;
  if (sol) {
    const com = (sol.com_m || []).map((x) => Number(x).toFixed(3)).join(", ");
    $("id-result").textContent = `m=${Number(sol.mass_kg).toFixed(3)} kg  com=[${com}] m  cond=${Number(sol.regressor_cond).toFixed(1)}  残差 ${Number(sol.residual_rms_before_nm).toFixed(2)}→${Number(sol.residual_rms_after_nm).toFixed(2)} N·m  估下垂 ${Number(sol.sag_est_before_deg).toFixed(2)}°→${Number(sol.sag_est_after_deg).toFixed(2)}°`;
    const files = [];
    if (id.session_dir) files.push(`会话 ${id.session_dir}`);
    if (id.payload_file) files.push(`前馈 ${id.payload_file}`);
    if (files.length) $("id-result").textContent += "  ·  " + files.join("  ·  ");
  } else if (id.error) {
    $("id-result").textContent = id.error;
  }
  const allow = !!lastCtrl?.allow_real_motion && !!lastCtrl?.engaged && !lastCtrl?.float;
  $("btn-id-measure").disabled = !allow || !!id.running;
  $("btn-id-stop").disabled = !id.running;
  renderPayloadSelect(id.profiles || [], id.applied_name);
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
    applyIdentify(data.identify);
    const modeLabel = data.ctrl?.float ? "实测 · 拖拽卸力" : (data.ctrl?.engaged ? "实测 · 已接管" : (data.mode === "real" ? "实测 · 未接管" : "实测 · 只读"));
    if (currentSet && currentSet.poses) {
      const tips = currentSet.poses.map((p) => p.tip_xyz).filter(Boolean);
      const armSc = currentSet.arm === "left" ? left?.scene : right?.scene;
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

function posesetName() {
  return $("poseset-select").value;
}

function renderPoseList(list) {
  const sel = $("poseset-select");
  const keep = posesetName();
  sel.innerHTML = "";
  const opt0 = document.createElement("option");
  opt0.value = "";
  opt0.textContent = list.length ? "选择姿态集…" : "还没有姿态集";
  sel.appendChild(opt0);
  for (const s of list) {
    const o = document.createElement("option");
    o.value = s.name;
    const side = s.arm === "left" ? "左" : "右";
    o.textContent = `${s.name} · ${side} · ${s.n}点` + (s.cond != null ? ` · cond ${s.cond}` : "");
    sel.appendChild(o);
  }
  if (keep && [...sel.options].some((o) => o.value === keep)) sel.value = keep;
}

function renderChips() {
  const box = $("pose-chips");
  box.innerHTML = "";
  $("pose-cond").textContent = "";
  if (!currentSet) return;
  const cond = currentSet.regressor_cond;
  $("pose-cond").textContent = currentSet.arm === "left" ? "左臂" : "右臂"
    + (cond != null ? ` · 回归 cond ${cond}` : "")
    + (currentSet.mirrored_from ? ` · 镜像自 ${currentSet.mirrored_from.name}` : "");
  (currentSet.poses || []).forEach((p, i) => {
    const wrap = document.createElement("span");
    wrap.style.display = "inline-flex";
    wrap.style.gap = "0";
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
  renderPoseList(d.list || []);
  if (selectName) $("poseset-select").value = selectName;
  if (posesetName()) await loadCurrentSet();
  else { currentSet = null; renderChips(); }
}

async function loadCurrentSet() {
  const name = posesetName();
  if (!name) { currentSet = null; renderChips(); return; }
  const d = await fetch("/api/posesets?name=" + encodeURIComponent(name)).then((r) => r.json());
  if (!d.ok) throw new Error(d.error);
  currentSet = d.set;
  renderChips();
}

async function gotoPose(i) {
  if (!currentSet) return;
  fillQ(currentSet.poses[i].q);
  const ctrl = lastCtrl;
  if (!ctrl?.allow_real_motion || !ctrl.engaged || ctrl.float) {
    flash("已填入该姿态角（需接管且非拖拽才能走过去）", true);
    return;
  }
  if (ctrl.arm !== currentSet.arm) {
    flash(`姿态集是${currentSet.arm === "left" ? "左" : "右"}臂，请先切控制臂`, false);
    return;
  }
  try {
    await post("/api/posesets/goto", { name: currentSet.name, index: i });
    flash(`正在去 ${currentSet.poses[i].name}`, true);
  } catch (e) { flash(String(e), false); }
}

async function deletePose(i) {
  if (!currentSet) return;
  if (!confirm(`删除 ${currentSet.poses[i].name}？`)) return;
  try {
    const d = await post("/api/posesets/delete-pose", { name: currentSet.name, index: i });
    currentSet = d.set;
    renderChips();
    await refreshPoseList(currentSet.name);
  } catch (e) { flash(String(e), false); }
}

async function recordPose() {
  if (!posesetName()) { flash("先新建或选择姿态集", false); return; }
  try {
    const d = await post("/api/posesets/record", { name: posesetName() });
    currentSet = d.set;
    renderChips();
    await refreshPoseList(currentSet.name);
    flash(`已记录 ${d.pose.name}（${currentSet.arm === "left" ? "左" : "右"}臂实测）`, true);
  } catch (e) { flash(String(e), false); }
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
    speedDirty = false;
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
    currentSet = null;
    await refreshPoseList();
    flash("已删集", true);
  } catch (e) { flash(String(e), false); }
};
$("btn-poseset-record").onclick = () => recordPose();
$("btn-poseset-mirror").onclick = async () => {
  if (!currentSet) { flash("先选姿态集", false); return; }
  const dst = currentSet.arm === "left" ? "右" : "左";
  if (!confirm(`把「${currentSet.name}」关于 y=0 镜像成${dst}臂姿态集？关节角会改符号，不是原样复制。`)) return;
  try {
    const d = await post("/api/posesets/mirror", { src: currentSet.name });
    await refreshPoseList(d.set.name);
    flash(`已镜像为「${d.set.name}」${dst}臂 ${d.set.poses.length} 点`, true);
  } catch (e) {
    if (String(e).includes("已存在") && confirm("对侧集已存在，覆盖？")) {
      try {
        const d = await post("/api/posesets/mirror", { src: currentSet.name, force: true });
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
refreshPoseList().catch(() => {});
$("btn-id-measure").onclick = async () => {
  if (!posesetName()) { flash("先选姿态集", false); return; }
  if (!confirm("将按当前限速走完姿态集（双向逼近）。扶住、守急停。继续？")) return;
  try {
    await post("/api/identify/measure", { name: posesetName() });
    flash("开始测量", true);
  } catch (e) { flash(String(e), false); }
};
$("btn-id-stop").onclick = async () => {
  try { await post("/api/identify/stop", {}); flash("已请求停止测量", true); }
  catch (e) { flash(String(e), false); }
};
$("btn-id-solve").onclick = async () => {
  try {
    const d = await post("/api/identify/solve", { source: $("id-source").value });
    applyIdentify(d.identify);
    flash("求解完成", true);
  } catch (e) { flash(String(e), false); }
};
function renderPayloadSelect(list, applied) {
  const sel = $("payload-select");
  if (!sel) return;
  if (document.activeElement === sel) return;
  const keep = sel.value;
  sel.innerHTML = "";
  const o0 = document.createElement("option");
  o0.value = "";
  o0.textContent = list.length ? "选择补偿文件…" : "还没有补偿文件";
  sel.appendChild(o0);
  for (const p of list) {
    const o = document.createElement("option");
    o.value = p.name;
    const side = p.arm === "left" ? "左" : (p.arm === "right" ? "右" : "?");
    const m = p.mass_kg != null ? `${Number(p.mass_kg).toFixed(2)}kg` : "";
    o.textContent = `${p.name} · ${side} ${m}`;
    sel.appendChild(o);
  }
  const want = applied || keep;
  if (want && [...sel.options].some((o) => o.value === want)) sel.value = want;
}

$("btn-id-apply").onclick = async () => {
  try {
    const name = ($("payload-new").value.trim() || $("payload-select").value || (currentSet && currentSet.name) || "").trim();
    const d = await post("/api/identify/apply", { name });
    applyIdentify(d.identify);
    flash(`已存并应用「${d.payload.name}」`, true);
  } catch (e) { flash(String(e), false); }
};
$("btn-id-clear").onclick = async () => {
  try {
    await post("/api/identify/clear", {});
    flash("已去掉负载项，只剩 URDF 臂自重", true);
  } catch (e) { flash(String(e), false); }
};
$("btn-payload-use").onclick = async () => {
  const name = $("payload-select").value;
  if (!name) { flash("先选补偿文件", false); return; }
  try {
    const d = await post("/api/identify/payload/apply", { name });
    applyIdentify(d.identify);
    flash(`已调用「${name}」进前馈`, true);
  } catch (e) { flash(String(e), false); }
};
$("btn-payload-save").onclick = async () => {
  const name = ($("payload-new").value.trim() || $("payload-select").value || (currentSet && currentSet.name) || "").trim();
  if (!name) { flash("填一个另存名字", false); return; }
  try {
    const d = await post("/api/identify/payload/save", { name });
    applyIdentify(d.identify);
    $("payload-select").value = name;
    flash(`已保存「${name}」（未改当前前馈）`, true);
  } catch (e) { flash(String(e), false); }
};
$("btn-payload-del").onclick = async () => {
  const name = $("payload-select").value;
  if (!name) return;
  if (!confirm(`删除补偿文件「${name}」？`)) return;
  try {
    const d = await post("/api/identify/payload/delete", { name });
    applyIdentify(d.identify);
    flash("已删补偿文件", true);
  } catch (e) { flash(String(e), false); }
};
setInterval(poll, 100);
poll();
