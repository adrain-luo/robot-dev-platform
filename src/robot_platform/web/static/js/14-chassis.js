// 14-chassis.js —— 底盘遥控：摇杆 / 按钮 / 键盘「按住才走」，底栏限速滑条，急停联动
// 按住期间每 CHASSIS_BEAT_MS 发一次 /api/chassis/move（x 前后、yaw 转向，归一化 ±1）当心跳；
// 后端 0.5 s 收不到心跳就停车。松手、页面切走、失焦、急停都立刻发 /api/chassis/stop。

const CHASSIS_BEAT_MS = 100;
const chassis = { x: 0, yaw: 0, timer: null, src: "", keys: new Set(), errShown: false, limitDirty: false };

function chassisAllowed() {
  return !!(window.ROBOT && window.ROBOT.chassis && window.lastCtrl && window.lastCtrl.allow_real_motion);
}

async function chassisBeat() {
  if (!chassis.x && !chassis.yaw) return;
  try {
    await post("/api/chassis/move", { x: chassis.x, yaw: chassis.yaw });
    chassis.errShown = false;
  } catch (e) {
    if (!chassis.errShown) flash(`底盘：${e.message || e}`, false);
    chassis.errShown = true;
    chassisRelease("出错");
  }
}

// 设定当前指令（来源：pad / btn / key）；非零就开始心跳
function chassisSet(x, yaw, src) {
  if (!chassisAllowed()) return;
  chassis.x = Math.max(-1, Math.min(1, x));
  chassis.yaw = Math.max(-1, Math.min(1, yaw));
  chassis.src = src;
  if (!chassis.x && !chassis.yaw) { chassisRelease("回中"); return; }
  if (!chassis.timer) {
    chassisBeat();
    chassis.timer = setInterval(chassisBeat, CHASSIS_BEAT_MS);
  }
}

function chassisRelease(why) {
  const was = chassis.timer || chassis.x || chassis.yaw;
  chassis.x = chassis.yaw = 0;
  chassis.src = "";
  if (chassis.timer) { clearInterval(chassis.timer); chassis.timer = null; }
  chassisKnob(0, 0);
  document.querySelectorAll(".chassis-hold.on").forEach((b) => b.classList.remove("on"));
  if (was) {
    fetch("/api/chassis/stop", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ reason: why || "松手" }), keepalive: true,
    }).catch(() => {});
  }
}

function chassisKnob(nx, ny) {
  const knob = $("chassis-knob");
  const pad = $("chassis-pad");
  if (!knob || !pad) return;
  const r = (pad.clientWidth - knob.clientWidth) / 2;
  knob.style.transform = `translate(${nx * r}px, ${ny * r}px)`;
}

// ---------------------------------------------------------------- 摇杆
(function setupPad() {
  const pad = $("chassis-pad");
  if (!pad) return;
  let active = null;
  const read = (ev) => {
    const b = pad.getBoundingClientRect();
    const r = b.width / 2;
    let nx = (ev.clientX - b.left - r) / r;
    let ny = (ev.clientY - b.top - r) / r;
    const n = Math.hypot(nx, ny);
    if (n > 1) { nx /= n; ny /= n; }
    // 中间留一点死区，免得手抖
    const dz = (v) => (Math.abs(v) < 0.12 ? 0 : v);
    chassisKnob(nx, ny);
    chassisSet(dz(-ny), dz(-nx), "pad");
  };
  pad.addEventListener("pointerdown", (ev) => {
    if (!chassisAllowed()) { flash("只读模式不能开底盘（需 --allow-real-motion）", false); return; }
    active = ev.pointerId;
    pad.setPointerCapture(ev.pointerId);
    pad.classList.add("on");
    read(ev);
  });
  pad.addEventListener("pointermove", (ev) => { if (ev.pointerId === active) read(ev); });
  const up = (ev) => {
    if (ev.pointerId !== active) return;
    active = null;
    pad.classList.remove("on");
    chassisRelease("松手");
  };
  pad.addEventListener("pointerup", up);
  pad.addEventListener("pointercancel", up);
  pad.addEventListener("lostpointercapture", up);
})();

// ---------------------------------------------------------------- 按钮（按住）
document.querySelectorAll(".chassis-hold").forEach((btn) => {
  btn.addEventListener("pointerdown", (ev) => {
    if (!chassisAllowed()) { flash("只读模式不能开底盘（需 --allow-real-motion）", false); return; }
    btn.setPointerCapture(ev.pointerId);
    btn.classList.add("on");
    chassisSet(Number(btn.dataset.x), Number(btn.dataset.yaw), "btn");
  });
  const up = () => { if (btn.classList.contains("on")) chassisRelease("松手"); };
  btn.addEventListener("pointerup", up);
  btn.addEventListener("pointercancel", up);
  btn.addEventListener("lostpointercapture", up);
});

// ---------------------------------------------------------------- 键盘
const CHASSIS_KEYS = {
  w: [1, 0], arrowup: [1, 0], s: [-1, 0], arrowdown: [-1, 0],
  a: [0, 1], arrowleft: [0, 1], d: [0, -1], arrowright: [0, -1],
};

function chassisFromKeys() {
  let x = 0, yaw = 0;
  for (const k of chassis.keys) { x += CHASSIS_KEYS[k][0]; yaw += CHASSIS_KEYS[k][1]; }
  if (!x && !yaw) { if (chassis.src === "key") chassisRelease("松键"); return; }
  chassisSet(x, yaw, "key");
}

function chassisTyping(ev) {
  const t = ev.target;
  return t && (t.tagName === "INPUT" || t.tagName === "TEXTAREA" || t.tagName === "SELECT" || t.isContentEditable);
}

document.addEventListener("keydown", (ev) => {
  if (!$("chassis-keys-on")?.checked || chassisTyping(ev)) return;
  const k = ev.key.toLowerCase();
  if (!(k in CHASSIS_KEYS)) return;
  ev.preventDefault();
  if (ev.repeat && chassis.keys.has(k)) return;
  chassis.keys.add(k);
  chassisFromKeys();
});
document.addEventListener("keyup", (ev) => {
  const k = ev.key.toLowerCase();
  if (!chassis.keys.delete(k)) return;
  chassisFromKeys();
});
if ($("chassis-keys-on")) {
  $("chassis-keys-on").addEventListener("change", () => {
    if (!$("chassis-keys-on").checked) { chassis.keys.clear(); if (chassis.src === "key") chassisRelease("关键盘"); }
  });
}

// 看不到页面 / 失焦：一律停
window.addEventListener("blur", () => { chassis.keys.clear(); chassisRelease("失焦"); });
document.addEventListener("visibilitychange", () => { if (document.hidden) { chassis.keys.clear(); chassisRelease("切走"); } });
window.addEventListener("pagehide", () => chassisRelease("关页"));

// 急停 / 停止底盘
if ($("btn-hold")) $("btn-hold").addEventListener("click", () => { chassis.keys.clear(); chassisRelease("急停"); });
if ($("btn-chassis-stop")) {
  $("btn-chassis-stop").onclick = async () => {
    chassis.keys.clear();
    chassisRelease("停止");
    try { await post("/api/chassis/stop", { reason: "停止按钮" }); flash("底盘已停", true); } catch (e) { flash(String(e.message || e), false); }
  };
}

// ---------------------------------------------------------------- 底栏限速
function showChassisLimit(lin, yaw) {
  if ($("chassis-speed-val")) $("chassis-speed-val").textContent = Number(lin).toFixed(2);
  if ($("chassis-yaw-val")) $("chassis-yaw-val").textContent = (Number(yaw) * 180 / Math.PI).toFixed(1);
}

function chassisYawFor(lin) {
  const c = window.ROBOT?.chassis;
  if (!c) return 0;
  return Math.min(c.max_yaw_rad_s, lin * c.default_yaw_rad_s / c.default_lin_mps);
}

if ($("chassis-speed")) {
  $("chassis-speed").addEventListener("input", () => {
    chassis.limitDirty = true;
    const v = Number($("chassis-speed").value);
    showChassisLimit(v, chassisYawFor(v));
  });
  $("chassis-speed").addEventListener("change", async () => {
    const v = Number($("chassis-speed").value);
    try {
      const d = await post("/api/chassis/limit", { lin_mps: v });
      showChassisLimit(d.limit.lin_mps, d.limit.yaw_rad_s);
      flash(`底盘限速 ${d.limit.lin_mps.toFixed(2)} m/s、转向 ${(d.limit.yaw_rad_s * 180 / Math.PI).toFixed(1)}°/s`, true);
    } catch (e) { flash(String(e.message || e), false); }
    chassis.limitDirty = false;
  });
}

// ---------------------------------------------------------------- 状态
function renderChassis(c) {
  if (!c) return;
  const allow = chassisAllowed();
  const pad = $("chassis-pad");
  if (pad) pad.classList.toggle("off", !allow);
  document.querySelectorAll(".chassis-hold").forEach((b) => { b.disabled = !allow; });
  if ($("chassis-speed")) $("chassis-speed").disabled = !allow;
  const sp = $("chassis-speed");
  if (c.limit && sp && document.activeElement !== sp && !chassis.limitDirty) {
    sp.value = c.limit.lin_mps;
    showChassisLimit(c.limit.lin_mps, c.limit.yaw_rad_s);
  }
  const el = $("chassis-now");
  if (!el) return;
  const parts = [];
  const o = c.odom;
  if (o) {
    parts.push(`实测 ${Number(o.vx).toFixed(2)} m/s、${(Number(o.vyaw) * 180 / Math.PI).toFixed(1)}°/s`);
    parts.push(`里程计 x ${Number(o.x).toFixed(2)} y ${Number(o.y).toFixed(2)} m，朝向 ${(Number(o.yaw) * 180 / Math.PI).toFixed(1)}°`);
    if (o.age_s > 1) parts.push(`里程计 ${Number(o.age_s).toFixed(1)} s 没更新`);
  } else if (c.available) {
    parts.push("没有里程计数据");
  }
  if (c.moving && c.cmd) parts.push(`在开：${c.cmd.vx.toFixed(2)} m/s、${(c.cmd.vyaw * 180 / Math.PI).toFixed(1)}°/s`);
  else if (c.stop_reason) parts.push(`已停（${c.stop_reason}）`);
  if (c.error) parts.push(c.error);
  if (!c.available && c.note) parts.push(c.note);
  el.textContent = `底盘 ${parts.join(" · ") || "—"}`;
}
