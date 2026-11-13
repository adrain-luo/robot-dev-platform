// 14-chassis.js —— 底盘遥控：摇杆 / 按钮 / 键盘「按住才走」（键盘开关在底栏最右，按住 E/Q 立柱升降），底栏限速滑条，急停联动
// 按住期间每 CHASSIS_BEAT_MS 发一次 /api/chassis/move（x 前后、yaw 转向，归一化 ±1）当心跳；
// 后端 0.5 s 收不到心跳就停车。松手、页面切走、失焦、急停都立刻发 /api/chassis/stop。

const CHASSIS_BEAT_MS = 100;
const chassis = { x: 0, yaw: 0, timer: null, src: "", keys: new Set(), errShown: false, linDirty: false, yawDirty: false };

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

// 立柱：按住 E 一直升、按住 Q 一直降，松键即停。按住期间每 LIFT_HOLD_MS 把目标重设为「当前高度 ± 步长」
// （步长取立柱卡片「步长 mm」，越大越快）；所以就算松键消息丢了 / 断网，最多再走一个步长就停在那里。
const LIFT_KEYS = { e: 1, q: -1 };
const LIFT_HOLD_MS = 150;
const liftHold = { dir: 0, timer: null, inflight: false };

async function liftHoldTick() {
  if (!liftHold.dir || liftHold.inflight) return;
  const step = parseFloat($("lift-step")?.value);
  if (!Number.isFinite(step) || step <= 0) { liftHoldStop("步长无效"); flash("立柱步长无效", false); return; }
  liftHold.inflight = true;
  try {
    const d = await post("/api/lift/step", { delta_mm: liftHold.dir * step });
    if (!d.ok) throw new Error(d.error || "立柱失败");
  } catch (e) {
    liftHoldStop("出错");
    flash(`立柱：${e.message || e}`, false);
  } finally {
    liftHold.inflight = false;
    // 松键时这一步还在路上：它到后端会重设目标，再补一次停
    if (!liftHold.dir) post("/api/lift/stop", { reason: "松键补停" }).catch(() => {});
  }
}

function liftHoldStart(dir) {
  if (liftHold.dir === dir) return;
  liftHold.dir = dir;
  clearInterval(liftHold.timer);
  liftHoldTick();
  liftHold.timer = setInterval(liftHoldTick, LIFT_HOLD_MS);
}

function liftHoldStop(why) {
  if (!liftHold.dir && !liftHold.timer) return;
  liftHold.dir = 0;
  clearInterval(liftHold.timer);
  liftHold.timer = null;
  post("/api/lift/stop", { reason: why || "松键" }).then(() => { window.liftTargetDirty = false; }).catch(() => {});
}

document.addEventListener("keydown", (ev) => {
  if (!$("chassis-keys-on")?.checked || chassisTyping(ev)) return;
  if (ev.ctrlKey || ev.metaKey || ev.altKey) return;
  const k = ev.key.toLowerCase();
  if (k in LIFT_KEYS) {
    if (!window.ROBOT?.lift) return;
    ev.preventDefault();
    if (ev.repeat) return;
    if ($("btn-lift-up")?.disabled) { flash("立柱现在不能动（未开真机运动？）", false); return; }
    liftHoldStart(LIFT_KEYS[k]);
    return;
  }
  if (!(k in CHASSIS_KEYS)) return;
  if (!window.ROBOT?.chassis) return;
  ev.preventDefault();
  if (ev.repeat && chassis.keys.has(k)) return;
  chassis.keys.add(k);
  chassisFromKeys();
});
document.addEventListener("keyup", (ev) => {
  const k = ev.key.toLowerCase();
  if (k in LIFT_KEYS) { if (liftHold.dir === LIFT_KEYS[k]) liftHoldStop("松键"); return; }
  if (!chassis.keys.delete(k)) return;
  chassisFromKeys();
});
if ($("chassis-keys-on")) {
  $("chassis-keys-on").addEventListener("change", () => {
    if (!$("chassis-keys-on").checked) {
      chassis.keys.clear(); if (chassis.src === "key") chassisRelease("关键盘");
      liftHoldStop("关键盘");
    }
  });
}

// 看不到页面 / 失焦：一律停
window.addEventListener("blur", () => { chassis.keys.clear(); chassisRelease("失焦"); liftHoldStop("失焦"); });
document.addEventListener("visibilitychange", () => {
  if (document.hidden) { chassis.keys.clear(); chassisRelease("切走"); liftHoldStop("切走"); }
});
window.addEventListener("pagehide", () => { chassisRelease("关页"); liftHoldStop("关页"); });

// 急停
if ($("btn-hold")) $("btn-hold").addEventListener("click", () => { chassis.keys.clear(); chassisRelease("急停"); liftHoldStop("急停"); });

// ---------------------------------------------------------------- 底栏限速（直线、转向两根滑条，互不影响）
const DEG = 180 / Math.PI;

function showChassisLin(lin) {
  if ($("chassis-speed-val")) $("chassis-speed-val").textContent = Number(lin).toFixed(2);
}
function showChassisYaw(yawRad) {
  if ($("chassis-yaw-val")) $("chassis-yaw-val").textContent = (Number(yawRad) * DEG).toFixed(1);
}

async function chassisSendLimit(body, dirtyKey) {
  try {
    const d = await post("/api/chassis/limit", body);
    showChassisLin(d.limit.lin_mps);
    showChassisYaw(d.limit.yaw_rad_s);
    flash(`底盘限速：直线 ${d.limit.lin_mps.toFixed(2)} m/s，转向 ${(d.limit.yaw_rad_s * DEG).toFixed(1)}°/s`, true);
  } catch (e) { flash(String(e.message || e), false); }
  chassis[dirtyKey] = false;
}

if ($("chassis-speed")) {
  $("chassis-speed").addEventListener("input", () => { chassis.linDirty = true; showChassisLin($("chassis-speed").value); });
  $("chassis-speed").addEventListener("change", () => chassisSendLimit({ lin_mps: Number($("chassis-speed").value) }, "linDirty"));
}
if ($("chassis-yaw")) {
  $("chassis-yaw").addEventListener("input", () => { chassis.yawDirty = true; showChassisYaw(Number($("chassis-yaw").value) / DEG); });
  $("chassis-yaw").addEventListener("change", () => chassisSendLimit({ yaw_rad_s: Number($("chassis-yaw").value) / DEG }, "yawDirty"));
}

// ---------------------------------------------------------------- 状态
function renderChassis(c) {
  if (!c) return;
  const allow = chassisAllowed();
  const pad = $("chassis-pad");
  if (pad) pad.classList.toggle("off", !allow);
  document.querySelectorAll(".chassis-hold").forEach((b) => { b.disabled = !allow; });
  const sp = $("chassis-speed");
  const yw = $("chassis-yaw");
  if (sp) sp.disabled = !allow;
  if (yw) yw.disabled = !allow;
  if (c.limit && sp && document.activeElement !== sp && !chassis.linDirty) {
    sp.value = c.limit.lin_mps;
    showChassisLin(c.limit.lin_mps);
  }
  if (c.limit && yw && document.activeElement !== yw && !chassis.yawDirty) {
    yw.value = (c.limit.yaw_rad_s * DEG).toFixed(1);
    showChassisYaw(c.limit.yaw_rad_s);
  }
  const el = $("chassis-now");
  if (!el) return;
  const parts = [];
  const o = c.odom;
  if (o) {
    parts.push(`实测 ${Number(o.vx).toFixed(2)} m/s、${(Number(o.vyaw) * DEG).toFixed(1)}°/s`);
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
