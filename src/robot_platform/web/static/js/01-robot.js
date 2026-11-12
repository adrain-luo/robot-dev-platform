// 01-robot.js —— 机型能力（/api/robot）：标题、腰与立柱控件按它显示 / 隐藏；关节表
// 由 web/static/js/ 下各文件按文件名顺序拼成 /static/app.js（同一个脚本作用域）。

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
  if ($("joints-note")) $("joints-note").textContent = `左右两只手 + 腰：${desc}。`;
  show("lift-zero-block", !!caps.lift);
  show("lift-ctrl", !!caps.lift);
  show("chassis-ctrl", !!caps.chassis);
  show("panel-base", !!(caps.lift || caps.chassis));
  // 各页卡片数按本机型实际显示的卡片算（如 G1 没有「底盘」卡片）
  document.querySelectorAll(".page[data-page]").forEach((page) => {
    const n = [...page.querySelectorAll("article.float")].filter((a) => a.style.display !== "none").length;
    const head = page.querySelector(".page-head span");
    if (head) head.textContent = `${n} 张卡片`;
    const mark = document.querySelector(`.bookmark[data-page="${page.dataset.page}"] .count`);
    if (mark) mark.textContent = String(n);
  });
  show("chassis-speed-wrap", !!caps.chassis);
  show("chassis-yaw-wrap", !!caps.chassis);
  if (caps.chassis && $("chassis-speed")) $("chassis-speed").max = String(caps.chassis.max_lin_mps);
  if (caps.chassis && $("chassis-yaw")) {
    $("chassis-yaw").max = String(Math.floor(caps.chassis.max_yaw_rad_s * 180 / Math.PI * 2) / 2);
  }
  show("waist-ctrl", !!(w && w.movable));
  if ($("waist-note")) {
    if (!w) $("waist-note").textContent = "这台没有腰。";
    else if (!w.movable) $("waist-note").textContent = "腰：只在接管时保持当前角，不开放转动（站立时转腰会影响平衡）。";
    else {
      const lim = w.joints.map((j) => `${WAIST_ROLE[j.role] || j.role} ${j.limit_deg[0].toFixed(1)}°～${j.limit_deg[1].toFixed(1)}°`).join("，");
      $("waist-note").textContent = `腰：接管后与手臂共用限速。${lim}。条形拖到目标松手后走。`;
    }
  }
  const hasPitch = !!(w && w.joints.some((j) => j.role === "pitch"));
  for (const id of ["waist-pitch-slider", "waist-pitch", "btn-waist-pitch-m", "btn-waist-pitch-p"]) {
    const el = $(id);
    const box = el && (el.closest("label") || el);
    if (box) box.style.display = hasPitch ? "" : "none";
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

function renderTable(snaps, waist) {
  const tb = $("joints-table").querySelector("tbody");
  tb.innerHTML = "";
  for (const snap of snaps || []) {
    if (snap && snap.joints && snap.joints.length) {
      appendJointRows(tb, snap.joints, { section: `${ARM_SIDE[snap.arm] || snap.arm || ""}臂` });
    }
  }
  if (waist && waist.joints && waist.joints.length) {
    appendJointRows(tb, waist.joints, { section: `腰 · ${waist.joints.map((j) => `${j.motor} ${WAIST_ROLE[j.label.replace("waist_", "")] || j.label}`).join(" / ")}` });
  }
}
