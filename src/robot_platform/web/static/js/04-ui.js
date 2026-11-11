// 04-ui.js —— 页面框架：轮询 /api/scene、左右书与卡片的展开收起
// 由 web/static/js/ 下各文件按文件名顺序拼成 /static/app.js（同一个脚本作用域）。

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
    const modeLabel = data.ctrl?.float ? "实测 · 有手在拖拽" : (data.ctrl?.engaged ? "实测 · 已接管" : (data.mode === "real" ? "实测 · 未接管" : "实测 · 只读"));
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
    V3.scene = { right: right?.scene, left: left?.scene, joints, modeLabel, imu: data.imu || null };
    V3.draw();
    // 三维和关节表都是双臂；「手臂控制」滑条跟卡片上选的那只手
    renderTable([left, right], data.waist);
    const motionArm = armPick("motion");
    const ctrlSnap = motionArm === "left" ? left : right;
    if (ctrlSnap && ctrlSnap.joints) syncArmSlidersFromLive(ctrlSnap.joints);
    const note = $("arm-ctrl-now");
    if (note && ctrlSnap && ctrlSnap.joints) {
      const live = ctrlSnap.joints.map((j) => `${j.label} ${Number(j.q_deg).toFixed(1)}°`).join(" · ");
      note.textContent = `${ARM_SIDE[motionArm]}臂 · ${live}`;
    }
  } catch (e) {
    flash(String(e), false);
    setPill(false, "服务断开");
  } finally {
    inflight = false;
  }
}

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
