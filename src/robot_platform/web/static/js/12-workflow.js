// 12-workflow.js —— 工作流：识别中心后到工作姿态 / 中心点（逐步或自动）
// 由 web/static/js/ 下各文件按文件名顺序拼成 /static/app.js（同一个脚本作用域）。

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
  // 已完成时「自动继续」= 复位后从 ① 开始新一轮，不必先点「完成」
  if (btnAll) btnAll.disabled = !!st.busy;
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
  // 工作流跟着 TCP 卡片当前那套 TCP 的手走（工作姿态也记着是哪只手的）
  const busy = await armBusyReasonFresh(window.lastTcpArm);
  if (busy) throw new Error(busy);
  wfDgCheckAbort();
  wfDgSetNote("① 调用并锁定工作姿态…");
  const d = await workPosePost("/api/work-pose/load", {
    name,
    tcp_name: window.lastTcpName,
    arm: window.lastTcpArm,
  });
  wfDgCheckAbort();
  if (!d.ok) throw new Error(d.error || "工作姿态失败");
  if (window.wfDetectGoto) window.wfDetectGoto.arm = d.arm || window.lastTcpArm;
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
  const arm = (window.wfDetectGoto && window.wfDetectGoto.arm) || window.lastTcpArm;
  const busy = await armBusyReasonFresh(arm);
  if (busy) throw new Error(busy);
  const { ox, oy, oz, hz } = wfDgReadOffset();
  const offset_mm = kind === "lift" ? [ox, oy, oz + hz] : [ox, oy, oz];
  const refine = !($("wf-dg-refine") && !$("wf-dg-refine").checked);
  const tag = kind === "lift" ? "③ 偏置+提升" : "④ 仅偏置";
  wfDgCheckAbort();
  wfDgSetNote(`${tag} 走点中…`);
  const d = await post("/api/points/goto-xyz", {
    arm,
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
  if (st.busy) return;
  // 上一轮已跑完：从 ① 开始新一轮
  const newRound = !!st.finished || st.next >= WF_DG_STEP_COUNT;
  const from = newRound ? 0 : st.next;
  if (from === 0) {
    const name = ($("wf-dg-wp")?.value || "").trim();
    if (!name) { flash("先选择工作姿态", false); return; }
  }
  const remain = WF_DG_STEP_NAMES.slice(from).join(" → ");
  const head = newRound ? "上一轮已完成，开始新一轮" : "自动继续（从当前进度）";
  if (!confirm(`${head}：${remain}。扶住、守急停。继续？`)) return;
  if (newRound) wfDgResetToStart("新一轮开始");
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
    wfDgSetNote("正在停止（急停会锁定，之后点「复位急停」再动手臂）…");
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
