// 08-spatial.js —— 空间点（定点碰 TCP、偏移瞄准）与棋盘触摸
// 由 web/static/js/ 下各文件按文件名顺序拼成 /static/app.js（同一个脚本作用域）。

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
    await pointPost("/api/points/save-now", { name, note: $("pt-note").value.trim(), arm: armPick("point") });
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
  const arm = armPick("board");
  const busy = armBusyReason(arm);
  if (busy) { flash(busy, false); return; }
  if (!confirm(`将用${ARM_SIDE[arm]}臂按限速逐个触摸棋盘角点（另一只手不受影响）。扶住、守急停。继续？`)) return;
  try {
    await boardPost("/api/board/start", {
      arm,
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
  const arm = armPick("point");
  const busy = armBusyReason(arm);
  if (busy) { flash(busy, false); return; }
  const off = readPtOffsetMm();
  const refine = !($("pt-refine") && !$("pt-refine").checked);
  const offTxt = `Δ[${off.x}, ${off.y}, ${off.z}] mm`;
  const refineTxt = refine ? "到位后误差补偿（最多 2 步）" : "仅开环走到瞄准点";
  if (!confirm(`将按限速用${ARM_SIDE[arm]}臂的 TCP 碰到「${name}」+ ${offTxt}。${refineTxt}。扶住、守急停。继续？`)) return;
  const btn = $("btn-pt-goto");
  if (btn) btn.disabled = true;
  flash(`正在走点（${offTxt}）${refine ? " / 误差补偿" : ""}…`, true);
  try {
    const d = await pointPost("/api/points/goto", {
      arm,
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
