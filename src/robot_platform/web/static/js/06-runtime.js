// 06-runtime.js —— 运行配置：手眼 / 重力 / TCP / 工作姿态成套调用、配置锁定
// 由 web/static/js/ 下各文件按文件名顺序拼成 /static/app.js（同一个脚本作用域）。

const CFG_MUTATORS = [
  "btn-cfg-stereo", "btn-cfg-grav-left", "btn-cfg-grav-right",
  "btn-cfg-tcp-left", "btn-cfg-tcp-right",
  "cfg-he", "cfg-grav-left", "cfg-grav-right", "cfg-tcp-left", "cfg-tcp-right",
  "btn-he-solve", "btn-id-apply",
];

const CFG_SELECT_IDS = [
  "cfg-he", "cfg-grav-left", "cfg-grav-right", "cfg-tcp-left", "cfg-tcp-right",
  "cfg-wp-left", "cfg-wp-right", "cfg-pose-ik-mode",
];

const cfgDirty = Object.create(null);

function markCfgDirty(id) {
  cfgDirty[id] = true;
}

function clearCfgDirty(id) {
  if (id) delete cfgDirty[id];
  else CFG_SELECT_IDS.forEach((k) => delete cfgDirty[k]);
}

function cfgSelectBusy(id) {
  const sel = $(id);
  return !!cfgDirty[id] || (sel && document.activeElement === sel);
}

CFG_SELECT_IDS.forEach((id) => {
  const el = $(id);
  if (el) el.addEventListener("change", () => markCfgDirty(id));
});

function shortPath(p) {
  if (!p) return "—";
  const parts = String(p).split("/");
  return parts.length >= 2 ? parts.slice(-2).join("/") : p;
}

function fmtGrav(side) {
  if (!side || !side.name && side.mass_kg == null) return "未配置";
  const m = side.mass_kg != null ? `${Number(side.mass_kg).toFixed(3)} kg` : "";
  return `${side.name || "（无名）"}${m ? " · " + m : ""}`;
}

function renderRuntimeConfig(rt) {
  if (!rt) return;
  window.lastRuntime = rt;
  const locked = !!rt.locked;
  window.configLocked = locked;
  const st = rt.stereo || {};
  const g = rt.gravity || {};
  const tcp = rt.tcp || {};
  const badge = $("cfg-lock-badge");
  if (badge) {
    badge.textContent = locked ? "已锁定" : "";
    badge.classList.toggle("locked", locked);
  }
  const now = $("cfg-now");
  if (now) {
    const live = g.live || {};
    const fmtLive = (side, x) => (x && x.name)
      ? `${side}臂前馈「${x.name}」${x.mass_kg != null ? ` ${Number(x.mass_kg).toFixed(3)} kg` : ""}`
      : `${side}臂无负载前馈`;
    const applied = window.lastCtrl?.allow_real_motion
      ? `${fmtLive("左", live.left)}
${fmtLive("右", live.right)}`
      : "只读模式，没有前馈";
    now.classList.toggle("locked", locked);
    now.textContent =
      `${locked ? "🔒 已锁定\n" : ""}` +
      `手眼：${st.active_result || "—"}\n` +
      `双目 yaml：${shortPath(st.intrinsics)}\n` +
      applied;
  }
  const wp = rt.work_pose || {};
  const nowL = $("cfg-now-left");
  if (nowL) {
    const show = wp.locked && wp.arm === "left" ? (wp.active_name || "无") : "无";
    nowL.textContent = `重力：${fmtGrav(g.left)}\nTCP：${tcp.left?.name || "—"}\n工作姿态锁定：${show}`;
  }
  const nowR = $("cfg-now-right");
  if (nowR) {
    const show = wp.locked && wp.arm === "right" ? (wp.active_name || "无") : "无";
    nowR.textContent = `重力：${fmtGrav(g.right)}\nTCP：${tcp.right?.name || "—"}\n工作姿态锁定：${show}`;
  }
  const bound = $("cfg-stereo-bound");
  if (bound) {
    const pending = cfgDirty["cfg-he"] && $("cfg-he")?.value;
    const pendingYaml = pending
      ? (st.handeye_saved || []).find((s) => s.name === pending)?.intrinsics_yaml
      : "";
    const show = pendingYaml || st.intrinsics;
    bound.textContent = pending
      ? `双目 yaml：${shortPath(show)}（已选「${pending}」，点应用后生效）`
      : `双目 yaml：${shortPath(st.intrinsics)}（随手眼绑定，不可单独改）`;
  }
  const heSel = $("cfg-he");
  if (heSel && !cfgSelectBusy("cfg-he")) {
    writeOptions(
      heSel,
      (rt.stereo?.handeye_saved || []).length ? "选择手眼结果…" : "还没有保存的手眼",
      (rt.stereo?.handeye_saved || []).map((s) => ({
        value: s.name,
        label: `${s.name}${s.baseline_mm != null ? ` · ${Number(s.baseline_mm).toFixed(1)} mm` : ""}${s.intrinsics_yaml ? " · 含双目" : ""}`,
      })),
      rt.stereo?.active_result || ""
    );
  }
  for (const [selId, arm] of [["cfg-grav-left", "left"], ["cfg-grav-right", "right"]]) {
    const sel = $(selId);
    if (!sel || cfgSelectBusy(selId)) continue;
    const rows = (rt.gravity?.profiles || [])
      .filter((p) => !p.arm || p.arm === arm)
      .map((p) => ({
        value: p.name,
        label: `${p.name}${p.mass_kg != null ? ` · ${Number(p.mass_kg).toFixed(2)} kg` : ""}`,
      }));
    const keep = arm === "left" ? rt.gravity?.left?.name : rt.gravity?.right?.name;
    writeOptions(sel, rows.length ? `选择${arm === "left" ? "左" : "右"}臂补偿…` : "还没有补偿文件", rows, keep || "");
  }
  for (const [selId, arm] of [["cfg-tcp-left", "left"], ["cfg-tcp-right", "right"]]) {
    const sel = $(selId);
    if (!sel || cfgSelectBusy(selId)) continue;
    const rows = (rt.tcp?.saved || [])
      .filter((s) => s.arm === arm)
      .map((s) => ({
        value: s.name,
        label: `${s.name}${s.residual_max_mm != null ? ` · max ${Number(s.residual_max_mm).toFixed(1)} mm` : ""}`,
      }));
    const keep = arm === "left" ? rt.tcp?.active_left : rt.tcp?.active_right;
    writeOptions(
      sel,
      rows.length ? `选择${arm === "left" ? "左" : "右"}臂 TCP…` : `还没有${arm === "left" ? "左" : "右"}臂 TCP`,
      rows,
      keep || ""
    );
  }
  // 工作姿态：按该侧当前 TCP 列姿态；配置锁定时仍可改（不上锁）
  for (const [selId, arm] of [["cfg-wp-left", "left"], ["cfg-wp-right", "right"]]) {
    const sel = $(selId);
    if (!sel || cfgSelectBusy(selId)) continue;
    const tcpName = arm === "left"
      ? ($("cfg-tcp-left")?.value || rt.tcp?.active_left || "")
      : ($("cfg-tcp-right")?.value || rt.tcp?.active_right || "");
    const all = wp.saved_all || window.lastWorkPoseAll || [];
    const rows = all
      .filter((s) => s.tcp_name === tcpName && (!s.arm || s.arm === arm))
      .map((s) => ({ value: s.name, label: s.note ? `${s.name} · ${s.note}` : s.name }));
    const keep = (wp.locked && wp.arm === arm && wp.tcp_name === tcpName) ? (wp.active_name || "") : "";
    writeOptions(
      sel,
      tcpName
        ? (rows.length ? `选择工作姿态（${tcpName}）…` : `「${tcpName}」还没有工作姿态`)
        : "先选该侧 TCP",
      rows,
      keep
    );
  }
  const allowMotion = !!window.lastCtrl?.allow_real_motion;
  CFG_MUTATORS.forEach((id) => {
    const el = $(id);
    if (!el) return;
    if (locked) {
      el.disabled = true;
      return;
    }
    if (id.startsWith("cfg-") || id.startsWith("btn-cfg-")) {
      el.disabled = false;
      return;
    }
    if (id === "btn-he-solve") {
      el.disabled = false;
      return;
    }
    if (id === "btn-id-apply") {
      el.disabled = !allowMotion;
    }
  });
  // 工作姿态 / IK 策略：永远可改（不受配置锁定）
  ["cfg-wp-left", "cfg-wp-right", "btn-cfg-wp-left", "btn-cfg-wp-right",
    "cfg-pose-ik-mode", "btn-cfg-pose-ik-mode"].forEach((id) => {
    const el = $(id);
    if (!el) return;
    if (id.startsWith("btn-cfg-wp")) el.disabled = !allowMotion;
    else el.disabled = false;
  });
  const ikSel = $("cfg-pose-ik-mode");
  if (ikSel && !cfgSelectBusy("cfg-pose-ik-mode")) {
    const mode = wp.ik_mode === "single" ? "single" : "robust";
    ikSel.value = mode;
  }
  const lockBtn = $("btn-cfg-lock");
  const unlockBtn = $("btn-cfg-unlock");
  if (lockBtn) lockBtn.disabled = locked;
  if (unlockBtn) unlockBtn.disabled = !locked;
}

async function cfgPost(path, body, okMsg, dirtyIds) {
  const d = await post(path, body || {});
  (dirtyIds || []).forEach(clearCfgDirty);
  if (d.runtime) renderRuntimeConfig(d.runtime);
  else if (d.stereo || d.tcp || d.gravity) renderRuntimeConfig(d);
  if (String(path).indexOf("work-pose") >= 0) {
    renderWorkPose(d, { active_name: d.tcp_name || window.lastTcpName, arm: d.arm });
    if (window.lastRuntime) {
      window.lastRuntime.work_pose = d;
      renderRuntimeConfig(window.lastRuntime);
    }
  }
  flash(okMsg || d.message || "已应用", true);
  return d;
}

$("btn-cfg-stereo").onclick = async () => {
  const name = $("cfg-he").value;
  if (!name) { flash("先选手眼结果", false); return; }
  try {
    await cfgPost(
      "/api/runtime-config/stereo",
      { active_result: name },
      `已应用手眼「${name}」（双目 yaml 已一并绑定）`,
      ["cfg-he"]
    );
  } catch (e) { flash(String(e), false); }
};

$("btn-cfg-grav-left").onclick = async () => {
  const name = $("cfg-grav-left").value;
  if (!name) { flash("先选左臂补偿", false); return; }
  try { await cfgPost("/api/runtime-config/gravity", { arm: "left", name }, `左臂重力已设为「${name}」`, ["cfg-grav-left"]); }
  catch (e) { flash(String(e), false); }
};

$("btn-cfg-grav-right").onclick = async () => {
  const name = $("cfg-grav-right").value;
  if (!name) { flash("先选右臂补偿", false); return; }
  try { await cfgPost("/api/runtime-config/gravity", { arm: "right", name }, `右臂重力已设为「${name}」`, ["cfg-grav-right"]); }
  catch (e) { flash(String(e), false); }
};

$("btn-cfg-tcp-left").onclick = async () => {
  const name = $("cfg-tcp-left").value;
  if (!name) { flash("先选左臂 TCP", false); return; }
  try { await cfgPost("/api/runtime-config/tcp", { arm: "left", name }, `左臂 TCP 已设为「${name}」`, ["cfg-tcp-left"]); }
  catch (e) { flash(String(e), false); }
};

$("btn-cfg-tcp-right").onclick = async () => {
  const name = $("cfg-tcp-right").value;
  if (!name) { flash("先选右臂 TCP", false); return; }
  try { await cfgPost("/api/runtime-config/tcp", { arm: "right", name }, `右臂 TCP 已设为「${name}」`, ["cfg-tcp-right"]); }
  catch (e) { flash(String(e), false); }
};

async function cfgApplyWorkPose(arm) {
  const tcpSel = arm === "left" ? $("cfg-tcp-left") : $("cfg-tcp-right");
  const wpSel = arm === "left" ? $("cfg-wp-left") : $("cfg-wp-right");
  const tcpName = tcpSel?.value || "";
  const name = wpSel?.value || "";
  if (!tcpName) { flash("先选该侧 TCP", false); return; }
  if (!name) { flash("先选工作姿态", false); return; }
  if (window.lastCtrl?.arms?.[arm]?.float) { flash(`${arm === "left" ? "左" : "右"}臂在拖拽，先退出拖拽`, false); return; }
  if (!confirm(`将转到工作姿态「${name}」（TCP「${tcpName}」）并锁定方向。扶住、守急停。继续？`)) return;
  try {
    await cfgPost(
      "/api/runtime-config/work-pose",
      { arm, tcp_name: tcpName, name },
      `已锁定工作姿态「${name}」`,
      [arm === "left" ? "cfg-wp-left" : "cfg-wp-right"],
    );
  } catch (e) { flash(String(e), false); }
}

if ($("btn-cfg-wp-left")) $("btn-cfg-wp-left").onclick = () => cfgApplyWorkPose("left");

if ($("btn-cfg-wp-right")) $("btn-cfg-wp-right").onclick = () => cfgApplyWorkPose("right");

if ($("btn-cfg-pose-ik-mode")) {
  $("btn-cfg-pose-ik-mode").onclick = async () => {
    const mode = $("cfg-pose-ik-mode")?.value || "robust";
    try {
      await cfgPost(
        "/api/runtime-config/pose-ik-mode",
        { mode },
        mode === "single" ? "已切到旧·单次局部 IK" : "已切到新·多初值 IK",
        ["cfg-pose-ik-mode"],
      );
    } catch (e) { flash(String(e), false); }
  };
}

// TCP 下拉变更时刷新该侧工作姿态列表
["cfg-tcp-left", "cfg-tcp-right"].forEach((id) => {
  const el = $(id);
  if (!el) return;
  el.addEventListener("change", () => {
    if (window.lastRuntime) renderRuntimeConfig(window.lastRuntime);
  });
});

$("btn-cfg-lock").onclick = async () => {
  try { await cfgPost("/api/runtime-config/lock", { locked: true }, "配置已锁定"); }
  catch (e) { flash(String(e), false); }
};

$("btn-cfg-unlock").onclick = async () => {
  try { await cfgPost("/api/runtime-config/lock", { locked: false }, "配置已解锁"); }
  catch (e) { flash(String(e), false); }
};
