// 05-payload.js —— 姿态集、末端负载辨识、静止判定、到位对比
// 由 web/static/js/ 下各文件按文件名顺序拼成 /static/app.js（同一个脚本作用域）。

let stillDirty = false;

let pinnedVal = null;

let valList = [];

let lastIdentify = null;

let valWasRunning = false;

function viewFromPin(p) {
  const rows = p.rows || [];
  return {
    kind: "validate",
    running: false,
    session: p.session,
    phase: "",
    progress: [rows.length, rows.length],
    band: p.friction_band || p.band || {},
    summary: p.summary || {},
    rows,
    pinned: true,
    file: p.file,
    finished: p.finished,
  };
}

function renderVal(id) {
  if (!$("val-table")) return;
  const running = !!(id && id.kind === "validate" && id.running);
  const src = (!running && pinnedVal) ? viewFromPin(pinnedVal) : (id || {});
  const [a, b] = src.progress || [0, 0];
  if (running) {
    $("val-progress").textContent = `${src.phase || ""} ${a}/${b}`;
  } else if (src.pinned) {
    $("val-progress").textContent = src.session || "";
  } else {
    $("val-progress").textContent = "";
  }
  const fileEl = $("val-file");
  if (fileEl) {
    const file = src.file || (src.session ? `data/validate/${src.session}.json` : "");
    fileEl.textContent = file;
  }
  const band = src.band || {};
  if (band.default_deg) {
    const where = band.fallback ? "无双向数据，常数兜底" : `${band.n_sessions}次双向 / ${band.n_poses}姿态`;
    $("val-band").textContent = `摩擦带（${where}）默认 [${band.default_deg.map((x) => Number(x).toFixed(2)).join(", ")}] °`;
  }
  const sum = src.summary || {};
  const host = $("val-summary");
  host.replaceChildren();
  for (const [k, v] of Object.entries(sum)) {
    const box = document.createElement("div");
    box.className = "scheme";
    const title = document.createElement("b");
    title.textContent = k;
    const ang = document.createElement("span");
    ang.innerHTML = `关节均 <strong>${Number(v.err_max_deg_mean).toFixed(2)}°</strong> · 最差 <strong>${Number(v.err_max_deg_worst).toFixed(2)}°</strong>`;
    const ee = document.createElement("span");
    ee.innerHTML = `末端均 <strong>${Number(v.ee_mm_mean).toFixed(1)} mm</strong> · 最差 <strong>${Number(v.ee_mm_worst).toFixed(1)} mm</strong>`;
    box.append(title, ang, ee);
    host.append(box);
  }
  const tb = $("val-table").querySelector("tbody");
  tb.innerHTML = (src.rows || []).map((r) =>
    `<tr><td>${r.mode}</td><td>${r.pose}</td><td>${Number(r.err_max_deg).toFixed(2)}</td>` +
    `<td>${Number(r.ee_mm).toFixed(1)}</td><td>${r.corr_applied || 0}${r.corr_reverted ? "↩" : ""}</td>` +
    `<td>${Number(r.total_s).toFixed(1)}${r.timed_out ? "⚠" : ""}</td></tr>`
  ).join("");
}

function filteredValList() {
  if (!window.currentSet) return valList;
  const rows = valList.filter((s) => s.arm === window.currentSet.arm && s.poseset === window.currentSet.name);
  return rows.length ? rows : valList;
}

function fillValSelect() {
  const sel = $("val-session");
  if (!sel || document.activeElement === sel) return;
  const keep = (pinnedVal && pinnedVal.session) || sel.value;
  const rows = filteredValList();
  while (sel.options.length) sel.remove(0);
  sel.add(new Option(rows.length ? "打开已保存的对比…" : "还没有对比记录", ""));
  for (const s of rows) {
    const side = s.arm === "left" ? "左" : (s.arm === "right" ? "右" : "?");
    const rel = `data/validate/${s.name}.json`;
    const opt = new Option(`${rel} · ${side} ${s.poseset || ""}`, s.name);
    opt.title = s.file || rel;
    sel.add(opt);
  }
  if (keep && [...sel.options].some((o) => o.value === keep)) sel.value = keep;
}

async function refreshValList() {
  const d = await fetch("/api/validate/list", { cache: "no-store" }).then((r) => r.json());
  if (!d.ok) return;
  valList = d.list || [];
  fillValSelect();
}

async function openVal(name) {
  if (!name) return;
  const d = await fetch("/api/validate/result?name=" + encodeURIComponent(name), { cache: "no-store" }).then((r) => r.json());
  if (!d.ok) throw new Error(d.error || "打不开对比记录");
  pinnedVal = d.result;
  const meta = valList.find((s) => s.name === name);
  if (meta && meta.file) pinnedVal.file = meta.file;
  fillValSelect();
  renderVal(lastIdentify || {});
}

function applyIdentify(id) {
  if (!id) return;
  lastIdentify = id;
  const running = !!(id.kind === "validate" && id.running);
  if (valWasRunning && !running && id.session) {
    pinnedVal = {
      session: id.session,
      rows: id.rows || [],
      summary: id.summary || {},
      friction_band: id.band || {},
    };
    refreshValList().catch(() => {});
  }
  valWasRunning = running;
  const [a, b] = id.progress || [0, 0];
  $("id-progress").textContent = id.running ? `${id.phase || ""} ${a}/${b}` : (id.session ? `会话 ${id.session}` : "");
  $("id-log").textContent = (id.log || []).join("\n");
  const sol = id.solution;
  const result = $("id-result");
  if (sol) {
    result.className = "stats";
    result.replaceChildren();
    const com = (sol.com_m || []).map((x) => Number(x).toFixed(3)).join(", ");
    const items = [
      ["质量", `${Number(sol.mass_kg).toFixed(3)} kg`],
      ["质心 m", com],
      ["条件数", Number(sol.regressor_cond).toFixed(1)],
      ["残差 N·m", `${Number(sol.residual_rms_before_nm).toFixed(2)} → ${Number(sol.residual_rms_after_nm).toFixed(2)}`],
      ["估下垂", `${Number(sol.sag_est_before_deg).toFixed(2)}° → ${Number(sol.sag_est_after_deg).toFixed(2)}°`],
    ];
    for (const [k, v] of items) {
      const d = document.createElement("div");
      d.className = "stat";
      const em = document.createElement("em");
      em.textContent = k;
      const strong = document.createElement("strong");
      strong.textContent = v;
      d.append(em, strong);
      result.append(d);
    }
    const files = [];
    if (id.session_dir) files.push(id.session_dir);
    if (id.payload_file) files.push(id.payload_file);
    if (files.length) {
      const foot = document.createElement("p");
      foot.className = "foot";
      foot.textContent = files.join("\n");
      result.append(foot);
    }
  } else if (id.error) {
    result.className = "";
    result.textContent = id.error;
  }
  const allow = !!window.lastCtrl?.allow_real_motion && !!window.lastCtrl?.engaged && !window.lastCtrl?.float;
  const measure = $("btn-id-measure");
  const stop = $("btn-id-stop");
  const valStart = $("btn-val-start");
  if (measure) measure.disabled = !allow || !!id.running;
  if (stop) stop.disabled = !id.running;
  if (valStart) valStart.disabled = !allow || !!id.running;
  renderVal(id);
  renderPayloadTable(id);
  renderSessionSelect(id.sessions || [], id.session);
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

let poseList = [];

function refillSelects() {
  renderPoseList(poseList);
  if (lastIdentify) {
    renderPayloadTable(lastIdentify);
    renderSessionSelect(lastIdentify.sessions || [], lastIdentify.session);
  }
  // 打开卡片时若还没拉过列表，不要用空 valList 冲掉服务端注入的选项
  if (valList.length) fillValSelect();
  else refreshValList().catch(() => {});
}

window.refillSelects = refillSelects;

function posesetName() {
  return $("poseset-select").value;
}

function renderPoseList(list) {
  const sel = $("poseset-select");
  if (!sel) return;
  const rows = (list || []).map((s) => {
    const side = s.arm === "left" ? "左" : "右";
    const cond = s.cond != null ? ` · cond ${s.cond}` : "";
    return { value: s.name, label: `${s.name} · ${side} · ${s.n}点${cond}` };
  });
  writeOptions(sel, rows.length ? "选择姿态集…" : "还没有姿态集", rows, posesetName());
}

function renderChips() {
  const box = $("pose-chips");
  box.innerHTML = "";
  $("pose-cond").textContent = "";
  if (!window.currentSet) return;
  const cond = window.currentSet.regressor_cond;
  $("pose-cond").textContent = window.currentSet.arm === "left" ? "左臂" : "右臂"
    + (cond != null ? ` · 回归 cond ${cond}` : "")
    + (window.currentSet.mirrored_from ? ` · 镜像自 ${window.currentSet.mirrored_from.name}` : "");
  (window.currentSet.poses || []).forEach((p, i) => {
    const wrap = document.createElement("span");
    wrap.className = "chip";
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
  poseList = d.list || [];
  renderPoseList(poseList);
  if (selectName) $("poseset-select").value = selectName;
  if (posesetName()) await loadCurrentSet();
  else { window.currentSet = null; renderChips(); }
}

async function loadCurrentSet() {
  const name = posesetName();
  if (!name) { window.currentSet = null; renderChips(); return; }
  const d = await fetch("/api/posesets?name=" + encodeURIComponent(name)).then((r) => r.json());
  if (!d.ok) throw new Error(d.error);
  window.currentSet = d.set;
  renderChips();
  try {
    await refreshValList();
    const rows = filteredValList();
    const keep = pinnedVal && rows.some((s) => s.name === pinnedVal.session);
    if (!keep && rows[0] && !(lastIdentify && lastIdentify.kind === "validate" && lastIdentify.running)) {
      await openVal(rows[0].name);
    } else {
      fillValSelect();
    }
  } catch (_) { /* ignore */ }
}

async function gotoPose(i) {
  if (!window.currentSet) return;
  fillQ(window.currentSet.poses[i].q);
  const ctrl = window.lastCtrl;
  if (!ctrl?.allow_real_motion || !ctrl.engaged || ctrl.float) {
    flash("已填入该姿态角（需接管且非拖拽才能走过去）", true);
    return;
  }
  if (ctrl.arm !== window.currentSet.arm) {
    flash(`姿态集是${window.currentSet.arm === "left" ? "左" : "右"}臂，请先切控制臂`, false);
    return;
  }
  try {
    await post("/api/posesets/goto", { name: window.currentSet.name, index: i });
    flash(`正在去 ${window.currentSet.poses[i].name}`, true);
  } catch (e) { flash(String(e), false); }
}

async function deletePose(i) {
  if (!window.currentSet) return;
  if (!confirm(`删除 ${window.currentSet.poses[i].name}？`)) return;
  try {
    const d = await post("/api/posesets/delete-pose", { name: window.currentSet.name, index: i });
    window.currentSet = d.set;
    renderChips();
    await refreshPoseList(window.currentSet.name);
  } catch (e) { flash(String(e), false); }
}

async function recordPose() {
  if (!posesetName()) { flash("先新建或选择姿态集", false); return; }
  try {
    const d = await post("/api/posesets/record", { name: posesetName() });
    window.currentSet = d.set;
    renderChips();
    await refreshPoseList(window.currentSet.name);
    flash(`已记录 ${d.pose.name}（${window.currentSet.arm === "left" ? "左" : "右"}臂实测）`, true);
  } catch (e) { flash(String(e), false); }
}

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
    window.currentSet = null;
    await refreshPoseList();
    flash("已删集", true);
  } catch (e) { flash(String(e), false); }
};

$("btn-poseset-record").onclick = () => recordPose();

$("btn-poseset-mirror").onclick = async () => {
  if (!window.currentSet) { flash("先选姿态集", false); return; }
  const dst = window.currentSet.arm === "left" ? "右" : "左";
  if (!confirm(`把「${window.currentSet.name}」关于 y=0 镜像成${dst}臂姿态集？关节角会改符号，不是原样复制。`)) return;
  try {
    const d = await post("/api/posesets/mirror", { src: window.currentSet.name });
    await refreshPoseList(d.set.name);
    flash(`已镜像为「${d.set.name}」${dst}臂 ${d.set.poses.length} 点`, true);
  } catch (e) {
    if (String(e).includes("已存在") && confirm("对侧集已存在，覆盖？")) {
      try {
        const d = await post("/api/posesets/mirror", { src: window.currentSet.name, force: true });
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

$("btn-id-measure").onclick = async () => {
  if (!posesetName()) { flash("先选姿态集", false); return; }
  if (!confirm("将按当前限速走完姿态集（双向逼近）。扶住、守急停。继续？")) return;
  try {
    await post("/api/identify/measure", { name: posesetName() });
    flash("开始测量", true);
  } catch (e) { flash(String(e), false); }
};

$("btn-id-stop").onclick = async () => {
  try { await post("/api/identify/stop", {}); flash("已请求停止", true); }
  catch (e) { flash(String(e), false); }
};

$("btn-id-solve").onclick = async () => {
  const session = $("id-session") && $("id-session").value;
  if (!session) { flash("先选一条测量会话（或先测量当前集）", false); return; }
  try {
    const d = await post("/api/identify/solve", { source: $("id-source").value, session });
    applyIdentify(d.identify);
    flash("已用会话 " + session + " 重新求解", true);
  } catch (e) { flash(String(e), false); }
};

function renderSessionSelect(list, current) {
  const sel = $("id-session");
  if (!sel || document.activeElement === sel) return;
  let rows = list || [];
  if (window.currentSet) {
    const filtered = rows.filter((s) => s.arm === window.currentSet.arm && s.poseset === window.currentSet.name);
    if (filtered.length) rows = filtered;
  }
  writeOptions(sel, rows.length ? "选择测量会话再求解…" : "还没有测量会话（先点测量当前集）", rows.map((s) => {
    const side = s.arm === "left" ? "左" : (s.arm === "right" ? "右" : "?");
    const bi = s.bidirectional ? "双向" : "单向";
    return { value: s.name, label: `${s.name} · ${side} ${s.poseset || ""} ${s.n}样 ${bi}` };
  }), current || sel.value);
}

function renderPayloadTable(id) {
  const body = $("payload-table-body");
  const empty = $("payload-table-empty");
  if (!body) return;
  const saved = (id && id.profiles) || [];
  window.lastPayloadSaved = saved;
  const activeL = (id && id.applied_left) || "";
  const activeR = (id && id.applied_right) || "";
  body.innerHTML = "";
  if (empty) empty.classList.toggle("show", saved.length === 0);
  saved.forEach((p) => {
    const name = p.name || "";
    const arm = p.arm === "left" || p.arm === "right" ? p.arm : "";
    const side = arm === "left" ? "左" : arm === "right" ? "右" : "—";
    const mass = p.mass_kg != null && Number.isFinite(Number(p.mass_kg))
      ? `${Number(p.mass_kg).toFixed(2)} kg`
      : "—";
    const on = (arm === "left" && name === activeL) || (arm === "right" && name === activeR);
    const tr = document.createElement("tr");
    tr.dataset.name = name;
    tr.innerHTML = `
      <td class="pay-name"></td>
      <td>${side}</td>
      <td class="pay-meta"></td>
      <td class="pay-meta pay-note"></td>
      <td><button type="button" class="pay-act pay-apply${on ? " on" : ""}" data-act="apply" title="${on ? "该手当前补偿" : "设为该手补偿"}">${on ? "✓" : "○"}</button></td>
      <td><button type="button" class="pay-act pay-del" data-act="delete">删除</button></td>
    `;
    tr.querySelector(".pay-name").textContent = name;
    tr.querySelector(".pay-meta").textContent = mass;
    tr.querySelector(".pay-note").textContent = p.note || "—";
    body.appendChild(tr);
  });
}

async function payloadApplySaved(name) {
  if (!name) return;
  try {
    const d = await post("/api/identify/payload/apply", { name });
    applyIdentify(d.identify);
    if (d.runtime) renderRuntimeConfig(d.runtime);
    window.lastPayloadPick = name;
    flash(d.message || `已应用「${name}」`, true);
  } catch (e) { flash(String(e), false); }
}

async function payloadDeleteSaved(name) {
  if (!name) return;
  if (!confirm(`删除补偿「${name}」？若正在应用将去掉负载项。`)) return;
  try {
    const d = await post("/api/identify/payload/delete", { name });
    applyIdentify(d.identify);
    if (d.runtime) renderRuntimeConfig(d.runtime);
    if (window.lastPayloadPick === name) window.lastPayloadPick = "";
    flash(d.message || "已删除", true);
  } catch (e) { flash(String(e), false); }
}

if ($("payload-table-body")) {
  $("payload-table-body").addEventListener("click", (ev) => {
    const btn = ev.target.closest("button[data-act]");
    if (!btn) return;
    const tr = btn.closest("tr");
    const name = tr?.dataset?.name || "";
    if (!name) return;
    if (btn.dataset.act === "apply") payloadApplySaved(name);
    else if (btn.dataset.act === "delete") payloadDeleteSaved(name);
  });
}

$("btn-id-apply").onclick = async () => {
  try {
    const name = (
      $("payload-new")?.value.trim()
      || (window.currentSet && window.currentSet.name)
      || ""
    ).trim();
    if (!name) { flash("填一个名字再存并应用", false); return; }
    const exists = (window.lastPayloadSaved || []).some((p) => p.name === name);
    if (exists && !confirm(`覆盖已保存的「${name}」？`)) return;
    const d = await post("/api/identify/apply", { name });
    applyIdentify(d.identify);
    if (d.runtime) renderRuntimeConfig(d.runtime);
    window.lastPayloadPick = d.payload?.name || name;
    if ($("payload-new")) $("payload-new").value = "";
    flash(`已存并应用「${d.payload.name}」`, true);
  } catch (e) { flash(String(e), false); }
};

$("btn-id-clear").onclick = async () => {
  try {
    const d = await post("/api/identify/clear", {});
    if (d.identify) applyIdentify(d.identify);
    flash("已去掉负载项，只剩 URDF 臂自重", true);
  } catch (e) { flash(String(e), false); }
};

$("btn-payload-save").onclick = async () => {
  const name = (
    $("payload-new")?.value.trim()
    || (window.currentSet && window.currentSet.name)
    || ""
  ).trim();
  if (!name) { flash("填一个另存名字", false); return; }
  const exists = (window.lastPayloadSaved || []).some((p) => p.name === name);
  if (exists && !confirm(`覆盖已保存的「${name}」？`)) return;
  try {
    const d = await post("/api/identify/payload/save", { name });
    applyIdentify(d.identify);
    if ($("payload-new")) $("payload-new").value = "";
    flash(`已保存「${name}」（未改当前前馈）`, true);
  } catch (e) { flash(String(e), false); }
};

$("btn-val-start").onclick = async () => {
  if (!posesetName()) { flash("先选姿态集", false); return; }
  const modes = [];
  if ($("val-base").checked) modes.push("baseline");
  if ($("val-pay").checked) modes.push("payload");
  if ($("val-corr").checked) modes.push("payload_correct");
  if (!modes.length) { flash("至少勾一种方案", false); return; }
  const payloadName = (lastIdentify && lastIdentify.applied_name)
    || window.lastPayloadPick
    || "";
  if ((modes.includes("payload") || modes.includes("payload_correct")) && !payloadName) {
    flash("补偿方案需要先在末端负载管理里应用一套补偿", false); return;
  }
  if (!confirm("将按所选方案走完整姿态集（同一限速）。扶住、守急停。继续？")) return;
  try {
    await post("/api/validate/start", {
      name: posesetName(),
      modes,
      payload: payloadName,
      correction_iters: parseInt($("val-iters").value, 10) || 2,
    });
    flash("开始对比", true);
  } catch (e) { flash(String(e), false); }
};

$("val-session").onchange = async () => {
  const name = $("val-session").value;
  if (!name) return;
  try { await openVal(name); }
  catch (e) { flash(String(e), false); }
};
