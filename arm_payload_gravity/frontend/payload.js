const $ = (id) => document.getElementById(id);
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
    const file = src.file || (src.session ? `arm_control/data/validate/${src.session}.json` : "");
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
  sel.innerHTML = "";
  const o0 = document.createElement("option");
  o0.value = "";
  o0.textContent = rows.length ? "打开已保存的对比…" : "还没有对比记录";
  sel.appendChild(o0);
  for (const s of rows) {
    const o = document.createElement("option");
    o.value = s.name;
    const side = s.arm === "left" ? "左" : (s.arm === "right" ? "右" : "?");
    const rel = `data/validate/${s.name}.json`;
    o.textContent = `${rel} · ${side} ${s.poseset || ""}`;
    o.title = s.file || rel;
    sel.appendChild(o);
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
  $("btn-id-measure").disabled = !allow || !!id.running;
  $("btn-id-stop").disabled = !id.running;
  $("btn-val-start").disabled = !allow || !!id.running;
  renderVal(id);
  renderPayloadSelect(id.profiles || [], id.applied_name);
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
  renderPoseList(d.list || []);
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
    window.speedDirty = false;
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
  if (!sel) return;
  if (document.activeElement === sel) return;
  const keep = sel.value;
  let rows = list || [];
  if (window.currentSet) {
    const arm = window.currentSet.arm;
    const name = window.currentSet.name;
    const filtered = rows.filter((s) => s.arm === arm && s.poseset === name);
    if (filtered.length) rows = filtered;
  }
  sel.innerHTML = "";
  const o0 = document.createElement("option");
  o0.value = "";
  o0.textContent = rows.length ? "选择测量会话再求解…" : "还没有测量会话（先点测量当前集）";
  sel.appendChild(o0);
  for (const s of rows) {
    const o = document.createElement("option");
    o.value = s.name;
    const side = s.arm === "left" ? "左" : (s.arm === "right" ? "右" : "?");
    const bi = s.bidirectional ? "双向" : "单向";
    o.textContent = `${s.name} · ${side} ${s.poseset || ""} ${s.n}样 ${bi}`;
    sel.appendChild(o);
  }
  const want = current || keep;
  if (want && [...sel.options].some((o) => o.value === want)) sel.value = want;
}

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
    const name = ($("payload-new").value.trim() || $("payload-select").value || (window.currentSet && window.currentSet.name) || "").trim();
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
  const name = ($("payload-new").value.trim() || $("payload-select").value || (window.currentSet && window.currentSet.name) || "").trim();
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
$("btn-val-start").onclick = async () => {
  if (!posesetName()) { flash("先选姿态集", false); return; }
  const modes = [];
  if ($("val-base").checked) modes.push("baseline");
  if ($("val-pay").checked) modes.push("payload");
  if ($("val-corr").checked) modes.push("payload_correct");
  if (!modes.length) { flash("至少勾一种方案", false); return; }
  if ((modes.includes("payload") || modes.includes("payload_correct")) && !$("payload-select").value) {
    flash("补偿方案请先选补偿文件", false); return;
  }
  if (!confirm("将按所选方案走完整姿态集（同一限速）。扶住、守急停。继续？")) return;
  try {
    await post("/api/validate/start", {
      name: posesetName(),
      modes,
      payload: $("payload-select").value,
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

