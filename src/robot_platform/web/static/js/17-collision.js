// 17-collision.js —— 控制页「碰撞检查」：开关、安全余量、当前最近的两处、上次拦截；三维里画碰撞体

const colUi = { st: null, geomShown: false, marginDirty: false, cfgDirty: false };

// 绕行规划结果的一句话（到位提示、卡片里共用）
function collisionPlanText(p) {
  if (!p) return "";
  const n = Number(p.waypoints || 0);
  const s = `${n ? `${n} 个中间点` : "直连"}，规划 ${(Number(p.ms || 0) / 1000).toFixed(1)} s`;
  return p.planner ? `${s}（${p.backend === "coal" ? "coal" : "内置"} + ${p.planner === "ompl" ? "OMPL" : "内置 RRT"}）` : s;
}

// 「配置」里的碰撞检测后端 / 绕行规划器 / 绕行开关
function colFillSelect(sel, items, cur) {
  if (!sel || !items) return;
  const key = JSON.stringify(items);
  if (sel.dataset.key !== key) {
    sel.innerHTML = "";
    for (const it of items) {
      const o = document.createElement("option");
      o.value = it.id;
      o.textContent = it.available ? it.label : `${it.label}（本机没装）`;
      o.disabled = !it.available;
      sel.appendChild(o);
    }
    sel.dataset.key = key;
  }
  if (!colUi.cfgDirty && document.activeElement !== sel) sel.value = cur;
}

function renderCollisionCfg(st) {
  const note = $("cfg-col-note");
  const ids = ["cfg-col-backend", "cfg-col-planner", "cfg-col-detour", "btn-cfg-col"];
  if (!st || !st.available) {
    ids.forEach((id) => { if ($(id)) $(id).disabled = true; });
    if (note) note.textContent = `碰撞检测：${st?.note || "没有碰撞模型"}`;
    return;
  }
  ids.forEach((id) => { if ($(id)) $(id).disabled = false; });
  colFillSelect($("cfg-col-backend"), st.backends, st.backend);
  colFillSelect($("cfg-col-planner"), st.planners, st.planner);
  const det = $("cfg-col-detour");
  if (det && !colUi.cfgDirty && document.activeElement !== det) det.checked = st.detour !== false;
  if (note) {
    const bl = (st.backends || []).find((b) => b.id === st.backend_active);
    const parts = [`在用：${bl ? bl.label : st.backend_active}`];
    if (st.backend !== st.backend_active) parts.push(`选的「${st.backend}」本机用不了，已退回内置`);
    parts.push(st.detour !== false ? "到位类目标直线会碰时自动绕行" : "只拦不绕");
    const p = st.last_plan;
    if (p) parts.push(p.ok ? `上次绕行（${p.time}）：${collisionPlanText(p)}` : `上次绕行失败（${p.time}）：${p.reason}`);
    note.textContent = parts.join(" · ");
  }
}

async function collisionApplyCfg() {
  const body = {
    backend: $("cfg-col-backend")?.value,
    planner: $("cfg-col-planner")?.value,
    detour: !!$("cfg-col-detour")?.checked,
  };
  const btn = $("btn-cfg-col");
  if (btn) btn.disabled = true;
  try {
    const d = await post("/api/collision/config", body);
    colUi.cfgDirty = false;
    flash(d.ok ? d.message : (d.error || "设置失败"), !!d.ok);
    renderCollision(d.collision);
  } catch (e) { flash(String(e.message || e), false); }
  finally { if (btn) btn.disabled = false; }
}

function colFmtMm(v) {
  return v == null || !Number.isFinite(Number(v)) ? "—" : `${Number(v).toFixed(0)} mm`;
}

function renderCollision(st) {
  colUi.st = st;
  renderCollisionCfg(st);
  const now = $("col-now");
  const last = $("col-last");
  const badge = $("col-badge");
  if (!st) return;
  if (!st.available) {
    if (now) { now.textContent = st.note || st.error || "没有碰撞模型"; now.className = "note bad"; }
    if (badge) badge.textContent = "无模型";
    ["col-on", "col-margin", "col-show"].forEach((id) => { if ($(id)) $(id).disabled = true; });
    return;
  }
  ["col-on", "col-margin", "col-show"].forEach((id) => { if ($(id)) $(id).disabled = false; });
  if ($("col-on") && document.activeElement !== $("col-on")) $("col-on").checked = !!st.enabled;
  if ($("col-margin") && document.activeElement !== $("col-margin") && !colUi.marginDirty) {
    $("col-margin").value = String(Math.round(st.margin_mm));
    $("col-margin-val").textContent = String(Math.round(st.margin_mm));
    colInflate(st.margin_mm);
  }
  const parts = [st.enabled ? `已开 · 余量 ${Math.round(st.margin_mm)} mm` : "已关（手臂目标不查自碰撞）"];
  if (st.gate_attached === false) parts.unshift("⚠ 控制循环没挂上碰撞检查，下目标时不会拦（重启服务）");
  const nClose = (st.close || []).length;
  if (nClose) parts.push(`${nClose} 处已比余量近（三维标红）`);
  const per = st.per_arm || {};
  for (const [arm, label] of [["left", "左手"], ["right", "右手"]]) {
    const p = per[arm];
    if (!p) continue;
    const d = Number(p.distance_mm);
    parts.push(d >= 149 ? `${label}离最近处 >150 mm` : `${label}最近：${p.pair_text} ${colFmtMm(d)}`);
  }
  // 末端胶囊体跟着生效的 TCP：没生效的手就只有腕
  const tools = st.tools || [];
  const toolSides = new Set(tools.map((t) => t.side));
  const noTool = [["left", "左手"], ["right", "右手"]].filter(([a]) => !toolSides.has(a)).map(([, l]) => l);
  if (noTool.length) parts.push(`${noTool.join("、")}没有生效 TCP，末端不算`);
  // 末端变了、或换了碰撞后端（内置画球 / 胶囊，coal 画凸包），三维里的碰撞体重取
  const toolKey = JSON.stringify([tools, st.backend_active]);
  if (colUi.toolKey !== undefined && colUi.toolKey !== toolKey && colUi.geomShown) collisionShowGeometry(true);
  colUi.toolKey = toolKey;
  if (st.last_check && st.last_check.checked) parts.push(`上次检查 ${Number(st.last_check.ms).toFixed(0)} ms`);
  parts.push(st.backend_active === "coal" ? "coal 后端" : "内置后端");
  if (now) {
    now.textContent = parts.join(" · ");
    const n = st.nearest;
    now.className = `note${!st.enabled || st.gate_attached === false ? " bad" : n && n.distance_mm < st.margin_mm ? " bad" : ""}`;
  }
  if (last) {
    const b = st.last_block;
    last.textContent = b
      ? `上次拦下（${b.time}）：${b.arm === "left" ? "左" : "右"}手 ${b.pair_text}，${b.distance_mm < 0 ? "穿透" : "间距"} ${Math.abs(b.distance_mm).toFixed(0)} mm`
      : "还没有拦过";
    const p = st.last_plan;
    if (p) last.textContent += p.ok ? ` · 上次绕行（${p.time}）：${collisionPlanText(p)}` : ` · 上次绕行失败（${p.time}）：${p.reason}`;
  }
  if (badge) badge.textContent = st.enabled ? "" : "已关";
  if (colUi.geomShown && window.RobotView?.setCollisionHighlight) {
    // 比余量近的都标红；刚被拦的那对再标 8 s
    const hot = [];
    for (const c of st.close || []) hot.push(...c.pair);
    const b = st.last_block;
    if (b && b.time && colUi.lastBlockSeen !== b.time) { colUi.lastBlockSeen = b.time; colUi.blockUntil = Date.now() + 8000; colUi.blockPair = b.pair; }
    if (colUi.blockUntil && Date.now() < colUi.blockUntil) hot.push(...(colUi.blockPair || []));
    window.RobotView.setCollisionHighlight(hot);
  }
}

// 三维碰撞体按「余量的一半」加厚：两块碰到一起 = 间距刚好到余量。拖滑条时实时跟着变（40 ms 合并一次）
function colInflate(marginMm) {
  const m = Number(marginMm) / 2000;
  if (colUi.inflate === m) return;
  colUi.inflate = m;
  if (colUi.raf) return;
  colUi.raf = setTimeout(() => {
    colUi.raf = 0;
    if (window.RobotView?.setCollisionInflate) window.RobotView.setCollisionInflate(colUi.inflate);
  }, 40);
}

async function collisionRefresh() {
  try {
    const d = await post("/api/collision/status", {});
    renderCollision(d.collision);
  } catch (e) { /* 轮询失败不打扰 */ }
}

async function collisionConfig(body) {
  try {
    const d = await post("/api/collision/config", body);
    flash(d.ok ? d.message : (d.error || "设置失败"), !!d.ok);
    colUi.marginDirty = false;
    renderCollision(d.collision);
  } catch (e) { flash(String(e.message || e), false); }
}

async function collisionShowGeometry(on) {
  colUi.geomShown = !!on;
  if (!window.RobotView?.setCollision) return;
  if (!on) { window.RobotView.setCollision(null); return; }
  try {
    const d = await post("/api/collision/geometry", {});
    if (!d.ok) { flash(d.error || "取碰撞体失败", false); return; }
    if (window.RobotView.setCollisionInflate) {
      colUi.inflate = Number($("col-margin")?.value || 0) / 2000;
      window.RobotView._collisionInflate = colUi.inflate;
    }
    if (!window.RobotView.setCollision(d)) flash("三维还没准备好，稍后再勾", false);
    collisionRefresh();
  } catch (e) { flash(String(e.message || e), false); }
}

if ($("col-on")) {
  $("col-on").addEventListener("change", () => {
    const on = $("col-on").checked;
    if (!on && !confirm("关掉碰撞检查后，手臂目标不再检查会不会撞到自己。确定关掉？")) { $("col-on").checked = true; return; }
    collisionConfig({ enabled: on });
  });
}
if ($("col-margin")) {
  $("col-margin").addEventListener("input", () => {
    colUi.marginDirty = true;
    $("col-margin-val").textContent = $("col-margin").value;
    colInflate($("col-margin").value);
  });
  $("col-margin").addEventListener("change", () => collisionConfig({ margin_mm: Number($("col-margin").value) }));
}
if ($("col-show")) $("col-show").addEventListener("change", () => collisionShowGeometry($("col-show").checked));
["cfg-col-backend", "cfg-col-planner", "cfg-col-detour"].forEach((id) => {
  if ($(id)) $(id).addEventListener("change", () => { colUi.cfgDirty = true; });
});
if ($("btn-cfg-col")) $("btn-cfg-col").addEventListener("click", collisionApplyCfg);

// 卡片看得见、或三维在画碰撞体时每秒刷一次
setInterval(() => {
  const panel = $("panel-collision");
  const cfg = $("cfg-col-note");
  const visible = (panel && panel.offsetParent !== null) || (cfg && cfg.offsetParent !== null);
  if (document.hidden || !(visible || colUi.geomShown)) return;
  collisionRefresh();
}, 1000);
collisionRefresh();
