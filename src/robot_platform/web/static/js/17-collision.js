// 17-collision.js —— 控制页「碰撞检查」：开关、安全余量、当前最近的两处、上次拦截；三维里画碰撞体

const colUi = { st: null, geomShown: false, marginDirty: false };

function colFmtMm(v) {
  return v == null || !Number.isFinite(Number(v)) ? "—" : `${Number(v).toFixed(0)} mm`;
}

function renderCollision(st) {
  colUi.st = st;
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
  const toolKey = JSON.stringify(tools);
  if (colUi.toolKey !== undefined && colUi.toolKey !== toolKey && colUi.geomShown) collisionShowGeometry(true);
  colUi.toolKey = toolKey;
  if (st.last_check && st.last_check.checked) parts.push(`上次检查 ${Number(st.last_check.ms).toFixed(0)} ms`);
  if (now) {
    now.textContent = parts.join(" · ");
    const n = st.nearest;
    now.className = `note${!st.enabled ? " bad" : n && n.distance_mm < st.margin_mm ? " bad" : ""}`;
  }
  if (last) {
    const b = st.last_block;
    last.textContent = b
      ? `上次拦下（${b.time}）：${b.arm === "left" ? "左" : "右"}手 ${b.pair_text}，${b.distance_mm < 0 ? "穿透" : "间距"} ${Math.abs(b.distance_mm).toFixed(0)} mm`
      : "还没有拦过";
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

// 卡片看得见、或三维在画碰撞体时每秒刷一次
setInterval(() => {
  const panel = $("panel-collision");
  const visible = panel && panel.offsetParent !== null;
  if (document.hidden || !(visible || colUi.geomShown)) return;
  collisionRefresh();
}, 1000);
collisionRefresh();
