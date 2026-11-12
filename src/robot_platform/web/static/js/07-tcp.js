// 07-tcp.js —— TCP：管理表、录入与求解、修正（新建 / 平移 / 覆盖 / 另存）、工作姿态、TCP 点动
// 由 web/static/js/ 下各文件按文件名顺序拼成 /static/app.js（同一个脚本作用域）。

function tcpOnWrist(tcp, arms) {
  const offset = tcp && tcp.result && tcp.result.tcp_m;
  const scene = tcp && tcp.arm === "left" ? arms && arms.left : arms && arms.right;
  const T = scene && scene.wrist_T;
  if (!offset || !T) return null;
  return [0, 1, 2].map((i) => T[i][3] + T[i][0] * offset[0] + T[i][1] * offset[1] + T[i][2] * offset[2]);
}

// 工作姿态自由度的短标签：可平转 = 绕重力 z 不约束
function wpFreeTag(free) {
  return free === "gravity_z" ? "（可平转）" : "";
}

function renderWorkPose(wp, tcp) {
  if (!wp) return;
  window.lastWorkPose = wp;
  if (wp.saved_all) window.lastWorkPoseAll = wp.saved_all;
  const badge = $("wp-lock-badge");
  if (badge) {
    badge.textContent = wp.locked ? "已锁定" : "";
    badge.classList.toggle("locked", !!wp.locked);
  }
  const now = $("wp-now");
  if (now) {
    now.textContent = wp.locked && wp.active_name
      ? `锁定：${wp.active_name}${wpFreeTag(wp.free)}${wp.note ? " · " + wp.note : ""}`
      : "锁定：无";
  }
  const hint = $("wp-tcp-hint");
  const tcpName = (tcp && tcp.active_name) || "";
  if (hint) {
    hint.textContent = tcpName
      ? `TCP：${tcpName}${tcp.arm ? `（${tcp.arm === "left" ? "左" : "右"}）` : ""}`
      : "TCP：—（先在 TCP 卡片调用一套）";
  }
  const rows = (wp.saved || []).map((s) => ({
    value: s.name,
    label: `${s.name}${wpFreeTag(s.free)}${s.note ? " · " + s.note : ""}`,
  }));
  const placeholder = tcpName
    ? (rows.length ? "选择工作姿态…" : "这个 TCP 还没有工作姿态")
    : "先调用 TCP";
  const preferred = (wp.locked && wp.active_name) || "";
  const sel = $("wp-select");
  if (sel && document.activeElement !== sel) {
    writeOptions(sel, placeholder, rows, preferred || sel.value || "");
  }
  const wfSel = $("wf-dg-wp");
  if (wfSel && document.activeElement !== wfSel) {
    writeOptions(wfSel, placeholder, rows, preferred || wfSel.value || "");
  }
}

async function workPosePost(path, body) {
  const d = await post(path, body || {});
  renderWorkPose(d, { active_name: d.tcp_name || window.lastTcpName, arm: d.arm });
  if (d.runtime) renderRuntimeConfig(d.runtime);
  return d;
}

function renderTcpTable(tcp) {
  const body = $("tcp-table-body");
  const empty = $("tcp-table-empty");
  if (!body) return;
  const saved = (tcp && tcp.saved) || [];
  window.lastTcpSaved = saved;
  const activeL = (tcp && tcp.active_left) || "";
  const activeR = (tcp && tcp.active_right) || "";
  body.innerHTML = "";
  if (empty) empty.classList.toggle("show", saved.length === 0);
  saved.forEach((s) => {
    const name = s.name || "";
    const arm = s.arm === "left" || s.arm === "right" ? s.arm : "";
    const side = arm === "left" ? "左" : arm === "right" ? "右" : "—";
    const on = (arm === "left" && name === activeL) || (arm === "right" && name === activeR);
    const tr = document.createElement("tr");
    tr.dataset.name = name;
    tr.dataset.arm = arm;
    tr.innerHTML = `
      <td class="tcp-name"></td>
      <td>${side}</td>
      <td class="tcp-note"></td>
      <td><button type="button" class="tcp-act tcp-apply${on ? " on" : ""}" data-act="apply" title="${on ? "当前应用" : "设为该手当前 TCP"}">${on ? "✓" : "○"}</button></td>
      <td><button type="button" class="tcp-act tcp-mirror" data-act="mirror" ${arm ? "" : "disabled"}>镜像</button></td>
      <td><button type="button" class="tcp-act tcp-del" data-act="delete">删除</button></td>
    `;
    tr.querySelector(".tcp-name").textContent = name;
    tr.querySelector(".tcp-note").textContent = s.note || "—";
    body.appendChild(tr);
  });
}

// ---------- TCP 修正：新建（腕末端中心）、沿腕三轴平移、覆盖 / 另存 ----------
window.tcpEdit = null;

 // { name, arm, base: [mm×3], cur: [mm×3] }
function tcpEditText() {
  const e = window.tcpEdit;
  if (!e) return "选一个 TCP 开始修正";
  const f = (v) => v.map((x) => Number(x).toFixed(1)).join(", ");
  const d = e.cur.map((v, i) => v - e.base[i]);
  const side = e.arm === "left" ? "左臂" : "右臂";
  const moved = d.some((v) => Math.abs(v) > 1e-9);
  return `「${e.name}」· ${side} · 原偏置 [${f(e.base)}] mm` +
    (moved ? ` → 新偏置 [${f(e.cur)}] mm（变化 [${d.map((v) => (v >= 0 ? "+" : "") + v.toFixed(1)).join(", ")}]，未保存）` : "，未修改");
}

function renderTcpEdit(tcp, arms) {
  const sel = $("tcp-edit-select");
  if (!sel) return;
  const saved = (tcp && tcp.saved) || [];
  if (document.activeElement !== sel) {
    const want = (window.tcpEdit && window.tcpEdit.name) || sel.value;
    writeOptions(sel, saved.length ? "选择要修正的 TCP…" : "还没有 TCP",
      saved.filter((s) => s.arm === "left" || s.arm === "right").map((s) => ({
        value: s.name,
        label: `${s.name} · ${s.arm === "left" ? "左" : "右"}${s.tcp_mm ? ` · [${s.tcp_mm.map((v) => Number(v).toFixed(0)).join(", ")}] mm` : ""}`,
      })), want);
  }
  // 选中的 TCP 被别处改了（例如另一页覆盖）、且这里没改动：跟上
  const e = window.tcpEdit;
  if (e) {
    const s = saved.find((x) => x.name === e.name);
    if (!s) { window.tcpEdit = null; }
    else if (s.tcp_mm && e.cur.every((v, i) => v === e.base[i])) {
      e.base = s.tcp_mm.slice(); e.cur = s.tcp_mm.slice();
    }
  }
  if ($("tcp-edit-now")) $("tcp-edit-now").textContent = tcpEditText();
  if (arms) window.tcpEditArms = arms;
  updateTcpPreview();
}

function updateTcpPreview() {
  const e = window.tcpEdit;
  const arms = window.tcpEditArms;
  const view = window.RobotView;
  if (!view || !view.setTcpPreview) return;
  if (!e || !arms) { view.setTcpPreview(null); return; }
  const pt = tcpOnWrist({ arm: e.arm, result: { tcp_m: e.cur.map((v) => v / 1000) } }, arms);
  view.setTcpPreview(pt);
}

function selectTcpEdit(name) {
  const s = (window.lastTcpSaved || []).find((x) => x.name === name);
  if (!s) { window.tcpEdit = null; }
  else {
    const mm = (s.tcp_mm || [0, 0, 0]).slice();
    window.tcpEdit = { name: s.name, arm: s.arm, base: mm.slice(), cur: mm.slice() };
  }
  if ($("tcp-edit-now")) $("tcp-edit-now").textContent = tcpEditText();
  updateTcpPreview();
}

function renderTcp(tcp, arms) {
  const note = $("tcp-now");
  if (!note || !tcp) return;
  window.lastTcpName = tcp.active_name || "";
  window.lastTcpArm = tcp.arm || "";
  // 每只手各自生效的 TCP（点动 / 碰点 / 棋盘触摸按卡片选的手取）
  window.lastTcpActive = { left: tcp.active_left || "", right: tcp.active_right || "" };
  renderTcpTable(tcp);
  const jogNow = $("tcp-jog-now");
  if (jogNow) {
    const arm = armPick("jog");
    const name = window.lastTcpActive[arm];
    jogNow.textContent = name
      ? `${ARM_SIDE[arm]}臂 TCP「${name}」· 躯干系点动`
      : `${ARM_SIDE[arm]}臂还没调用 TCP（先在标定册 TCP 卡片调用）`;
  }
  const side = tcp.arm === "left" ? "左臂" : tcp.arm === "right" ? "右臂" : "";
  let text = tcp.active_name ? `会话已加载「${tcp.active_name}」。` : "";
  text += side ? `${side}已记 ${tcp.n} 个姿态` : "还没有姿态";
  if (tcp.note) text += `。${tcp.note}`;
  const r = tcp.result;
  if (r) {
    const mm = (arr) => arr.map((v) => Number(v).toFixed(1)).join(", ");
    const res = r.residual_mm || {};
    text += `。工作点相对腕 [${mm(r.tcp_mm || [0, 0, 0])}] mm。`;
    if (r.fixed_mm) text += `固定点 [${mm(r.fixed_mm)}] mm。`;
    if (r.residual_mm) text += `残差平均 ${Number(res.mean).toFixed(1)}、最大 ${Number(res.max).toFixed(1)} mm。`;
    if (r.manual) text += "（手动设定 / 修正，无标定残差）";
    if (r.warning) text += r.warning;
  }
  note.textContent = text;
  renderTcpEdit(tcp, arms);
  if (!arms) return;
  const live = tcpOnWrist(tcp, arms);
  const key = live ? live.map((v) => Number(v).toFixed(4)).join(",") : "";
  if (key === window.tcpPointKey) return;
  if (window.RobotView && window.RobotView.setFixedPoint && window.RobotView.setFixedPoint(live || null)) {
    window.tcpPointKey = key;
  }
}

async function tcpPost(path, body) {
  const d = await post(path, body || {});
  renderTcp(d);
  if (d.work_pose) renderWorkPose(d.work_pose, d);
  if (d.runtime) renderRuntimeConfig(d.runtime);
  return d;
}

async function tcpApplySaved(name) {
  if (!name) return;
  try {
    const d = await tcpPost("/api/tcp/load", { name });
    flash(`已应用「${name}」为${d.arm === "left" ? "左" : "右"}臂当前 TCP`, true);
  } catch (e) { flash(String(e), false); }
}

async function tcpMirrorSaved(name) {
  if (!name) return;
  const row = (window.lastTcpSaved || []).find((s) => s.name === name);
  const dst = row && row.arm === "left" ? "右" : "左";
  if (!confirm(`把「${name}」关于躯干 y=0 镜像成${dst}臂？新名为「${name}_${dst}」，不含工作姿态。`)) return;
  try {
    const d = await tcpPost("/api/tcp/mirror", { name });
    flash(d.message || `已镜像为「${d.mirrored_name}」`, true);
  } catch (e) {
    if (String(e).includes("已存在") && confirm(`对侧「${name}_${dst}」已存在，覆盖？`)) {
      try {
        const d = await tcpPost("/api/tcp/mirror", { name, force: true });
        flash(d.message || `已覆盖镜像「${d.mirrored_name}」`, true);
      } catch (e2) { flash(String(e2), false); }
    } else flash(String(e), false);
  }
}

async function tcpDeleteSaved(name) {
  if (!name) return;
  if (!confirm(`删除 TCP「${name}」？将取消该手应用，并删除其下全部工作姿态。`)) return;
  try {
    const d = await tcpPost("/api/tcp/delete", { name });
    flash(d.message || "已删除", true);
  } catch (e) { flash(String(e), false); }
}

if ($("tcp-table-body")) {
  $("tcp-table-body").addEventListener("click", (ev) => {
    const btn = ev.target.closest("button[data-act]");
    if (!btn) return;
    const tr = btn.closest("tr");
    const name = tr?.dataset?.name || "";
    if (!name) return;
    if (btn.dataset.act === "apply") tcpApplySaved(name);
    else if (btn.dataset.act === "mirror") tcpMirrorSaved(name);
    else if (btn.dataset.act === "delete") tcpDeleteSaved(name);
  });
}

if ($("btn-wp-save")) {
  $("btn-wp-save").onclick = async () => {
    const name = $("wp-name").value.trim() || $("wp-select").value;
    if (!name) { flash("先给工作姿态起个名字", false); return; }
    if (!window.lastTcpName) { flash("先调用一套 TCP", false); return; }
    const exists = [...($("wp-select")?.options || [])].some((o) => o.value === name);
    if (exists && !confirm(`覆盖工作姿态「${name}」？`)) return;
    try {
      await workPosePost("/api/work-pose/save", {
        arm: window.lastTcpArm,
        name,
        note: $("wp-note")?.value.trim() || "",
        free: $("wp-free-yaw")?.checked ? "gravity_z" : "",
      });
      if ($("wp-name")) $("wp-name").value = "";
      flash(`已记下${$("wp-free-yaw")?.checked ? "可平转" : ""}工作姿态「${name}」`, true);
    } catch (e) { flash(String(e), false); }
  };
}

if ($("btn-wp-load")) {
  $("btn-wp-load").onclick = async () => {
    const name = $("wp-select")?.value;
    if (!name) { flash("先选一个工作姿态", false); return; }
    if (!window.lastTcpName) { flash("先调用一套 TCP", false); return; }
    const busy = armBusyReason(window.lastTcpArm);
    if (busy) { flash(busy, false); return; }
    if (!confirm(`将把${ARM_SIDE[window.lastTcpArm] || ""}臂转到「${name}」并锁定腕部方向（尽量保持当前 TCP 位置）。扶住、守急停。继续？`)) return;
    const btn = $("btn-wp-load");
    if (btn) btn.disabled = true;
    flash("正在转到工作姿态…", true);
    try {
      const d = await workPosePost("/api/work-pose/load", {
        name,
        tcp_name: window.lastTcpName,
        arm: window.lastTcpArm,
      });
      flash(
        d.message
          || `已锁定「${name}」· 位置 ${Number(d.measured_miss_mm || 0).toFixed(1)} mm · 姿态 ${Number(d.measured_ori_deg || 0).toFixed(1)}°`,
        true,
      );
    } catch (e) { flash(String(e), false); }
    finally { if (btn) btn.disabled = false; }
  };
}

if ($("btn-wp-clear")) {
  $("btn-wp-clear").onclick = async () => {
    try {
      await workPosePost("/api/work-pose/clear", {});
      flash("已解锁工作姿态", true);
    } catch (e) { flash(String(e), false); }
  };
}

if ($("btn-wp-del")) {
  $("btn-wp-del").onclick = async () => {
    const name = $("wp-select")?.value;
    if (!name) { flash("先选一个工作姿态", false); return; }
    if (!window.lastTcpName) { flash("先调用一套 TCP", false); return; }
    if (!confirm(`删除工作姿态「${name}」？`)) return;
    try {
      await workPosePost("/api/work-pose/delete", { name, tcp_name: window.lastTcpName });
      flash("已删除", true);
    } catch (e) { flash(String(e), false); }
  };
}

$("btn-tcp-record").onclick = async () => {
  try {
    const d = await tcpPost("/api/tcp/record", { arm: armPick("tcp") });
    const side = d.arm === "left" ? "左" : "右";
    flash(`已记${side}臂第 ${d.n} 个姿态`, true);
  } catch (e) { flash(String(e), false); }
};

$("btn-tcp-undo").onclick = async () => {
  try { await tcpPost("/api/tcp/undo"); } catch (e) { flash(String(e), false); }
};

$("btn-tcp-clear").onclick = async () => {
  try { await tcpPost("/api/tcp/clear"); flash("已清空 TCP 记录", true); }
  catch (e) { flash(String(e), false); }
};

$("btn-tcp-solve").onclick = async () => {
  try {
    const d = await tcpPost("/api/tcp/solve");
    const res = (d.result && d.result.residual_mm) || {};
    flash(`TCP 已算出，最大残差 ${Number(res.max).toFixed(1)} mm`, true);
  } catch (e) { flash(String(e), false); }
};

$("tcp-edit-select").onchange = () => selectTcpEdit($("tcp-edit-select").value);

$("btn-tcp-new").onclick = async () => {
  const name = $("tcp-new-name").value.trim();
  if (!name) { flash("给新 TCP 起个名字", false); return; }
  const arm = $("tcp-new-arm").value;
  const exists = (window.lastTcpSaved || []).some((s) => s.name === name);
  if (exists && !confirm(`TCP「${name}」已存在，覆盖成腕末端中心（偏置 0）？`)) return;
  try {
    const d = await post("/api/tcp/new-flange", { name, arm, force: exists });
    renderTcp(d);
    $("tcp-new-name").value = "";
    selectTcpEdit(name);
    flash(d.message || `已新建「${name}」`, true);
  } catch (e) { flash(String(e), false); }
};

$("btn-tcp-edit-move").onclick = () => {
  const e = window.tcpEdit;
  if (!e) { flash("先在下拉框选一个 TCP", false); return; }
  const d = ["tcp-edit-dx", "tcp-edit-dy", "tcp-edit-dz"].map((id) => parseFloat($(id).value) || 0);
  if (d.every((v) => v === 0)) { flash("填平移距离（毫米）", false); return; }
  if (d.some((v) => Math.abs(v) > 200)) { flash("单次平移每轴不超过 200 mm", false); return; }
  e.cur = e.cur.map((v, i) => Math.round((v + d[i]) * 1000) / 1000);
  $("tcp-edit-now").textContent = tcpEditText();
  updateTcpPreview();
};

$("btn-tcp-edit-reset").onclick = () => {
  const e = window.tcpEdit;
  if (!e) return;
  e.cur = e.base.slice();
  $("tcp-edit-now").textContent = tcpEditText();
  updateTcpPreview();
};

async function tcpEditSave(mode) {
  const e = window.tcpEdit;
  if (!e) { flash("先在下拉框选一个 TCP", false); return; }
  const body = { src: e.name, tcp_mm: e.cur, mode };
  if (mode === "saveas") {
    body.name = $("tcp-edit-newname").value.trim();
    if (!body.name) { flash("另存要填新名字", false); return; }
    if (body.name === e.name) { flash("另存要换个名字；改原名请用覆盖保存", false); return; }
    const exists = (window.lastTcpSaved || []).some((s) => s.name === body.name);
    if (exists && !confirm(`TCP「${body.name}」已存在，覆盖？`)) return;
    body.force = exists;
  } else if (!confirm(`覆盖「${e.name}」的偏置为 [${e.cur.map((v) => v.toFixed(1)).join(", ")}] mm？`)) {
    return;
  }
  try {
    const d = await post("/api/tcp/edit-save", body);
    renderTcp(d);
    if (mode === "saveas") $("tcp-edit-newname").value = "";
    selectTcpEdit(d.saved_name || e.name);
    flash(d.message || "已保存", true);
  } catch (err) { flash(String(err), false); }
}

$("btn-tcp-edit-overwrite").onclick = () => tcpEditSave("overwrite");

$("btn-tcp-edit-saveas").onclick = () => tcpEditSave("saveas");

$("btn-tcp-save").onclick = async () => {
  const name = $("tcp-name").value.trim();
  if (!name) { flash("填一个名字", false); return; }
  const exists = (window.lastTcpSaved || []).some((s) => s.name === name);
  if (exists && !confirm(`覆盖已保存的「${name}」？`)) return;
  try {
    await tcpPost("/api/tcp/save", { name, note: $("tcp-note").value.trim() });
    $("tcp-name").value = "";
    flash(`已保存「${name}」`, true);
  } catch (e) { flash(String(e), false); }
};

async function tcpJogStep(mode, axis, sign) {
  if (tcpJogBusy) return;
  const arm = armPick("jog");
  if (!(window.lastTcpActive || {})[arm]) { flash(`先为${ARM_SIDE[arm]}臂调用一套 TCP`, false); return; }
  const busy = armBusyReason(arm);
  if (busy) { flash(busy, false); return; }
  const stepMm = Number($("tcp-jog-step-mm")?.value);
  const stepDeg = Number($("tcp-jog-step-deg")?.value);
  const body = { mode, axis, sign: Number(sign), arm };
  if (mode === "translate") {
    if (!Number.isFinite(stepMm) || stepMm <= 0) { flash("平移步长无效", false); return; }
    body.step_mm = stepMm;
  } else {
    if (!Number.isFinite(stepDeg) || stepDeg <= 0) { flash("旋转步长无效", false); return; }
    body.step_deg = stepDeg;
  }
  tcpJogBusy = true;
  document.querySelectorAll(".tcp-jog-btn").forEach((b) => { b.disabled = true; });
  try {
    const d = await post("/api/tcp/jog", body);
    if (d.work_pose) {
      renderWorkPose(d.work_pose, { active_name: window.lastTcpName, arm: d.arm });
    }
    if (d.runtime) renderRuntimeConfig(d.runtime);
    flash(d.message || "已发出 TCP 点动", true);
  } catch (e) {
    flash(String(e), false);
  } finally {
    tcpJogBusy = false;
    const ctrl = window.lastCtrl;
    const allow = !!(ctrl && ctrl.allow_real_motion);
    document.querySelectorAll(".tcp-jog-btn").forEach((b) => {
      b.disabled = !allow || !ctrl?.engaged || !!ctrl?.float;
    });
  }
}

document.querySelectorAll(".tcp-jog-btn").forEach((btn) => {
  btn.addEventListener("click", () => {
    tcpJogStep(btn.dataset.jog, btn.dataset.axis, btn.dataset.sign);
  });
});
