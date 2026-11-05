// 09-camera-handeye.js —— 相机卡片（双目 / RGB-D、彩色内参、预览）与手眼标定（采集、求解、保存、调用）
// 由 web/static/js/ 下各文件按文件名顺序拼成 /static/app.js（同一个脚本作用域）。

function heForm() {
  return {
    session: $("he-session").value,
    square_size_mm: $("he-square").value,
    eye: $("he-eye").value,
    board_size: $("he-board").value.trim() || "11x8",
    camera_host: $("he-host").value.trim(),
  };
}

function renderHeTable(st) {
  const body = $("he-table-body");
  const empty = $("he-table-empty");
  if (!body) return;
  const saved = (st && st.saved) || [];
  window.lastHeSaved = saved;
  const active = (st && st.active_result) || "";
  body.innerHTML = "";
  if (empty) empty.classList.toggle("show", saved.length === 0);
  saved.forEach((s) => {
    const name = s.name || "";
    const side = s.arm === "left" ? "左" : s.arm === "right" ? "右" : "—";
    const base = s.baseline_mm != null && Number.isFinite(Number(s.baseline_mm))
      ? `${Number(s.baseline_mm).toFixed(1)} mm`
      : "—";
    const on = !!name && name === active;
    const tr = document.createElement("tr");
    tr.dataset.name = name;
    tr.innerHTML = `
      <td class="he-name"></td>
      <td>${side}</td>
      <td class="he-meta"></td>
      <td><button type="button" class="he-act he-apply${on ? " on" : ""}" data-act="apply" title="${on ? "当前应用" : "设为当前手眼"}">${on ? "✓" : "○"}</button></td>
      <td><button type="button" class="he-act he-del" data-act="delete">删除</button></td>
    `;
    tr.querySelector(".he-name").textContent = name;
    tr.querySelector(".he-meta").textContent = base;
    body.appendChild(tr);
  });
}

// 相机类型：双目（G1D teleimager）或 RGB-D（G1 D435i）。按它调整「双目」卡片与手眼表单
function applyCameraKind(st) {
  const kind = st && st.camera_kind;
  if (!kind || window.cameraKind === kind) return;
  window.cameraKind = kind;
  const rgbd = kind === "rgbd";
  const toggle = document.querySelector("#panel-cam > .float-toggle");
  if (toggle) toggle.textContent = rgbd ? "相机（RGB-D）" : "双目";
  if ($("cam-preview")) $("cam-preview").alt = rgbd ? "彩色 / 深度" : "左右目";
  if ($("cam-disp")) $("cam-disp").alt = rgbd ? "深度" : "视差";
  if ($("btn-cam-match")) $("btn-cam-match").textContent = "匹配点云";
  if ($("btn-cam-match-debug")) $("btn-cam-match-debug").style.display = rgbd ? "none" : "";
  if ($("cam-intr")) $("cam-intr").style.display = rgbd ? "" : "none";
  const host = $("he-host") && $("he-host").closest("label");
  if (host) host.style.display = rgbd ? "none" : "";
  const eye = $("he-eye");
  if (eye) {
    for (const opt of eye.options) {
      if (opt.value === "left") opt.textContent = rgbd ? "彩色" : "左目";
      else opt.disabled = rgbd;
    }
    if (rgbd) eye.value = "left";
  }
  if ($("he-intro")) {
    $("he-intro").textContent = rgbd
      ? "用姿态集里已经记好的点。采集按限速走完每点停稳，存彩色图与深度图；求解用左侧相机卡片选定的彩色内参（未选为出厂），只算彩色相机到躯干外参。左右对称用姿态集「镜像到对侧」。画面在左侧「相机」。"
      : "用姿态集里已经记好的点。采集按限速走完每点停稳存图；求解先估内参再算相机到躯干外参。左右对称用姿态集「镜像到对侧」。画面在左侧「双目」。";
  }
}

window.applyCameraKind = applyCameraKind;

// 相机下拉框：列出机型 robot.json 里声明的相机，选中的记到实例配置（state/handeye.json 的 camera）
let camSelectBusy = false;
function renderCameraSelect(st) {
  const sel = $("cam-select");
  if (!sel || !Array.isArray(st.cameras) || camSelectBusy) return;
  const sig = JSON.stringify(st.cameras.map((c) => [c.id, c.label]));
  if (sel.dataset.sig !== sig) {
    sel.dataset.sig = sig;
    sel.innerHTML = "";
    for (const c of st.cameras) {
      const o = document.createElement("option");
      o.value = c.id;
      o.textContent = c.label;
      sel.appendChild(o);
    }
    if (!st.cameras.length) {
      const o = document.createElement("option");
      o.value = "";
      o.textContent = st.camera_label || "（机型没有声明相机）";
      sel.appendChild(o);
    }
  }
  // 实例里用旧写法直接指定了来源、且对不上机型里的相机时，显示一项「实例配置」
  const cur = st.camera || "";
  if (cur && st.cameras.length) {
    const stale = [...sel.options].find((o) => o.value === "");
    if (stale) stale.remove();
  }
  if (!cur && st.cameras.length) {
    let extra = [...sel.options].find((o) => o.value === "");
    if (!extra) {
      extra = document.createElement("option");
      extra.value = "";
      sel.prepend(extra);
    }
    extra.textContent = `实例配置：${st.camera_label || st.camera_source}`;
  }
  if (document.activeElement !== sel) sel.value = cur;
  sel.disabled = !!st.config_locked || (st.cameras.length <= 1 && !!cur);
  sel.title = st.config_locked ? "配置已锁定" : `来源 ${st.camera_source}`;
}

if ($("cam-select")) {
  $("cam-select").onchange = async () => {
    const camera = $("cam-select").value;
    if (!camera) return;
    camSelectBusy = true;
    try {
      const d = await post("/api/handeye/config", { camera });
      camSelectBusy = false;
      renderHe(d);
      if ($("cam-preview")) $("cam-preview").removeAttribute("src");
      flash(`相机：${d.camera_label}`, true);
    } catch (e) {
      camSelectBusy = false;
      flash(String(e.message || e), false);
    }
  };
}

// RGB-D 彩色内参：出厂 / 自标定（上传到实例 calib/color_intrinsics）。
// 选中即写入实例配置（cloud_color_intrinsics），相机卡片与渲染卡片的匹配点云、手眼求解都用它；配置锁定时不能改。
let camIntrBusy = false;

function renderColorIntr(st) {
  const sel = $("cam-intr-select");
  if (!sel || !Array.isArray(st.color_intrinsics) || camIntrBusy) return;
  window.colorIntrFiles = st.color_intrinsics;
  window.colorIntrLocked = !!st.config_locked;
  const want = st.cloud_color_intrinsics || "";
  const sig = JSON.stringify(st.color_intrinsics.map((f) => [f.name, f.error || ""]));
  if (sel.dataset.sig !== sig) {
    sel.dataset.sig = sig;
    sel.innerHTML = '<option value="">出厂（camera_info）</option>';
    for (const f of st.color_intrinsics) {
      const o = document.createElement("option");
      o.value = f.name;
      o.textContent = f.error ? `${f.name}（无效）` : `${f.name} · ${f.size[0]}×${f.size[1]}`;
      o.disabled = !!f.error;
      sel.appendChild(o);
    }
  }
  if (document.activeElement !== sel) {
    sel.value = [...sel.options].some((o) => o.value === want && !o.disabled) ? want : "";
  }
  sel.disabled = !!st.config_locked;
  showColorIntrInfo();
}

function showColorIntrInfo() {
  const box = $("cam-intr-info");
  const sel = $("cam-intr-select");
  if (!box || !sel) return;
  const f = (window.colorIntrFiles || []).find((x) => x.name === sel.value);
  if ($("btn-cam-intr-delete")) $("btn-cam-intr-delete").disabled = !sel.value || !!window.colorIntrLocked;
  const lock = window.colorIntrLocked ? "配置已锁定，内参不能改（到「控制 → 配置」解锁）。" : "";
  if (!f) {
    box.textContent = lock + "出厂内参：随 ROS camera_info 走，无畸变系数。";
    return;
  }
  if (f.error) { box.textContent = `文件无效：${f.error}`; return; }
  const rms = f.rms_px == null ? "" : `，重投影 RMS ${Number(f.rms_px).toFixed(3)} px`;
  box.textContent = lock + `${f.size[0]}×${f.size[1]}：fx ${f.fx.toFixed(1)} fy ${f.fy.toFixed(1)} cx ${f.cx.toFixed(1)} cy ${f.cy.toFixed(1)}`
    + `，畸变 [${f.dist.map((v) => v.toFixed(3)).join(", ")}]${rms}。分辨率与当前画面不同时自动换算。只改点云取色位置，点的三维位置仍由深度决定。`;
}

async function selectColorIntr(name) {
  camIntrBusy = true;
  try {
    const d = await post("/api/handeye/config", { color_intrinsics: name });
    camIntrBusy = false;
    renderHe(d);
    flash(name ? `彩色内参：${name}` : "彩色内参：出厂", true);
  } catch (e) {
    camIntrBusy = false;
    flash(String(e.message || e), false);
    refreshHandeye().catch(() => {});  // 退回服务端的选择
  }
}

if ($("cam-intr-select")) {
  $("cam-intr-select").onchange = () => { showColorIntrInfo(); selectColorIntr($("cam-intr-select").value); };
}

if ($("btn-cam-intr-upload")) $("btn-cam-intr-upload").onclick = () => $("cam-intr-file").click();

if ($("cam-intr-file")) {
  $("cam-intr-file").onchange = async (ev) => {
    const file = ev.target.files && ev.target.files[0];
    ev.target.value = "";
    if (!file) return;
    let text;
    try { text = await file.text(); JSON.parse(text); } catch (e) { flash(`读不了 ${file.name}：${e.message}`, false); return; }
    const guess = file.name.replace(/\.json$/i, "").replace(/[^\w\-.一-鿿]/g, "_").slice(0, 48);
    const name = (prompt("内参文件名（中英文、数字、_ - .）", guess) || "").trim();
    if (!name) return;
    const exists = (window.colorIntrFiles || []).some((f) => f.name === name);
    if (exists && !confirm(`内参文件「${name}」已存在，覆盖？`)) return;
    try {
      const d = await post("/api/handeye/color-intrinsics/upload", { name, doc: text, force: exists });
      $("cam-intr-select").dataset.sig = "";
      renderHe(d);
      flash(d.message || "已上传", true);
      if (!d.config_locked && confirm(`改用刚上传的「${d.saved_name}」？`)) await selectColorIntr(d.saved_name);
    } catch (e) {
      flash(String(e.message || e), false);
    }
  };
}

if ($("btn-cam-intr-delete")) {
  $("btn-cam-intr-delete").onclick = async () => {
    const name = $("cam-intr-select").value;
    if (!name || !confirm(`删除内参文件「${name}」？`)) return;
    try {
      const d = await post("/api/handeye/color-intrinsics/delete", { name });
      $("cam-intr-select").value = "";
      renderHe(d);
      flash(d.message || "已删除", true);
    } catch (e) {
      flash(String(e.message || e), false);
    }
  };
}

function renderHe(st) {
  if (!st) return;
  applyCameraKind(st);
  renderCameraSelect(st);
  renderColorIntr(st);
  renderHeTable(st);
  const sel = $("he-session");
  if (sel && document.activeElement !== sel) {
    writeOptions(sel, (st.sessions || []).length ? "选择采集会话…" : "还没有采集会话",
      (st.sessions || []).map((s) => {
        const side = s.arm === "left" ? "左" : "右";
        const ps = s.poseset ? ` · ${s.poseset}` : "";
        return { value: s.name, label: `${s.name} · ${side} ${s.n}张${ps}` };
      }),
      st.session || sel.value);
  }
  if ($("he-count") && !st.running) $("he-count").textContent = st.session ? `${st.count || 0}张` : "";
  const msg = $("he-msg");
  if (msg && document.activeElement !== $("he-host")) {
    msg.textContent = st.error || st.camera_error || (st.running ? (st.phase || "") : "");
  }
  const sq = $("he-square");
  if (sq && document.activeElement !== sq && st.square_size_mm != null && sq.value === "") sq.value = st.square_size_mm;
  if (st.eye && document.activeElement !== $("he-eye")) $("he-eye").value = st.eye;
  if (st.board_size && document.activeElement !== $("he-board")) $("he-board").value = st.board_size;
  if (st.camera_host && document.activeElement !== $("he-host") && $("he-host").value === "") $("he-host").value = st.camera_host;
  const box = $("he-result");
  if (!box) return;
  const res = st.result;
  if (!res) {
    if (!st.intrinsics_fit) return;
  }
  const lines = [];
  const fit = st.intrinsics_fit;
  if (fit && fit.camera === "rgbd") {
    const n = fit.num_pairs != null ? ` · ${fit.num_pairs} 张` : "";
    lines.push(`<div class="stat"><em>本次内参${n}</em><strong>出厂 fx ${Number(fit.left_fx).toFixed(1)} · fy ${Number(fit.left_fy).toFixed(1)}</strong><span class="note">RGB-D 相机，未从图像重新标定</span></div>`);
  } else if (fit && fit.left_fx != null) {
    const rms = fit.left_rms_px != null
      ? `<span class="note">重投影 左 ${Number(fit.left_rms_px).toFixed(3)} / 右 ${Number(fit.right_rms_px).toFixed(3)} / 双目 ${Number(fit.stereo_rms_px).toFixed(3)} px · 基线 ${Number(fit.baseline_mm).toFixed(1)} mm</span>`
      : `<span class="note">基线 ${Number(fit.baseline_mm).toFixed(1)} mm</span>`;
    const n = fit.num_pairs != null ? ` · ${fit.num_pairs} 对` : "";
    lines.push(`<div class="stat"><em>本次内参${n}</em><strong>左 fx ${Number(fit.left_fx).toFixed(1)} · 右 fx ${Number(fit.right_fx).toFixed(1)}</strong>${rms}</div>`);
  }
  if (res) {
    for (const eye of ["left", "right"]) {
      const r = res[eye];
      if (!r) continue;
      const t = r.t_cam2base_m || [];
      const tr = r.residual_translation_mm || {};
      const eyeName = eye === "left" ? (window.cameraKind === "rgbd" ? "彩色" : "左目") : "右目";
      lines.push(`<div class="stat"><em>${eyeName} → ${r.base_link || "torso_link"}</em><strong>${t.map((x) => Number(x).toFixed(4)).join(", ")} m</strong><span class="note">平移残差 均 ${Number(tr.mean).toFixed(2)} / 最大 ${Number(tr.max).toFixed(2)} mm · 内点 ${r.num_inliers}/${r.num_samples}</span></div>`);
    }
    if (res.stereo_baseline_mm != null) {
      lines.push(`<div class="stat"><em>右目相对左目</em><strong>${Number(res.stereo_baseline_mm).toFixed(2)} mm</strong></div>`);
    }
  }
  if (lines.length) box.innerHTML = lines.join("");
}

async function heApplySaved(name) {
  if (!name) return;
  try {
    const d = await post("/api/handeye/load", { name });
    renderHe(d);
    if (d.runtime) renderRuntimeConfig(d.runtime);
    flash(`已应用手眼「${name}」`, true);
  } catch (e) { flash(String(e), false); }
}

async function heDeleteSaved(name) {
  if (!name) return;
  if (!confirm(`删除手眼「${name}」？若正在应用将取消，并删掉配套 yaml。`)) return;
  try {
    const d = await post("/api/handeye/delete", { name });
    renderHe(d);
    if (d.runtime) renderRuntimeConfig(d.runtime);
    const box = $("he-result");
    if (box) box.innerHTML = "";
    flash("已删除", true);
  } catch (e) { flash(String(e), false); }
}

if ($("he-table-body")) {
  $("he-table-body").addEventListener("click", (ev) => {
    const btn = ev.target.closest("button[data-act]");
    if (!btn) return;
    const tr = btn.closest("tr");
    const name = tr?.dataset?.name || "";
    if (!name) return;
    if (btn.dataset.act === "apply") heApplySaved(name);
    else if (btn.dataset.act === "delete") heDeleteSaved(name);
  });
}

async function refreshHandeye() {
  const res = await fetch("/api/handeye", { cache: "no-store" });
  const data = await res.json();
  if (!data.ok) throw new Error(data.error || "手眼状态失败");
  renderHe(data);
  return data;
}

window.refreshHandeye = refreshHandeye;

function applyHeRun(st) {
  if (!st) return;
  applyCameraKind(st);
  renderCameraSelect(st);
  renderColorIntr(st);
  if (st.running) {
    const p = st.progress || [0, 0];
    if ($("he-count")) $("he-count").textContent = `${p[0]}/${p[1]}`;
    const msg = $("he-msg");
    if (msg) msg.textContent = st.phase || "";
  }
  if (window.heWasRunning && !st.running) {
    refreshHandeye().catch((e) => flash(String(e), false));
  }
  window.heWasRunning = !!st.running;
}

window.applyHeRun = applyHeRun;

let camPreviewBusy = false;

function camPreviewWanted() {
  const card = $("panel-cam");
  if (!card || !card.classList.contains("open")) return false;
  const book = card.closest(".book");
  if (book && !book.classList.contains("open")) return false;
  const page = card.closest(".page");
  if (page && !page.classList.contains("active")) return false;
  if (typeof document.hidden === "boolean" && document.hidden) return false;
  return true;
}

async function refreshCamPreview() {
  if (!camPreviewWanted() || camPreviewBusy) return;
  const img = $("cam-preview");
  if (!img) return;
  camPreviewBusy = true;
  try {
    const res = await fetch("/api/handeye/preview.jpg?t=" + Date.now(), { cache: "no-store" });
    if (!camPreviewWanted()) return; // 拉取中途若已收起，丢弃
    const msg = $("cam-msg");
    if (!res.ok) {
      let text = window.cameraKind === "rgbd" ? "相机没有画面" : "双目没有画面";
      try { const data = await res.json(); if (data.error) text = data.error; } catch (e) { /* 非 JSON */ }
      if (msg) msg.textContent = text;
      return;
    }
    const blob = await res.blob();
    const url = URL.createObjectURL(blob);
    if (img.dataset.obj) URL.revokeObjectURL(img.dataset.obj);
    img.dataset.obj = url;
    img.src = url;
    if (msg) msg.textContent = "";
  } catch (e) {
    const msg = $("cam-msg");
    if (msg) msg.textContent = String(e);
  } finally {
    camPreviewBusy = false;
  }
}


document.addEventListener("visibilitychange", () => {
  if (!document.hidden && camPreviewWanted()) refreshCamPreview();
});

$("btn-he-run").onclick = async () => {
  if (!posesetName()) { flash("先在姿态集里选一套已经记好的点", false); return; }
  const arm = window.currentSet && window.currentSet.arm;
  const side = arm === "left" ? "左臂" : arm === "right" ? "右臂" : "这一套";
  if (!confirm(`将按当前限速走完「${posesetName()}」（${side}），每个点停稳后存图。扶住、守急停。继续？`)) return;
  try {
    const d = await post("/api/handeye/run", { name: posesetName(), ...heForm() });
    applyHeRun(d);
    flash("开始按姿态集采集", true);
  } catch (e) { flash(String(e), false); }
};

$("btn-he-stop").onclick = async () => {
  try { await post("/api/handeye/stop", {}); flash("已请求停止", true); }
  catch (e) { flash(String(e), false); }
};

$("btn-he-capture").onclick = async () => {
  try {
    const d = await post("/api/handeye/capture", heForm());
    renderHe(d);
    flash(`已存 ${d.session} / ${String(d.index).padStart(4, "0")}`, true);
  } catch (e) { flash(String(e), false); }
};

$("he-session").onchange = async () => {
  const name = $("he-session").value;
  if (!name) return;
  try { renderHe(await post("/api/handeye/session", { name })); }
  catch (e) { flash(String(e), false); }
};

$("btn-he-solve").onclick = async () => {
  const f = heForm();
  if (!f.session) { flash("先选一次采集会话", false); return; }
  if (!f.square_size_mm) { flash("填棋盘格子边长（毫米）", false); return; }
  try {
    flash("先估内参，再做手眼…", true);
    const d = await post("/api/handeye/solve", f);
    renderHe(d);
    flash("求解完成，结果写在该会话目录", true);
  } catch (e) { flash(String(e), false); }
};

$("btn-he-save").onclick = async () => {
  const name = $("he-calib-name").value.trim();
  if (!name) { flash("填一个名字", false); return; }
  const exists = (window.lastHeSaved || []).some((s) => s.name === name);
  if (exists && !confirm(`覆盖已保存的「${name}」？`)) return;
  try {
    const d = await post("/api/handeye/save", { name, session: $("he-session").value });
    $("he-calib-name").value = "";
    renderHe(d);
    if (d.runtime) renderRuntimeConfig(d.runtime);
    flash(`已保存并应用「${name}」`, true);
  } catch (e) { flash(String(e), false); }
};
