// 10-render.js —— 渲染卡片：点云挂接 / 叠帧 / 最远距离、清除与优化、匹配点云
// 由 web/static/js/ 下各文件按文件名顺序拼成 /static/app.js（同一个脚本作用域）。

function syncRenderNote(extra) {
  const el = $("render-note");
  if (!el || !window.RobotView || !window.RobotView.cloudStatus) return;
  const st = window.RobotView.cloudStatus();
  const frame = st.frame === "scene" ? "场景" : "视觉";
  const stack = st.accumulate ? "固定" : "单帧";
  const dist = Number(st.maxDistM);
  const distTxt = Number.isFinite(dist) && dist > 0 ? `≤${dist.toFixed(1)} m` : "不限距";
  let text = `${frame} · ${stack} · ${distTxt} · ${st.layers} 层 · ${st.points}`;
  if (st.fullPoints && st.fullPoints !== st.points) text += `/${st.fullPoints}`;
  text += " 点";
  if (st.hasCompare) text += " · 含调试层";
  if (extra) text += ` · ${extra}`;
  el.textContent = text;
}

function applyRenderControlsFromUi() {
  if (!window.RobotView) return;
  const frame = $("render-frame")?.value || "vision";
  const stack = $("render-stack")?.value || "single";
  if (window.RobotView.setCloudFrame) window.RobotView.setCloudFrame(frame);
  if (window.RobotView.setCloudAccumulate) window.RobotView.setCloudAccumulate(stack === "fixed");
  if (window.RobotView.setImuLevel) window.RobotView.setImuLevel(!!$("render-imu-level")?.checked);
  const distEl = $("cloud-max-dist");
  if (distEl && window.RobotView.setCloudMaxDist) {
    window.RobotView.setCloudMaxDist(parseFloat(distEl.value) || 2);
  }
  syncRenderNote();
}

function syncCloudMaxDistLabel() {
  const sl = $("cloud-max-dist");
  const lab = $("cloud-max-dist-val");
  if (!sl || !lab) return;
  lab.textContent = Number(sl.value).toFixed(1);
}

if ($("render-imu-level")) {
  $("render-imu-level").addEventListener("change", () => {
    if (window.RobotView && window.RobotView.setImuLevel) window.RobotView.setImuLevel($("render-imu-level").checked);
  });
}

if ($("cloud-max-dist")) {
  syncCloudMaxDistLabel();
  $("cloud-max-dist").addEventListener("input", () => {
    syncCloudMaxDistLabel();
    const m = parseFloat($("cloud-max-dist").value) || 2;
    if (window.RobotView && window.RobotView.setCloudMaxDist) {
      window.RobotView.setCloudMaxDist(m);
      syncRenderNote();
    }
  });
}

if ($("render-frame")) {
  $("render-frame").onchange = () => {
    applyRenderControlsFromUi();
    flash($("render-frame").value === "scene" ? "点云钉在场景" : "点云跟随躯干", true);
  };
}

if ($("render-stack")) {
  $("render-stack").onchange = () => {
    applyRenderControlsFromUi();
    flash($("render-stack").value === "fixed" ? "固定场景：匹配会叠加" : "单帧场景：匹配会替换", true);
  };
}

if ($("btn-cloud-clear")) {
  $("btn-cloud-clear").onclick = () => {
    if (!window.RobotView || !window.RobotView.clearClouds) {
      flash("三维还没准备好", false);
      return;
    }
    window.RobotView.clearClouds();
    syncRenderNote("已清除");
    flash("点云已清除（棋盘/红黄点保留）", true);
  };
}

if ($("btn-cloud-optimize")) {
  $("btn-cloud-optimize").onclick = () => {
    if (!window.RobotView || !window.RobotView.optimizeClouds) {
      flash("三维还没准备好", false);
      return;
    }
    const r = window.RobotView.optimizeClouds();
    if (!r.ok) { flash(r.error || "优化失败", false); return; }
    syncRenderNote(`滤除 ${r.dropped}，剩 ${r.kept}`);
    flash(`点云优化：去掉 ${r.dropped} 个噪点，剩 ${r.kept}`, true);
  };
}

window.addEventListener("robot-view-ready", () => {
  applyRenderControlsFromUi();
  syncRenderNote();
});

async function matchCloud(body, slot) {
  const msg = $("cam-match");
  if (msg) msg.textContent = slot === "compare" ? "正在用调试口的内外参匹配…" : "正在匹配…";
  try {
    applyRenderControlsFromUi();
    const d = await post("/api/handeye/match", body);
    const img = $("cam-disp");
    if (img && d.disparity_jpeg) img.src = "data:image/jpeg;base64," + d.disparity_jpeg;
    const drawn = d.cloud && window.RobotView && window.RobotView.setCloud(d.cloud.xyz, d.cloud.rgb, slot);
    const dep = d.depth_m || {};
    const st = window.RobotView && window.RobotView.cloudStatus ? window.RobotView.cloudStatus() : null;
    let text = `有效 ${d.valid} 点，画面上 ${d.saved} 点。深度中位 ${Number(dep.median).toFixed(2)} m。`
      + (drawn ? " 已画到中央模型上。" : " 三维画面还没准备好。");
    if (st) {
      text += st.accumulate ? ` 固定场景第 ${st.layers} 层。` : " 单帧已替换。";
      text += st.frame === "scene" ? " 挂接：场景。" : " 挂接：视觉。";
    }
    if (d.note) text += " " + d.note;
    if (msg) msg.textContent = text;
    syncRenderNote(drawn ? "刚匹配" : null);
  } catch (e) {
    if (msg) msg.textContent = String(e);
  }
}

$("btn-cam-match").onclick = () => matchCloud({}, "main");

$("btn-cam-match-debug").onclick = () => matchCloud({ source: "debug" }, "compare");

if ($("btn-render-match")) $("btn-render-match").onclick = () => matchCloud({}, "main");
