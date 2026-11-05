// 13-lidar.js —— 雷达卡片：选雷达、实时显示 / 取一帧点云（躯干系，画在中央三维里）、清除
// 由 web/static/js/ 下各文件按文件名顺序拼成 /static/app.js（同一个脚本作用域）。

let lidarLive = false;
let lidarBusy = false;
let lidarTimer = null;
const LIDAR_LIVE_MS = 400;

function renderLidarStatus(st) {
  const sel = $("lidar-select");
  if (!sel || !st) return;
  const sig = JSON.stringify((st.lidars || []).map((l) => [l.id, l.label]));
  if (sel.dataset.sig !== sig) {
    sel.dataset.sig = sig;
    sel.innerHTML = "";
    for (const l of st.lidars || []) {
      const o = document.createElement("option");
      o.value = l.id;
      o.textContent = l.label;
      sel.appendChild(o);
    }
    if (!(st.lidars || []).length) {
      const o = document.createElement("option");
      o.value = "";
      o.textContent = "（机型没有声明雷达）";
      sel.appendChild(o);
    }
  }
  if (document.activeElement !== sel) sel.value = st.lidar || "";
  sel.disabled = (st.lidars || []).length <= 1;
  const none = !(st.lidars || []).length;
  ["btn-lidar-live", "btn-lidar-once"].forEach((id) => { if ($(id)) $(id).disabled = none; });
  if (none && $("lidar-note")) $("lidar-note").textContent = "这台机器的 robot.json 没有声明雷达（lidars）。";
  if ($("lidar-desc") && st.topic) {
    $("lidar-desc").title = `话题 ${st.topic} · 坐标系 ${st.link}${st.mock ? " · 仿真" : ""}`;
  }
}

async function refreshLidarStatus() {
  try {
    renderLidarStatus(await fetch("/api/lidar", { cache: "no-store" }).then((r) => r.json()));
  } catch (e) { /* 服务断开时由顶部状态提示 */ }
}

function lidarQuery() {
  const p = new URLSearchParams({
    max_range: $("lidar-range").value || "8",
    max_points: $("lidar-points").value || "8000",
    color: $("lidar-color").value || "height",
  });
  return "/api/lidar/cloud?" + p.toString();
}

async function lidarFetchOnce() {
  if (lidarBusy) return;
  lidarBusy = true;
  const note = $("lidar-note");
  try {
    const d = await fetch(lidarQuery(), { cache: "no-store" }).then((r) => r.json());
    if (!d.ok) {
      if (note) note.textContent = d.error || "没有雷达数据";
      return;
    }
    const drawn = window.RobotView && window.RobotView.setLidarCloud
      ? window.RobotView.setLidarCloud(d.xyz, d.rgb) : false;
    const rng = d.range_m ? `距离 ${d.range_m.near.toFixed(2)}–${d.range_m.far.toFixed(1)} m（中位 ${d.range_m.median.toFixed(2)}）` : "";
    if (note) {
      note.textContent = `${d.hz > 0 ? d.hz + " Hz" : "测速中"} · 每帧 ${d.n_raw} 点，范围内 ${d.n_valid}，显示 ${d.n_sent}。${rng}`
        + (drawn ? "" : " 三维画面还没准备好。") + (lidarLive ? " 实时中。" : "");
    }
  } catch (e) {
    if (note) note.textContent = String(e);
  } finally {
    lidarBusy = false;
  }
}

function lidarWanted() {
  const card = $("panel-lidar");
  if (!card || !card.classList.contains("open")) return false;
  const book = card.closest(".book");
  if (book && !book.classList.contains("open")) return false;
  return !document.hidden;
}

function setLidarLive(on) {
  lidarLive = !!on;
  if ($("btn-lidar-live")) $("btn-lidar-live").textContent = lidarLive ? "停止实时" : "实时显示";
  if (lidarTimer) { clearInterval(lidarTimer); lidarTimer = null; }
  if (lidarLive) {
    lidarFetchOnce();
    // 卡片收起 / 书合上 / 页面切走时暂停取数，展开回来自动继续
    lidarTimer = setInterval(() => { if (lidarWanted()) lidarFetchOnce(); }, LIDAR_LIVE_MS);
  }
}

if ($("btn-lidar-live")) $("btn-lidar-live").onclick = () => setLidarLive(!lidarLive);
if ($("btn-lidar-once")) $("btn-lidar-once").onclick = () => { setLidarLive(false); lidarFetchOnce(); };
if ($("btn-lidar-clear")) {
  $("btn-lidar-clear").onclick = () => {
    setLidarLive(false);
    if (window.RobotView && window.RobotView.clearLidarCloud) window.RobotView.clearLidarCloud();
    if ($("lidar-note")) $("lidar-note").textContent = "已清除";
  };
}
if ($("lidar-select")) {
  $("lidar-select").onchange = async () => {
    try {
      renderLidarStatus(await post("/api/lidar/config", { lidar: $("lidar-select").value }));
      if (lidarLive) lidarFetchOnce();
    } catch (e) { flash(String(e.message || e), false); }
  };
}
if ($("lidar-range")) {
  // 拖动时只更新数字；松手后（非实时模式下已有点云时）按新距离重取一帧，实时模式下一拍自动生效。
  $("lidar-range").addEventListener("input", () => {
    $("lidar-range-val").textContent = (parseFloat($("lidar-range").value) || 8).toFixed(1);
  });
}
["lidar-range", "lidar-points", "lidar-color"].forEach((id) => {
  if ($(id)) $(id).addEventListener("change", () => { if (!lidarLive && window.RobotView && window.RobotView.lidarPoints) lidarFetchOnce(); });
});
