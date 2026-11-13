// 16-suction.js —— 控制页「末端 → 吸盘」：吸 / 释放（后端转发到盒子上的 es80z-api）

const SUCTION_STATE = { sucking: "吸着（真空开）", released: "已释放", blowing: "吹气中", unknown: "状态未知" };
const suctionUi = { busy: false };

function renderSuction(st) {
  const now = $("suction-now");
  const badge = $("suction-badge");
  if (!st) return;
  if (!st.available) {
    if (now) { now.textContent = st.error || "吸盘服务不可用"; now.className = "note bad"; }
    if (badge) badge.textContent = "离线";
  } else {
    const parts = [SUCTION_STATE[st.state] || st.state];
    if (st.fault) parts.push("故障");
    if (st.serial_present === false) parts.push("串口不在");
    if (st.serial_error) parts.push(st.serial_error);
    if (st.model) parts.push(st.model);
    if (now) { now.textContent = parts.join(" · "); now.className = `note${st.fault ? " bad" : st.state === "sucking" ? " on" : ""}`; }
    if (badge) badge.textContent = st.state === "sucking" ? "吸着" : (st.fault ? "故障" : "");
  }
  const allow = !!(window.lastCtrl || {}).allow_real_motion;
  ["btn-suction-suck", "btn-suction-release"].forEach((id) => {
    const b = $(id);
    if (b) b.disabled = suctionUi.busy || !allow || !st.available;
  });
}

async function suctionRefresh() {
  try {
    const d = await post("/api/suction/status", {});
    renderSuction(d.suction);
  } catch (e) { /* 轮询失败不打扰 */ }
}

async function suctionCmd(cmd) {
  suctionUi.busy = true;
  ["btn-suction-suck", "btn-suction-release"].forEach((id) => { if ($(id)) $(id).disabled = true; });
  if ($("suction-now")) $("suction-now").textContent = cmd === "suck" ? "吸…" : "释放…";
  try {
    const d = await post(`/api/suction/${cmd}`, {});
    flash(d.ok ? d.message : (d.error || "吸盘失败"), !!d.ok);
    suctionUi.busy = false;
    renderSuction(d.suction);
  } catch (e) {
    suctionUi.busy = false;
    flash(String(e.message || e), false);
    suctionRefresh();
  }
}

if ($("btn-suction-suck")) $("btn-suction-suck").addEventListener("click", () => suctionCmd("suck"));
if ($("btn-suction-release")) $("btn-suction-release").addEventListener("click", () => suctionCmd("release"));
// 卡片看得见时每 3 s 刷一次状态
setInterval(() => {
  const panel = $("panel-tool");
  if (document.hidden || suctionUi.busy || !panel || panel.offsetParent === null) return;
  suctionRefresh();
}, 3000);
suctionRefresh();
