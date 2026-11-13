// 15-single-pose.js —— 控制页「单姿态记录和到位」：一个名字 = 一只手的一个姿态
// 记录 / 点名字或「到位」走过去（后端按已应用的 IK 策略解末端位姿）/ 改名 / 删除。按「手」切换只看一只手。

const singlePose = { list: [], ikMode: "", busy: false };

function spArm() {
  return armPick("single");
}

function spRender() {
  const body = $("sp-table-body");
  if (!body) return;
  const arm = spArm();
  const rows = singlePose.list.filter((p) => p.arm === arm);
  body.innerHTML = "";
  $("sp-table-empty")?.classList.toggle("show", rows.length === 0);
  for (const p of rows) {
    const tr = document.createElement("tr");
    tr.dataset.name = p.name;
    const xyz = p.tcp_xyz || p.p;
    const meta = Array.isArray(xyz) ? xyz.map((v) => (v * 1000).toFixed(0)).join(", ") : "—";
    tr.innerHTML = `
      <td><button type="button" class="sp-name-btn" data-act="goto"></button></td>
      <td class="sp-meta"></td>
      <td><button type="button" class="sp-act" data-act="goto">到位</button></td>
      <td><button type="button" class="sp-act" data-act="rename">改名</button></td>
      <td><button type="button" class="sp-act sp-del" data-act="delete">删除</button></td>`;
    tr.querySelector(".sp-name-btn").textContent = p.name;
    tr.querySelector(".sp-name-btn").title = `到位「${p.name}」`;
    const td = tr.querySelector(".sp-meta");
    td.textContent = meta;
    td.title = p.tcp_name ? `记录时 TCP「${p.tcp_name}」的位置（躯干系）` : "法兰位置（躯干系，记录时没调用 TCP）";
    body.appendChild(tr);
  }
  spSyncButtons();
  const mode = $("sp-mode");
  const IK_NAMES = { single: "单次局部", robust: "多初值", luo: "罗-IK" };
  if (mode) mode.textContent = singlePose.ikMode ? `IK：${IK_NAMES[singlePose.ikMode] || singlePose.ikMode}` : "";
}

function spSyncButtons() {
  const ctrl = window.lastCtrl || {};
  const allow = !!ctrl.allow_real_motion;
  document.querySelectorAll("#sp-table [data-act='goto']").forEach((b) => {
    b.disabled = singlePose.busy || !allow;
  });
  if ($("btn-sp-record")) $("btn-sp-record").disabled = singlePose.busy;
}

async function spRefresh() {
  try {
    const d = await post("/api/single-pose/list", {});
    singlePose.list = d.list || [];
    singlePose.ikMode = d.ik_mode || "";
    spRender();
  } catch (e) {
    flash(`单姿态：${e.message || e}`, false);
  }
}

async function spRecord() {
  const name = ($("sp-name")?.value || "").trim();
  if (!name) { flash("先给姿态起个名字", false); return; }
  const arm = spArm();
  try {
    const d = await post("/api/single-pose/record", { name, arm });
    singlePose.list = d.list || singlePose.list;
    if ($("sp-name")) $("sp-name").value = "";
    spRender();
    flash(d.message || "已记录", true);
  } catch (e) {
    flash(String(e.message || e), false);
  }
}

async function spGoto(name) {
  const p = singlePose.list.find((x) => x.name === name);
  if (!p) return;
  const busy = await armBusyReasonFresh(p.arm);
  if (busy) { flash(busy, false); return; }
  if (!confirm(`${ARM_SIDE[p.arm] || ""}手到位「${name}」？按底栏限速走过去，扶住、守急停。`)) return;
  singlePose.busy = true;
  spSyncButtons();
  if ($("sp-now")) $("sp-now").textContent = `到位「${name}」中…`;
  try {
    const d = await post("/api/single-pose/goto", { name });
    if ($("sp-now")) $("sp-now").textContent = d.message || "已到位";
    flash(d.message || "已到位", true);
  } catch (e) {
    if ($("sp-now")) $("sp-now").textContent = String(e.message || e);
    flash(String(e.message || e), false);
  } finally {
    singlePose.busy = false;
    spSyncButtons();
  }
}

async function spRename(name) {
  const nn = (prompt(`把「${name}」改名为：`, name) || "").trim();
  if (!nn || nn === name) return;
  try {
    const d = await post("/api/single-pose/rename", { name, new_name: nn });
    singlePose.list = d.list || singlePose.list;
    spRender();
    flash(d.message || "已改名", true);
  } catch (e) {
    flash(String(e.message || e), false);
  }
}

async function spDelete(name) {
  if (!confirm(`删除姿态「${name}」？删除后不能恢复。`)) return;
  try {
    const d = await post("/api/single-pose/delete", { name });
    singlePose.list = d.list || singlePose.list;
    spRender();
    flash(d.message || "已删除", true);
  } catch (e) {
    flash(String(e.message || e), false);
  }
}

if ($("sp-table-body")) {
  $("sp-table-body").addEventListener("click", (ev) => {
    const btn = ev.target.closest("button[data-act]");
    if (!btn || btn.disabled) return;
    const name = btn.closest("tr")?.dataset.name;
    if (!name) return;
    const act = btn.dataset.act;
    if (act === "goto") spGoto(name);
    else if (act === "rename") spRename(name);
    else if (act === "delete") spDelete(name);
  });
}
if ($("btn-sp-record")) $("btn-sp-record").addEventListener("click", spRecord);
if ($("sp-name")) {
  $("sp-name").addEventListener("keydown", (ev) => { if (ev.key === "Enter") spRecord(); });
}
document.addEventListener("arm-pick", (ev) => { if (ev.detail?.key === "single") spRender(); });
// IK 策略在「配置」卡片里切换后，标题跟着变
if ($("btn-cfg-pose-ik-mode")) $("btn-cfg-pose-ik-mode").addEventListener("click", () => setTimeout(spRefresh, 800));
setInterval(spSyncButtons, 1000);
spRefresh();
