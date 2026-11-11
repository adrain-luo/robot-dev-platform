// 99-main.js —— 启动：首次拉取机型与数据、各轮询定时器。放最后，此时上面的函数与变量都已就绪
// 由 web/static/js/ 下各文件按文件名顺序拼成 /static/app.js（同一个脚本作用域）。

fetch("/api/robot", { cache: "no-store" }).then((r) => r.json()).then(applyRobotCaps).catch(() => {});

initArmPicks();

ensureGotoInputs();

V3.init();

setInterval(poll, 100);

poll();

refreshValList().catch(() => {});

refreshPoseList().catch((e) => {
  const m = document.getElementById("msg");
  if (!m) return;
  m.className = "err";
  m.textContent = "姿态集列表没有加载: " + e;
});

setInterval(refreshCamPreview, 700);

refreshLidarStatus();

window.removeEventListener("error", showLoadError);
