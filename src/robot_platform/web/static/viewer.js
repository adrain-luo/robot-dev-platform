import * as THREE from "./vendor/three.module.js";
import { OrbitControls } from "./vendor/OrbitControls.js";
import { STLLoader } from "./vendor/STLLoader.js";

const BASE = "/urdf/";
const vector = (s, fallback = "0 0 0") => (s || fallback).trim().split(/\s+/).map(Number);

function origin(node, element) {
  const pose = element.querySelector(":scope > origin");
  node.position.fromArray(vector(pose?.getAttribute("xyz")));
  const [roll, pitch, yaw] = vector(pose?.getAttribute("rpy"));
  node.rotation.set(roll, pitch, yaw, "ZYX");
}

function skipMesh(filename) {
  const name = (filename || "").split("/").pop() || "";
  return /dex1|hand_/i.test(name);
}

function skipLink(name) {
  return /dex1|hand_/i.test(name || "");
}

function denoisePoints(xyz, rgb01, { voxel = 0.018, minNeighbors = 6 } = {}) {
  const n = Math.floor(xyz.length / 3);
  if (n < 20) return { xyz, rgb: rgb01, kept: n, dropped: 0 };
  const inv = 1 / voxel;
  const bins = new Map();
  for (let i = 0; i < n; i++) {
    const ix = Math.floor(xyz[i * 3] * inv);
    const iy = Math.floor(xyz[i * 3 + 1] * inv);
    const iz = Math.floor(xyz[i * 3 + 2] * inv);
    const k = `${ix},${iy},${iz}`;
    let arr = bins.get(k);
    if (!arr) { arr = []; bins.set(k, arr); }
    arr.push(i);
  }
  const keep = [];
  for (let i = 0; i < n; i++) {
    const ix = Math.floor(xyz[i * 3] * inv);
    const iy = Math.floor(xyz[i * 3 + 1] * inv);
    const iz = Math.floor(xyz[i * 3 + 2] * inv);
    let c = 0;
    for (let dx = -1; dx <= 1; dx++) {
      for (let dy = -1; dy <= 1; dy++) {
        for (let dz = -1; dz <= 1; dz++) {
          const arr = bins.get(`${ix + dx},${iy + dy},${iz + dz}`);
          if (arr) c += arr.length;
        }
      }
    }
    if (c >= minNeighbors) keep.push(i);
  }
  if (keep.length === n) return { xyz, rgb: rgb01, kept: n, dropped: 0 };
  const outX = new Float32Array(keep.length * 3);
  const outC = new Float32Array(keep.length * 3);
  for (let j = 0; j < keep.length; j++) {
    const i = keep[j];
    outX[j * 3] = xyz[i * 3];
    outX[j * 3 + 1] = xyz[i * 3 + 1];
    outX[j * 3 + 2] = xyz[i * 3 + 2];
    outC[j * 3] = rgb01[i * 3];
    outC[j * 3 + 1] = rgb01[i * 3 + 1];
    outC[j * 3 + 2] = rgb01[i * 3 + 2];
  }
  return { xyz: outX, rgb: outC, kept: keep.length, dropped: n - keep.length };
}

const View = {
  ready: false,
  links: new Map(),
  motions: new Map(),
  cloudFrame: "vision", // vision=跟躯干；scene=钉在世界
  imuLevel: true, // 腿式机型按 IMU 横滚 / 俯仰摆正骨盆（渲染卡片可关）
  cloudAccumulate: false, // false=单帧；true=固定场景累加
  cloudMaxDistM: 2.0, // 距机器人中心（基座原点）最远显示距离；0=不裁
  cloudLayers: [],
  async attach(canvas) {
    if (this.canvas) return;
    this.canvas = canvas;
    const renderer = this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    renderer.setClearColor(0x0c1016, 1);
    const scene = this.scene3 = new THREE.Scene();
    scene.add(new THREE.HemisphereLight(0xffffff, 0x3a4450, 1.4));
    const key = new THREE.DirectionalLight(0xffffff, 2.2);
    key.position.set(2.5, -3, 4);
    scene.add(key);
    const fill = new THREE.DirectionalLight(0xc5d4e8, 0.8);
    fill.position.set(-2, 2, 2);
    scene.add(fill);
    const camera = this.camera = new THREE.PerspectiveCamera(35, 1, 0.01, 100);
    camera.up.set(0, 0, 1);
    const controls = this.controls = new OrbitControls(camera, canvas);
    controls.enableDamping = false;
    controls.enablePan = true;
    controls.mouseButtons = { LEFT: THREE.MOUSE.ROTATE, MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.PAN };
    controls.addEventListener("change", () => this.render());
    canvas.addEventListener("contextmenu", (e) => e.preventDefault());
    canvas.addEventListener("dblclick", () => this.home());
    this.overlays = new THREE.Group();
    scene.add(this.overlays);
    this.cmdLine = this._line(0x6b7280);
    this.tgtLine = this._line(0x60a5fa);
    this.pay = new THREE.Mesh(
      new THREE.SphereGeometry(0.02, 16, 12),
      new THREE.MeshStandardMaterial({ color: 0xe879f9 })
    );
    this.pay.visible = false;
    this.overlays.add(this.pay);
    this._resizeTimer = 0;
    const kick = () => this.resize();
    new ResizeObserver(kick).observe(canvas.parentElement || canvas);
    requestAnimationFrame(kick);
    try {
      const robot = await this._load();
      // 记下机器人自身（URDF）的网格：贴地只看它们，不让后来挂上去的点云 / 标记把模型顶高
      robot.traverse((o) => { if (o.isMesh) o.userData.robotBody = true; });
      scene.add(robot);
      this.robot = robot;
      robot.updateMatrixWorld(true);
      const box = new THREE.Box3().setFromObject(robot);
      // 腿式机型（如 G1）根在骨盆、一截在原点下面：抬到脚落地，之后随腿部实测再贴地。
      // 由后端机型能力明确告知，不按几何猜（轮式 G1D 不随腿重贴地，地面看雷达）
      this._floating = await fetch("/api/robot", { cache: "no-store" })
        .then((r) => r.json()).then((c) => !!c.legged).catch(() => false);
      // 腿式按脚贴地；轮式（G1D）先按轮子最低点贴地，有雷达点云后改用雷达拟合的地面（setLidarCloud）
      this._ground(box);
      const size = box.getSize(new THREE.Vector3());
      this.height = Math.max(size.z, 0.4);
      this._gridDefault();
      this.ready = true;
      this.resize();
      this.home();
      this.render();
      this.loadSensors();
    } catch (err) {
      const note = document.getElementById("msg");
      if (note) note.textContent = "三维模型没有载入：" + err.message;
    }
  },
  _line(color) {
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.Float32BufferAttribute([0, 0, 0, 0, 0, 0], 3));
    const mat = new THREE.LineBasicMaterial({ color });
    const line = new THREE.LineSegments(geo, mat);
    this.overlays.add(line);
    return line;
  },
  async _load() {
    const text = await fetch(BASE + "robot.urdf").then((r) => {
      if (!r.ok) throw new Error("读不到 URDF");
      return r.text();
    });
    const xml = new DOMParser().parseFromString(text, "application/xml");
    if (xml.querySelector("parsererror")) throw new Error("URDF 无效");
    const robot = new THREE.Group();
    const links = this.links;
    const loader = new STLLoader();
    const geometries = new Map();
    const pending = [];
    for (const link of xml.querySelectorAll("robot > link")) {
      const group = new THREE.Group();
      group.name = link.getAttribute("name");
      links.set(group.name, group);
      if (skipLink(group.name)) continue;
      for (const visual of link.querySelectorAll(":scope > visual")) {
        pending.push(this._visual(visual, xml, group, loader, geometries));
      }
    }
    for (const joint of xml.querySelectorAll("robot > joint")) {
      const parent = links.get(joint.querySelector("parent")?.getAttribute("link"));
      const child = links.get(joint.querySelector("child")?.getAttribute("link"));
      if (!parent || !child || child.parent) continue;
      const pivot = new THREE.Group();
      origin(pivot, joint);
      const motion = new THREE.Group();
      const axis = new THREE.Vector3(...vector(joint.querySelector("axis")?.getAttribute("xyz"), "1 0 0")).normalize();
      const type = joint.getAttribute("type") || "fixed";
      this.motions.set(joint.getAttribute("name"), { node: motion, type, axis });
      parent.add(pivot);
      pivot.add(motion);
      motion.add(child);
    }
    for (const link of links.values()) if (!link.parent) robot.add(link);
    await Promise.all(pending);
    return robot;
  },
  async _visual(visual, xml, group, loader, geometries) {
    const geometry = visual.querySelector("geometry");
    const mesh = geometry?.querySelector("mesh");
    let shape;
    if (mesh) {
      const filename = mesh.getAttribute("filename") || "";
      if (skipMesh(filename)) return;
      const url = BASE + filename.replace(/^\.\//, "");
      if (!geometries.has(url)) {
        geometries.set(url, loader.loadAsync(url).catch(() => null));
      }
      shape = await geometries.get(url);
      if (!shape) return;
    } else if (geometry?.querySelector("box")) {
      shape = new THREE.BoxGeometry(...vector(geometry.querySelector("box").getAttribute("size")));
    } else if (geometry?.querySelector("cylinder")) {
      const c = geometry.querySelector("cylinder");
      shape = new THREE.CylinderGeometry(+c.getAttribute("radius"), +c.getAttribute("radius"), +c.getAttribute("length"), 20);
      shape.rotateX(Math.PI / 2);
    } else return;
    const material = visual.querySelector("material");
    const named = [...xml.querySelectorAll("robot > material")].find((m) => m.getAttribute("name") === material?.getAttribute("name"));
    const rgba = vector((material?.querySelector("color") || named?.querySelector("color"))?.getAttribute("rgba"), "0.72 0.76 0.82 1");
    const body = new THREE.Mesh(shape, new THREE.MeshStandardMaterial({
      color: new THREE.Color(rgba[0], rgba[1], rgba[2]),
      roughness: 0.62,
      metalness: 0.12,
    }));
    origin(body, visual);
    group.add(body);
  },
  resize() {
    const cv = this.canvas;
    if (!cv || !this.renderer) return;
    const parent = cv.parentElement;
    const w = parent ? parent.clientWidth : cv.clientWidth;
    const h = parent ? parent.clientHeight : cv.clientHeight;
    if (!w || !h) {
      if (this._resizeTimer) cancelAnimationFrame(this._resizeTimer);
      this._resizeTimer = requestAnimationFrame(() => this.resize());
      return;
    }
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.render();
  },
  home() {
    if (!this.camera) return;
    // 对准模型高度中部；腿式机型（G1）重心比带立柱的 G1D 高，按 0.35 会切掉头
    const h = this.height || 1;
    const distance = Math.max(h, 1) * (this._floating ? 2.0 : 1.7);
    this.camera.position.set(distance * 0.85, -distance * 0.65, h * (this._floating ? 0.8 : 0.72));
    this.controls.target.set(0, 0, h * (this._floating ? 0.5 : 0.35));
    this.controls.update();
    this.render();
  },
  _cloudParent() {
    if (this.cloudFrame === "scene") return this.scene3;
    return this.links.get("torso_link") || null;
  },
  _disposePoints(obj) {
    if (!obj) return;
    if (obj.parent) obj.parent.remove(obj);
    obj.geometry?.dispose();
    obj.material?.dispose();
  },
  _robotCenterWorld() {
    // 机器人中心：基座在场景里的原点（robot 根）
    const o = new THREE.Vector3(0, 0, 0);
    if (this.robot) {
      this.robot.updateMatrixWorld(true);
      o.setFromMatrixPosition(this.robot.matrixWorld);
    }
    return o;
  },
  _xyzToWorld(xyz, torso) {
    torso.updateMatrixWorld(true);
    const m = torso.matrixWorld;
    const out = new Float32Array(xyz.length);
    const v = new THREE.Vector3();
    for (let i = 0; i < xyz.length; i += 3) {
      v.set(xyz[i], xyz[i + 1], xyz[i + 2]).applyMatrix4(m);
      out[i] = v.x; out[i + 1] = v.y; out[i + 2] = v.z;
    }
    return out;
  },
  _xyzWorldToTorso(xyz, torso) {
    torso.updateMatrixWorld(true);
    const inv = new THREE.Matrix4().copy(torso.matrixWorld).invert();
    const out = new Float32Array(xyz.length);
    const v = new THREE.Vector3();
    for (let i = 0; i < xyz.length; i += 3) {
      v.set(xyz[i], xyz[i + 1], xyz[i + 2]).applyMatrix4(inv);
      out[i] = v.x; out[i + 1] = v.y; out[i + 2] = v.z;
    }
    return out;
  },
  /** 按距机器人中心裁切；xyz/rgb 为当前挂接坐标系下的完整点。 */
  _filterByMaxDist(xyz, rgb, frame) {
    const maxM = Number(this.cloudMaxDistM);
    if (!(maxM > 0) || !Number.isFinite(maxM)) {
      return { xyz: Float32Array.from(xyz), rgb: Float32Array.from(rgb), kept: xyz.length / 3, dropped: 0 };
    }
    const torso = this.links.get("torso_link");
    const center = this._robotCenterWorld();
    const max2 = maxM * maxM;
    const n = Math.floor(xyz.length / 3);
    const outX = new Float32Array(xyz.length);
    const outC = new Float32Array(rgb.length);
    let w = 0;
    const v = new THREE.Vector3();
    const needWorld = frame !== "scene" && torso;
    if (needWorld) torso.updateMatrixWorld(true);
    const m = needWorld ? torso.matrixWorld : null;
    for (let i = 0; i < n; i++) {
      const i3 = i * 3;
      v.set(xyz[i3], xyz[i3 + 1], xyz[i3 + 2]);
      if (m) v.applyMatrix4(m);
      const dx = v.x - center.x;
      const dy = v.y - center.y;
      const dz = v.z - center.z;
      if (dx * dx + dy * dy + dz * dz > max2) continue;
      outX[w] = xyz[i3];
      outX[w + 1] = xyz[i3 + 1];
      outX[w + 2] = xyz[i3 + 2];
      outC[w] = rgb[i3];
      outC[w + 1] = rgb[i3 + 1];
      outC[w + 2] = rgb[i3 + 2];
      w += 3;
    }
    return {
      xyz: outX.subarray(0, w),
      rgb: outC.subarray(0, w),
      kept: w / 3,
      dropped: n - w / 3,
    };
  },
  _applyCloudMaxDist(pts) {
    if (!pts?.userData?.fullXyz) return;
    const frame = pts.userData.frame || this.cloudFrame;
    const filtered = this._filterByMaxDist(pts.userData.fullXyz, pts.userData.fullRgb, frame);
    pts.geometry.setAttribute("position", new THREE.Float32BufferAttribute(filtered.xyz, 3));
    pts.geometry.setAttribute("color", new THREE.Float32BufferAttribute(filtered.rgb, 3));
    pts.geometry.computeBoundingSphere();
  },
  setCloudMaxDist(m) {
    const next = Math.max(0, Number(m) || 0);
    this.cloudMaxDistM = next;
    for (const layer of this.cloudLayers) this._applyCloudMaxDist(layer);
    if (this.cloudCompare) this._applyCloudMaxDist(this.cloudCompare);
    this.render();
    return this.cloudStatus();
  },
  clearClouds({ keepCompare = false } = {}) {
    for (const layer of this.cloudLayers) this._disposePoints(layer);
    this.cloudLayers = [];
    if (this.cloud) {
      this._disposePoints(this.cloud);
      this.cloud = null;
    }
    if (!keepCompare && this.cloudCompare) {
      this._disposePoints(this.cloudCompare);
      this.cloudCompare = null;
    }
    this.render();
    return true;
  },
  setCloudFrame(frame) {
    const next = frame === "scene" ? "scene" : "vision";
    if (next === this.cloudFrame) return this.cloudStatus();
    const torso = this.links.get("torso_link");
    if (!this.ready || !torso || !this.scene3) {
      this.cloudFrame = next;
      return this.cloudStatus();
    }
    const layers = [...this.cloudLayers];
    if (this.cloudCompare) layers.push(this.cloudCompare);
    for (const pts of layers) {
      const wasScene = pts.userData.frame === "scene";
      const full = pts.userData.fullXyz;
      let nextFull = null;
      if (full && next === "scene" && !wasScene) nextFull = this._xyzToWorld(full, torso);
      else if (full && next === "vision" && wasScene) nextFull = this._xyzWorldToTorso(full, torso);
      else if (!full) {
        const pos = pts.geometry.getAttribute("position");
        const arr = pos.array;
        let nextArr;
        if (next === "scene" && !wasScene) nextArr = this._xyzToWorld(arr, torso);
        else if (next === "vision" && wasScene) nextArr = this._xyzWorldToTorso(arr, torso);
        else continue;
        pos.array.set(nextArr);
        pos.needsUpdate = true;
        pts.geometry.computeBoundingSphere();
      }
      if (nextFull) {
        pts.userData.fullXyz = nextFull;
      }
      pts.userData.frame = next;
      if (pts.parent) pts.parent.remove(pts);
      (next === "scene" ? this.scene3 : torso).add(pts);
      this._applyCloudMaxDist(pts);
    }
    this.cloudFrame = next;
    this.render();
    return this.cloudStatus();
  },
  setCloudAccumulate(on) {
    this.cloudAccumulate = !!on;
    return this.cloudStatus();
  },
  cloudStatus() {
    let n = 0;
    let full = 0;
    for (const layer of this.cloudLayers) {
      const pos = layer.geometry?.getAttribute("position");
      if (pos) n += pos.count;
      if (layer.userData?.fullXyz) full += layer.userData.fullXyz.length / 3;
    }
    if (this.cloudCompare) {
      const pos = this.cloudCompare.geometry?.getAttribute("position");
      if (pos) n += pos.count;
      if (this.cloudCompare.userData?.fullXyz) full += this.cloudCompare.userData.fullXyz.length / 3;
    }
    return {
      frame: this.cloudFrame,
      accumulate: this.cloudAccumulate,
      layers: this.cloudLayers.length,
      points: n,
      fullPoints: full || n,
      maxDistM: this.cloudMaxDistM,
      hasCompare: !!this.cloudCompare,
    };
  },
  setCloud(xyz, rgb, slot) {
    const torso = this.links.get("torso_link");
    if (!this.ready || !torso || !this.scene3 || !xyz || xyz.length < 3) return false;
    const compare = slot === "compare";
    if (compare) {
      this._disposePoints(this.cloudCompare);
      this.cloudCompare = null;
    } else if (!this.cloudAccumulate) {
      this.clearClouds({ keepCompare: true });
    }

    let positions = Float32Array.from(xyz);
    const colors = new Float32Array(rgb.length);
    for (let i = 0; i < rgb.length; i++) colors[i] = rgb[i] / 255;
    const frame = this.cloudFrame;
    if (frame === "scene") {
      positions = this._xyzToWorld(positions, torso);
    }
    const fullXyz = Float32Array.from(positions);
    const fullRgb = Float32Array.from(colors);
    const filtered = this._filterByMaxDist(fullXyz, fullRgb, frame);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.Float32BufferAttribute(filtered.xyz, 3));
    geo.setAttribute("color", new THREE.Float32BufferAttribute(filtered.rgb, 3));
    const mat = new THREE.PointsMaterial({
      size: compare ? 2.6 : 2.2,
      vertexColors: true,
      sizeAttenuation: false,
    });
    const pts = new THREE.Points(geo, mat);
    pts.userData.frame = frame;
    pts.userData.slot = compare ? "compare" : "main";
    pts.userData.fullXyz = fullXyz;
    pts.userData.fullRgb = fullRgb;
    (frame === "scene" ? this.scene3 : torso).add(pts);
    if (compare) this.cloudCompare = pts;
    else {
      this.cloudLayers.push(pts);
      this.cloud = pts; // 兼容旧引用：最近一层
    }
    this.render();
    return true;
  },
  // ---------------------------------------------------------------- 地面网格
  // 每格 10 cm（每 1 m 一条亮线），铺在 z = 0（地面）；范围跟雷达点云走，没有点云时围着机器人
  GRID_STEP: 0.1,
  _gridDefault() {
    const r = Math.ceil(Math.max((this.height || 1) * 1.2, 1) / 0.5) * 0.5;
    this._setGrid(-r, r, -r, r);
  },
  _setGrid(x0, x1, y0, y1) {
    const key = [x0, x1, y0, y1].map((v) => v.toFixed(2)).join(",");
    if (this._gridKey === key) return;
    this._gridKey = key;
    if (this.grid) {
      this.scene3.remove(this.grid);
      this.grid.geometry.dispose();
      this.grid.material.dispose();
    }
    const step = this.GRID_STEP;
    const pos = [];
    const col = [];
    const minor = new THREE.Color(0x1a2330);
    const major = new THREE.Color(0x33415a);
    const push = (ax, ay, bx, by, c) => {
      pos.push(ax, ay, 0, bx, by, 0);
      col.push(c.r, c.g, c.b, c.r, c.g, c.b);
    };
    const nx = Math.round((x1 - x0) / step);
    const ny = Math.round((y1 - y0) / step);
    for (let i = 0; i <= nx; i++) {
      const x = x0 + i * step;
      push(x, y0, x, y1, Math.abs(x - Math.round(x)) < 1e-6 ? major : minor);
    }
    for (let j = 0; j <= ny; j++) {
      const y = y0 + j * step;
      push(x0, y, x1, y, Math.abs(y - Math.round(y)) < 1e-6 ? major : minor);
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.Float32BufferAttribute(pos, 3));
    geo.setAttribute("color", new THREE.Float32BufferAttribute(col, 3));
    this.grid = new THREE.LineSegments(geo, new THREE.LineBasicMaterial({ vertexColors: true }));
    this.grid.userData.slot = "grid";
    this.scene3.add(this.grid);
  },
  // 网格范围 = 雷达点云的水平范围（世界系，向外取整到 0.5 m，含机器人周围 1 m），抖动时不重建
  _gridFromCloud(pointsObj) {
    const arr = pointsObj.geometry.getAttribute("position");
    if (!arr || !arr.count) return;
    pointsObj.updateMatrixWorld(true);
    const m = pointsObj.matrixWorld.elements;
    const a = arr.array;
    let x0 = -1, x1 = 1, y0 = -1, y1 = 1;
    const stride = Math.max(1, Math.floor(arr.count / 4000));
    for (let i = 0; i < arr.count; i += stride) {
      const px = a[3 * i], py = a[3 * i + 1], pz = a[3 * i + 2];
      const wx = m[0] * px + m[4] * py + m[8] * pz + m[12];
      const wy = m[1] * px + m[5] * py + m[9] * pz + m[13];
      if (wx < x0) x0 = wx; if (wx > x1) x1 = wx;
      if (wy < y0) y0 = wy; if (wy > y1) y1 = wy;
    }
    const out = (v, f) => f(v / 0.5) * 0.5;
    this._setGrid(out(x0, Math.floor), out(x1, Math.ceil), out(y0, Math.floor), out(y1, Math.ceil));
  },
  // 雷达拟合的地面（躯干系平面 n·p + d = 0）→ 把整机上下挪，让这个地面落在 z = 0（网格）。
  // 只用于轮式机型（G1D）；腿式（G1）照旧按脚贴地。小于 1 mm 的变化不动，大的分两三帧收过去，免得闪
  _groundFromLidar(ground) {
    const torso = this.links.get("torso_link");
    if (!ground || !torso || this._floating || !Array.isArray(ground.n)) return false;
    torso.updateMatrixWorld(true);
    const n = new THREE.Vector3(+ground.n[0], +ground.n[1], +ground.n[2]);
    const p0 = torso.localToWorld(n.clone().multiplyScalar(-(+ground.d)));
    const nw = n.clone().transformDirection(torso.matrixWorld);
    if (!(Math.abs(nw.z) > 0.8)) return false;
    // 机器人正下方（底盘原点的水平位置）的地面高度
    const o = this.robot.position;
    const zg = p0.z - ((o.x - p0.x) * nw.x + (o.y - p0.y) * nw.y) / nw.z;
    if (!Number.isFinite(zg) || Math.abs(zg) < 0.001) return true;
    const first = !this.groundSource;
    this.robot.position.z -= first ? zg : 0.5 * zg;
    this.robot.updateMatrixWorld(true);
    this.groundSource = "lidar";
    this.groundInfo = { inliers: ground.inliers, rms_mm: ground.rms_mm };
    return true;
  },
  // 雷达点云：单独一层，挂在躯干上（点已在躯干系），每次刷新替换，不参与相机点云的叠帧 / 清除
  setLidarCloud(xyz, rgb, ground) {
    const torso = this.links.get("torso_link");
    if (!this.ready || !torso || !xyz || xyz.length < 3) return false;
    const colors = new Float32Array(rgb.length);
    for (let i = 0; i < rgb.length; i++) colors[i] = rgb[i] / 255;
    if (!this.lidarPoints) {
      const geo = new THREE.BufferGeometry();
      const mat = new THREE.PointsMaterial({ size: 2.0, vertexColors: true, sizeAttenuation: false });
      this.lidarPoints = new THREE.Points(geo, mat);
      this.lidarPoints.userData.slot = "lidar";
      torso.add(this.lidarPoints);
    }
    const geo = this.lidarPoints.geometry;
    geo.setAttribute("position", new THREE.Float32BufferAttribute(Float32Array.from(xyz), 3));
    geo.setAttribute("color", new THREE.Float32BufferAttribute(colors, 3));
    geo.computeBoundingSphere();
    this._groundFromLidar(ground);
    this._gridFromCloud(this.lidarPoints);
    this.render();
    return true;
  },
  clearLidarCloud() {
    if (this.lidarPoints) {
      this._disposePoints(this.lidarPoints);
      this.lidarPoints = null;
      // 地面高度留着（还是最好的估计），网格回到机器人周围
      this._gridDefault();
      this.render();
    }
    return true;
  },
  optimizeClouds() {
    if (!this.ready) return { ok: false, error: "三维还没准备好" };
    const targets = [...this.cloudLayers];
    if (this.cloudCompare) targets.push(this.cloudCompare);
    if (!targets.length) return { ok: false, error: "没有点云可优化" };
    let kept = 0;
    let dropped = 0;
    for (const pts of targets) {
      const srcX = pts.userData.fullXyz || pts.geometry.getAttribute("position")?.array;
      const srcC = pts.userData.fullRgb || pts.geometry.getAttribute("color")?.array;
      if (!srcX || !srcC) continue;
      const out = denoisePoints(srcX, srcC);
      dropped += out.dropped;
      kept += out.kept;
      if (out.dropped <= 0) continue;
      pts.userData.fullXyz = out.xyz;
      pts.userData.fullRgb = out.rgb;
      this._applyCloudMaxDist(pts);
    }
    this.render();
    return { ok: true, kept, dropped, ...this.cloudStatus() };
  },
  setFixedPoint(xyz) {
    const torso = this.links.get("torso_link");
    if (!this.ready || !torso) return false;
    if (this.fixedPoint && xyz && xyz.length >= 3) {
      this.fixedPoint.position.set(xyz[0], xyz[1], xyz[2]);
      this.render();
      return true;
    }
    if (this.fixedPoint) {
      torso.remove(this.fixedPoint);
      this.fixedPoint.geometry.dispose();
      this.fixedPoint.material.dispose();
      this.fixedPoint = null;
    }
    if (!xyz || xyz.length < 3) {
      this.render();
      return true;
    }
    const geo = new THREE.SphereGeometry(0.012, 18, 14);
    const mat = new THREE.MeshBasicMaterial({ color: 0xff2a2a, depthTest: true });
    this.fixedPoint = new THREE.Mesh(geo, mat);
    this.fixedPoint.position.set(xyz[0], xyz[1], xyz[2]);
    torso.add(this.fixedPoint);
    this.render();
    return true;
  },
  /** TCP 修正中的预览位置；橙色。传 null 清除。 */
  setTcpPreview(xyz) {
    const torso = this.links.get("torso_link");
    if (!this.ready || !torso) return false;
    if (!this.tcpPreview && xyz && xyz.length >= 3) {
      this.tcpPreview = new THREE.Mesh(
        new THREE.SphereGeometry(0.011, 18, 14),
        new THREE.MeshBasicMaterial({ color: 0xff8c1a, depthTest: true }),
      );
      torso.add(this.tcpPreview);
    }
    if (this.tcpPreview) {
      const show = !!(xyz && xyz.length >= 3);
      this.tcpPreview.visible = show;
      if (show) this.tcpPreview.position.set(xyz[0], xyz[1], xyz[2]);
    }
    this.render();
    return true;
  },
  setSpatialPoint(xyz) {
    const torso = this.links.get("torso_link");
    if (!this.ready || !torso) return false;
    if (this.spatialPoint) {
      torso.remove(this.spatialPoint);
      this.spatialPoint.geometry.dispose();
      this.spatialPoint.material.dispose();
      this.spatialPoint = null;
    }
    if (!xyz || xyz.length < 3) {
      this.render();
      return true;
    }
    const geo = new THREE.SphereGeometry(0.014, 18, 14);
    const mat = new THREE.MeshBasicMaterial({ color: 0xffc107, depthTest: true });
    this.spatialPoint = new THREE.Mesh(geo, mat);
    this.spatialPoint.position.set(xyz[0], xyz[1], xyz[2]);
    torso.add(this.spatialPoint);
    this.render();
    return true;
  },
  /** 碰点瞄准（空间点+偏置）；青色。无偏置时传 null 清除。 */
  setSpatialAim(xyz) {
    const torso = this.links.get("torso_link");
    if (!this.ready || !torso) return false;
    if (this.spatialAim) {
      torso.remove(this.spatialAim);
      this.spatialAim.geometry.dispose();
      this.spatialAim.material.dispose();
      this.spatialAim = null;
    }
    if (!xyz || xyz.length < 3) {
      this.render();
      return true;
    }
    const geo = new THREE.SphereGeometry(0.011, 16, 12);
    const mat = new THREE.MeshBasicMaterial({ color: 0x22d3ee, depthTest: true });
    this.spatialAim = new THREE.Mesh(geo, mat);
    this.spatialAim.position.set(xyz[0], xyz[1], xyz[2]);
    torso.add(this.spatialAim);
    this.render();
    return true;
  },
  setBoardPoints(xyz) {
    const torso = this.links.get("torso_link");
    if (!this.ready || !torso) return false;
    if (this.boardPoints) {
      torso.remove(this.boardPoints);
      this.boardPoints.traverse((obj) => {
        if (obj.geometry) obj.geometry.dispose();
        if (obj.material) obj.material.dispose();
      });
      this.boardPoints = null;
    }
    if (!xyz || xyz.length < 3) {
      this.render();
      return true;
    }
    const group = new THREE.Group();
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.Float32BufferAttribute(xyz, 3));
    group.add(new THREE.Points(geo, new THREE.PointsMaterial({
      color: 0x38bdf8, size: 7, sizeAttenuation: false,
    })));
    const lineGeo = new THREE.BufferGeometry();
    lineGeo.setAttribute("position", new THREE.Float32BufferAttribute(xyz, 3));
    group.add(new THREE.Line(lineGeo, new THREE.LineBasicMaterial({ color: 0x38bdf8 })));
    const head = new THREE.Mesh(
      new THREE.SphereGeometry(0.01, 14, 10),
      new THREE.MeshBasicMaterial({ color: 0x22c55e })
    );
    head.position.set(xyz[0], xyz[1], xyz[2]);
    group.add(head);
    this.boardPoints = group;
    torso.add(group);
    this.render();
    return true;
  },
  /** 工作流临时点（浅蓝色）；不落盘。传 null 清除。 */
  setWorkflowPoint(xyz) {
    const torso = this.links.get("torso_link");
    if (!this.ready || !torso) return false;
    if (this.workflowPoint) {
      torso.remove(this.workflowPoint);
      this.workflowPoint.geometry.dispose();
      this.workflowPoint.material.dispose();
      this.workflowPoint = null;
    }
    if (!xyz || xyz.length < 3) {
      this.render();
      return true;
    }
    const geo = new THREE.SphereGeometry(0.015, 18, 14);
    const mat = new THREE.MeshBasicMaterial({ color: 0x7dd3fc, depthTest: true });
    this.workflowPoint = new THREE.Mesh(geo, mat);
    this.workflowPoint.position.set(xyz[0], xyz[1], xyz[2]);
    torso.add(this.workflowPoint);
    this.render();
    return true;
  },
  /** 识别定位：左红 / 右蓝 / 中心绿。传入 flat xyz 数组或 null 清除。 */
  setDetectPoints(leftXyz, rightXyz, centerXyz) {
    const torso = this.links.get("torso_link");
    if (!this.ready || !torso) return false;
    if (this.detectPoints) {
      torso.remove(this.detectPoints);
      this.detectPoints.traverse((obj) => {
        if (obj.geometry) obj.geometry.dispose();
        if (obj.material) obj.material.dispose();
      });
      this.detectPoints = null;
    }
    const hasL = leftXyz && leftXyz.length >= 3;
    const hasR = rightXyz && rightXyz.length >= 3;
    const hasC = centerXyz && centerXyz.length >= 3;
    if (!hasL && !hasR && !hasC) {
      this.render();
      return true;
    }
    const group = new THREE.Group();
    const addCloud = (xyz, color, size) => {
      const geo = new THREE.BufferGeometry();
      geo.setAttribute("position", new THREE.Float32BufferAttribute(xyz, 3));
      group.add(new THREE.Points(geo, new THREE.PointsMaterial({
        color, size, sizeAttenuation: false,
      })));
    };
    if (hasL) addCloud(leftXyz, 0xff2a2a, 6);
    if (hasR) addCloud(rightXyz, 0x3b82f6, 6);
    if (hasC) {
      const geo = new THREE.SphereGeometry(0.014, 18, 14);
      const mat = new THREE.MeshBasicMaterial({ color: 0x22c55e, depthTest: true });
      const mesh = new THREE.Mesh(geo, mat);
      mesh.position.set(centerXyz[0], centerXyz[1], centerXyz[2]);
      group.add(mesh);
    }
    this.detectPoints = group;
    torso.add(group);
    this.render();
    return true;
  },
  render() {
    if (this.renderer && this.scene3 && this.camera) this.renderer.render(this.scene3, this.camera);
  },
  // ---------------------------------------------------------------- 碰撞体（/api/collision/geometry）
  // 各连杆系下的胶囊体 / 球（[ax,ay,az,bx,by,bz,r]），挂在对应连杆上随关节动；半透明，highlight 的连杆标红。
  _collisionGroups: null,
  _collisionGeom: null,
  _collisionInflate: 0,
  _collisionHot: null,
  // inflate（m）：每个碰撞体半径加厚这么多。卡片传「余量的一半」：两块碰到一起 = 间距刚好到余量
  setCollisionInflate(m) {
    this._collisionInflate = Math.max(0, Number(m) || 0);
    if (this._collisionGeom) this.setCollision(this._collisionGeom);
  },
  setCollision(geom) {
    for (const g of this._collisionGroups || []) {
      if (g.parent) g.parent.remove(g);
      g.traverse((o) => { if (o.geometry) o.geometry.dispose(); if (o.material) o.material.dispose(); });
    }
    this._collisionGroups = [];
    this._collisionGeom = geom || null;
    if (!this.ready || !geom || !geom.links) { this.render(); return false; }
    const inf = this._collisionInflate || 0;
    const sphereGeo = new THREE.SphereGeometry(1, 14, 10);
    const up = new THREE.Vector3(0, 1, 0);
    const m4 = new THREE.Matrix4();
    for (const [link, prims] of Object.entries(geom.links)) {
      const parent = this.links.get(link);
      if (!parent) continue;
      const mat = new THREE.MeshStandardMaterial({
        color: 0x38bdf8, transparent: true, opacity: 0.28, depthWrite: false, roughness: 0.6, metalness: 0,
      });
      const group = new THREE.Group();
      group.userData.collisionLink = link;
      group.userData.mat = mat;
      const spheres = prims.filter((p) => p[0] === p[3] && p[1] === p[4] && p[2] === p[5]);
      if (spheres.length) {
        const inst = new THREE.InstancedMesh(sphereGeo.clone(), mat, spheres.length);
        spheres.forEach((p, i) => {
          const r = p[6] + inf;
          m4.makeScale(r, r, r).setPosition(p[0], p[1], p[2]);
          inst.setMatrixAt(i, m4);
        });
        inst.instanceMatrix.needsUpdate = true;
        group.add(inst);
      }
      for (const p of prims) {
        if (spheres.includes(p)) continue;
        const a = new THREE.Vector3(p[0], p[1], p[2]);
        const b = new THREE.Vector3(p[3], p[4], p[5]);
        const len = a.distanceTo(b);
        const mesh = new THREE.Mesh(new THREE.CapsuleGeometry(p[6] + inf, len, 6, 14), mat);
        mesh.position.copy(a).add(b).multiplyScalar(0.5);
        mesh.quaternion.setFromUnitVectors(up, b.clone().sub(a).normalize());
        group.add(mesh);
      }
      parent.add(group);
      this._collisionGroups.push(group);
    }
    sphereGeo.dispose();
    if (this._collisionHot) this.setCollisionHighlight(this._collisionHot);
    this.render();
    return true;
  },
  setCollisionHighlight(links) {
    this._collisionHot = links || [];
    const hot = new Set(this._collisionHot);
    for (const g of this._collisionGroups || []) {
      const on = hot.has(g.userData.collisionLink);
      g.userData.mat.color.setHex(on ? 0xef4444 : 0x38bdf8);
      g.userData.mat.opacity = on ? 0.5 : 0.28;
    }
    this.render();
  },
  // ---------------------------------------------------------------- 传感器位置标记（/api/sensors）
  // 相机：小机身 + 视锥（光学系 z 朝前）；雷达：圆柱。挂在对应 URDF 连杆上，随躯干一起动。
  // 传感器多在头壳里面，标记不做深度遮挡，透过外壳也看得见。
  sensorsVisible: true,
  async loadSensors() {
    if (!this.ready) return;
    try {
      const d = await fetch("/api/sensors", { cache: "no-store" }).then((r) => r.json());
      this.setSensors((d && d.sensors) || []);
    } catch (err) {
      this.setSensors([]);
    }
  },
  setSensorsVisible(on) {
    this.sensorsVisible = !!on;
    for (const g of this._sensorGroups || []) g.visible = this.sensorsVisible;
    this.render();
  },
  _sensorMat(color, opacity = 1) {
    return { color, transparent: true, opacity, depthTest: false, depthWrite: false };
  },
  _sensorLabel(text, color) {
    const pad = 10, font = 28;
    const c = document.createElement("canvas");
    const ctx = c.getContext("2d");
    ctx.font = `600 ${font}px system-ui, sans-serif`;
    c.width = Math.ceil(ctx.measureText(text).width) + pad * 2;
    c.height = font + pad * 2;
    ctx.font = `600 ${font}px system-ui, sans-serif`;
    ctx.fillStyle = "rgba(15, 20, 28, 0.82)";
    ctx.fillRect(0, 0, c.width, c.height);
    ctx.strokeStyle = color;
    ctx.lineWidth = 3;
    ctx.strokeRect(1.5, 1.5, c.width - 3, c.height - 3);
    ctx.fillStyle = "#f3f4f6";
    ctx.textBaseline = "middle";
    ctx.fillText(text, pad, c.height / 2);
    const tex = new THREE.CanvasTexture(c);
    const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, depthWrite: false }));
    const h = 0.022;
    sp.scale.set(h * c.width / c.height, h, 1);
    sp.renderOrder = 1001;
    return sp;
  },
  _cameraMarker(color) {
    const g = new THREE.Group();
    const body = new THREE.Mesh(new THREE.BoxGeometry(0.022, 0.016, 0.012), new THREE.MeshBasicMaterial(this._sensorMat(color, 0.9)));
    body.position.z = -0.006;
    g.add(body);
    // 视锥：光学系 z 朝前、x 右、y 下，深 6 cm
    const zf = 0.06, xf = 0.04, yf = 0.03;
    const c = [[xf, yf], [-xf, yf], [-xf, -yf], [xf, -yf]].map(([x, y]) => new THREE.Vector3(x, y, zf));
    const o = new THREE.Vector3(0, 0, 0);
    const pts = [];
    for (let i = 0; i < 4; i++) pts.push(o, c[i], c[i], c[(i + 1) % 4]);
    g.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial(this._sensorMat(color))));
    // 图像「上」方向（光学 -y）画一小截，看得出相机有没有装反
    g.add(new THREE.LineSegments(
      new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, -yf, zf), new THREE.Vector3(0, -yf - 0.015, zf)]),
      new THREE.LineBasicMaterial(this._sensorMat(color))));
    return g;
  },
  _lidarMarker(color) {
    const g = new THREE.Group();
    // MID-360：半球形，直径约 65 mm。圆底面在连杆原点，球顶朝连杆 +z（雷达自身轴向；
    // G1 / G1D 头里是倒装的，mid360_link 的 z 在躯干系里朝下，所以球顶朝下）
    const r = 0.0325;
    // three 的球面 phi 0..π/2 是绕 y 轴的上半球（顶朝 +y），转到顶朝 +z
    const dome = new THREE.Mesh(new THREE.SphereGeometry(r, 24, 12, 0, Math.PI * 2, 0, Math.PI / 2),
      new THREE.MeshBasicMaterial({ ...this._sensorMat(color, 0.35), side: THREE.DoubleSide }));
    dome.rotation.x = Math.PI / 2;
    g.add(dome);
    const wire = new THREE.LineSegments(
      new THREE.WireframeGeometry(new THREE.SphereGeometry(r, 12, 4, 0, Math.PI * 2, 0, Math.PI / 2)),
      new THREE.LineBasicMaterial(this._sensorMat(color, 0.9)));
    wire.rotation.x = Math.PI / 2;
    g.add(wire);
    // 底面圆
    const base = [];
    for (let i = 0; i < 32; i++) {
      const a0 = (i / 32) * Math.PI * 2, a1 = ((i + 1) / 32) * Math.PI * 2;
      base.push(new THREE.Vector3(r * Math.cos(a0), r * Math.sin(a0), 0), new THREE.Vector3(r * Math.cos(a1), r * Math.sin(a1), 0));
    }
    g.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(base), new THREE.LineBasicMaterial(this._sensorMat(color))));
    // 轴向（球顶方向）多画一截
    g.add(new THREE.LineSegments(
      new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, r + 0.02)]),
      new THREE.LineBasicMaterial(this._sensorMat(color))));
    return g;
  },
  setSensors(list) {
    for (const g of this._sensorGroups || []) {
      if (g.parent) g.parent.remove(g);
      g.traverse((o) => {
        if (o.geometry) o.geometry.dispose();
        if (o.material) { if (o.material.map) o.material.map.dispose(); o.material.dispose(); }
      });
    }
    this._sensorGroups = [];
    if (!this.ready) return;
    const COLORS = { camera: 0x22d3ee, cameraNominal: 0xf59e0b, lidar: 0xa3e635 };
    for (const s of list || []) {
      const parent = this.links.get(s.parent_link);
      if (!parent || !Array.isArray(s.T) || s.T.length !== 4) continue;
      const color = s.kind === "lidar" ? COLORS.lidar : (s.calibrated ? COLORS.camera : COLORS.cameraNominal);
      const css = "#" + color.toString(16).padStart(6, "0");
      const g = new THREE.Group();
      g.name = `sensor:${s.id}`;
      g.userData.sensor = s;
      const m = new THREE.Matrix4();
      m.set(...s.T[0], ...s.T[1], ...s.T[2], ...s.T[3]);
      m.decompose(g.position, g.quaternion, g.scale);
      g.add(s.kind === "lidar" ? this._lidarMarker(color) : this._cameraMarker(color));
      g.traverse((o) => { o.renderOrder = 1000; });
      parent.add(g);
      g.updateMatrixWorld(true);
      // 标签按世界方向错开（雷达在后上方，相机在前方，左右眼上下错开），免得叠在一起；
      // 按挂上去时的世界朝向换到局部（躯干只会小角度倾斜）
      const off = s.kind === "lidar" ? [-0.07, 0, 0.075]
        : s.eye === "left" ? [0.1, 0, 0.05] : s.eye === "right" ? [0.1, 0, -0.005] : [0.1, 0, 0.03];
      const label = this._sensorLabel(s.calibrated ? `${s.label}（标定）` : `${s.label}（标称）`, css);
      const qw = g.getWorldQuaternion(new THREE.Quaternion()).invert();
      label.position.copy(new THREE.Vector3(...off).applyQuaternion(qw));
      g.add(label);
      g.visible = this.sensorsVisible;
      this._sensorGroups.push(g);
    }
    this.render();
  },
  // 腿式机型：按机身 IMU（在骨盆）的横滚 / 俯仰转骨盆，网格平面即真实水平面；不用偏航（会漂，也无意义）。
  // 返回姿态是否明显变了（变了就马上重新贴地）。
  setImuLevel(on) {
    this.imuLevel = !!on;
    if (!this.imuLevel && this.robot) {
      this.robot.rotation.set(0, 0, 0);
      this._imuRP = null;
      this.robot.updateMatrixWorld(true);
      this._ground();
      this.render();
    }
    return this.imuLevel;
  },
  _applyImuLevel(imu) {
    if (!this._floating || !this.robot || this.imuLevel === false) return false;
    const rpy = imu && imu.rpy;
    if (!Array.isArray(rpy) || rpy.length < 2 || !rpy.slice(0, 2).every(Number.isFinite)) return false;
    const [r, p] = rpy;
    // IMU 的 rpy 是 ZYX 欧拉角：R = Rz(yaw)·Ry(pitch)·Rx(roll)，去掉偏航即 Ry(pitch)·Rx(roll)
    this.robot.rotation.set(r, p, 0, "ZYX");
    const prev = this._imuRP;
    this._imuRP = [r, p];
    return !prev || Math.abs(prev[0] - r) + Math.abs(prev[1] - p) > 0.005;
  },
  _bodyBox() {
    const box = new THREE.Box3();
    this.robot.traverse((o) => {
      if (o.isMesh && o.userData.robotBody) box.expandByObject(o);
    });
    return box.isEmpty() ? new THREE.Box3().setFromObject(this.robot) : box;
  },
  _ground(box) {
    // 只按机器人自身网格贴地：躯干上挂的雷达 / 相机点云有地面以下的点，算进去会把整机顶高
    const b = box || this._bodyBox();
    this.robot.position.z -= b.min.z;
    this.robot.updateMatrixWorld(true);
    this._groundAt = performance.now();
  },
  sync(data) {
    if (!this.ready || !data) return false;
    try {
      for (const [name, mot] of this.motions) {
        const raw = data.joints ? data.joints[name] : undefined;
        const q = Number.isFinite(raw) ? raw : 0;
        if (mot.type === "prismatic") mot.node.position.copy(mot.axis).multiplyScalar(q);
        else if (mot.type === "revolute" || mot.type === "continuous") {
          if (mot.axis.lengthSq() < 1e-12) continue;
          mot.node.quaternion.setFromAxisAngle(mot.axis, q);
        }
      }
      const tilted = this._applyImuLevel(data.imu);
      this.robot.updateMatrixWorld(true);
      if (this._floating && (tilted || performance.now() - (this._groundAt || 0) > 1000)) this._ground();
      this._lines(this.cmdLine, data, "cmd_links");
      this._lines(this.tgtLine, data, "target_links");
      this._payload(data);
      this.render();
      return true;
    } catch (err) {
      const note = document.getElementById("msg");
      if (note) note.textContent = "三维画面刷新失败：" + err.message;
      return false;
    }
  },
  _lines(line, data, key) {
    const pts = [];
    for (const arm of ["right", "left"]) {
      const chain = data[arm]?.[key];
      if (!chain) continue;
      const world = this._torso(chain);
      for (let i = 1; i < world.length; i++) pts.push(world[i - 1], world[i]);
    }
    line.geometry.dispose();
    line.geometry = new THREE.BufferGeometry().setFromPoints(pts);
    line.visible = pts.length > 0;
  },
  _torso(pts) {
    const link = this.links.get("torso_link");
    if (!link) return [];
    return pts.filter(Boolean).map((p) => link.localToWorld(new THREE.Vector3(+p[0], +p[1], +p[2])));
  },
  _payload(data) {
    // 只有一个负载标记：优先画控制臂的，没有再画另一只
    const ctrlArm = (window.lastCtrl && window.lastCtrl.arm) || "right";
    let placed = false;
    for (const arm of ctrlArm === "left" ? ["left", "right"] : ["right", "left"]) {
      if (placed) break;
      const sc = data[arm];
      if (!sc || !(sc.payload_mass > 1e-6) || !sc.payload_com_m) continue;
      const wrist = this.links.get(arm === "left" ? "left_wrist_yaw_link" : "right_wrist_yaw_link");
      if (!wrist) continue;
      const local = new THREE.Vector3(...sc.payload_com_m);
      this.pay.position.copy(wrist.localToWorld(local));
      this.pay.visible = true;
      placed = true;
    }
    if (!placed) this.pay.visible = false;
  },
};

window.RobotView = View;
window.dispatchEvent(new Event("robot-view-ready"));
