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
      // 由后端机型能力明确告知，不按几何猜（G1D 底盘也有部分低于原点，但它不该动）
      this._floating = await fetch("/api/robot", { cache: "no-store" })
        .then((r) => r.json()).then((c) => !!c.legged).catch(() => false);
      if (this._floating) this._ground(box);
      const size = box.getSize(new THREE.Vector3());
      this.height = Math.max(size.z, 0.4);
      const grid = new THREE.GridHelper(Math.max(this.height * 2.4, 2), 16, 0x243044, 0x1a2330);
      grid.rotation.x = Math.PI / 2;
      scene.add(grid);
      this.ready = true;
      this.resize();
      this.home();
      this.render();
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
  // 雷达点云：单独一层，挂在躯干上（点已在躯干系），每次刷新替换，不参与相机点云的叠帧 / 清除
  setLidarCloud(xyz, rgb) {
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
    this.render();
    return true;
  },
  clearLidarCloud() {
    if (this.lidarPoints) {
      this._disposePoints(this.lidarPoints);
      this.lidarPoints = null;
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
    const show = $("arm-select") ? $("arm-select").value : "both";
    let placed = false;
    for (const arm of ["right", "left"]) {
      if (show === "left" && arm === "right") continue;
      if (show === "right" && arm === "left") continue;
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
