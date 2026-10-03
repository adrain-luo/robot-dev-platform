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

const View = {
  ready: false,
  links: new Map(),
  motions: new Map(),
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
    new ResizeObserver(() => this.resize()).observe(canvas);
    try {
      const robot = await this._load();
      scene.add(robot);
      this.robot = robot;
      robot.updateMatrixWorld(true);
      const box = new THREE.Box3().setFromObject(robot);
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
    const mat = new THREE.LineBasicMaterial({ color });
    const line = new THREE.LineSegments(geo, mat);
    this.overlays.add(line);
    return line;
  },
  async _load() {
    const text = await fetch(BASE + "g1_d.urdf").then((r) => {
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
    const w = cv.clientWidth, h = cv.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.render();
  },
  home() {
    if (!this.camera) return;
    const distance = Math.max(this.height || 1.2, 1) * 1.7;
    this.camera.position.set(distance * 0.85, -distance * 0.65, (this.height || 1) * 0.72);
    this.controls.target.set(0, 0, (this.height || 1) * 0.35);
    this.controls.update();
    this.render();
  },
  render() {
    if (this.renderer && this.scene3 && this.camera) this.renderer.render(this.scene3, this.camera);
  },
  sync(data) {
    if (!this.ready || !data) return false;
    for (const [name, mot] of this.motions) {
      const q = Number(data.joints?.[name] || 0);
      if (mot.type === "prismatic") mot.node.position.copy(mot.axis).multiplyScalar(q);
      else if (mot.type === "revolute" || mot.type === "continuous") mot.node.quaternion.setFromAxisAngle(mot.axis, q);
    }
    this.robot.updateMatrixWorld(true);
    this._lines(this.cmdLine, data, "cmd_links");
    this._lines(this.tgtLine, data, "target_links");
    this._payload(data);
    this.render();
    return true;
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
