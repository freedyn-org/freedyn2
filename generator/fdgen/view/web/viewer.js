// FreeDyn 2 viewer: the scene of fdgen.view.scene (bodies with shapes, joints, forces, road) moved
// by the result CSV (R_x..R_z, theta0..theta3 per body), a timeline and plots of the channels.
// For checking a model: the checks of fdgen.checks (findings, references), a click on an element
// shows its data and YAML source and highlights what it is connected to. Every simulation is a
// run: other runs are compared in the plots (dashed), the reference run also as grey bodies and
// as the base of the difference plots (run strip: fdview/runs.js); while a simulation runs, its
// curves grow live. Force arrows from the force outputs, eigenmodes animated about a state;
// tuning, compute state, notes and the report: fdview/panels.js.
// Served by `freedyn2 view` (polls api/state and reloads on a new version) or as one exported
// HTML file (window.FD_STATIC).

import * as THREE from "three";
import { OrbitControls } from "three/addons/OrbitControls.js";
import { STLLoader } from "three/addons/STLLoader.js";
import { OBJLoader } from "three/addons/OBJLoader.js";
import { PlotArea, index, scaleColor } from "fdview/plot.js";
import { channelsOf, evaluate, parse, unit } from "fdview/channels.js";
import { Panels, tuneLabel } from "fdview/panels.js";
import { RunStrip } from "fdview/runs.js";

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);  // FreeDyn: z up

const $ = (id) => document.getElementById(id);
const STATIC = window.FD_STATIC || null;
const JOINT_COLOR = 0xd6336c, SPRING_COLOR = 0xe8590c, TIRE_COLOR = 0x12b886, SENSOR_COLOR = 0x7048e8, POINT_COLOR = 0x1c7ed6;
const FORCE_COLOR = 0xc92a2a, TORQUE_COLOR = 0x7048e8;

// ---------- renderer, camera, lights ----------
const view = $("view");
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio || 1);
view.prepend(renderer.domElement);
const world = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(40, 1, 0.01, 1e5);
camera.position.set(-4, -6, 3);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
world.add(new THREE.HemisphereLight(0xffffff, 0x8a8f98, 1.6));
const sun = new THREE.DirectionalLight(0xffffff, 1.6);
sun.position.set(-3, -5, 8);
world.add(sun);
world.add(sun.target);
const root = new THREE.Group();
world.add(root);

function resize() {
  const w = view.clientWidth, h = view.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / Math.max(h, 1);
  camera.updateProjectionMatrix();
  render();
}
new ResizeObserver(resize).observe(view);

function themeBackground() {
  world.background = new THREE.Color(getComputedStyle(document.documentElement).getPropertyValue("--scene").trim() || "#e9edf3");
}
themeBackground();
window.matchMedia("(prefers-color-scheme: dark)").addEventListener("change", themeBackground);

// ---------- data ----------
function parseCSV(text) {
  const lines = text.split(/\r?\n/).filter((l) => l.length);
  if (lines.length < 2) return null;
  const names = lines[0].split(",");
  const n = lines.length - 1, cols = names.map(() => new Float64Array(n));
  for (let i = 0; i < n; i++) {
    const f = lines[i + 1].split(",");
    for (let j = 0; j < names.length; j++) cols[j][i] = +f[j];
  }
  const map = new Map(names.map((name, j) => [name, cols[j]]));
  return { names, map, t: map.get("t"), n };
}

function base64Bytes(s) {
  const bin = atob(s), out = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
  return out;
}
const float32 = (s) => new Float32Array(base64Bytes(s).buffer);

async function meshBytes(url) {
  if (STATIC) return base64Bytes(STATIC.meshes[url]).buffer;
  const r = await fetch(url);
  if (!r.ok) throw new Error(`${url}: ${r.status}`);
  return r.arrayBuffer();
}

// ---------- scene objects ----------
const q = (t) => new THREE.Quaternion(t[1], t[2], t[3], t[0]).normalize();
let model = null, results = null, bodies = [], updaters = [], modelSize = 1, traces = null;
let reference = null;  // the reference run (parsed CSV), if it is not the current one
const state = { time: 0, playing: false, speed: 1, follow: "", selection: null, arrowScale: 1, mode: null };
const GHOST_COLOR = "#8a8f98";

function material(color, opacity = 1) {
  return new THREE.MeshStandardMaterial({ color, roughness: 0.65, metalness: 0.08, transparent: opacity < 1,
    opacity, depthWrite: opacity >= 1, side: THREE.DoubleSide });
}

function shapeObject(s, defaultShape) {
  const opacity = s.opacity ?? (defaultShape ? 0.8 : 1);
  const mat = material(s.color, opacity);
  const holder = new THREE.Group();
  holder.userData.shape = true;
  holder.position.fromArray(s.position);
  holder.quaternion.copy(q(s.orientation));
  let geo = null;
  if (s.type === "box") geo = new THREE.BoxGeometry(...s.size);
  else if (s.type === "cylinder") geo = new THREE.CylinderGeometry(s.radius, s.radius, s.length, 40).rotateX(Math.PI / 2);
  else if (s.type === "sphere") geo = new THREE.SphereGeometry(s.radius, 32, 16);
  if (geo) {
    holder.add(new THREE.Mesh(geo, mat));
    const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geo, 30),
      new THREE.LineBasicMaterial({ color: 0x000000, transparent: true, opacity: 0.25 }));
    holder.add(edges);
  } else if (s.type === "mesh") {
    meshBytes(s.url).then((buf) => {
      let obj;
      if (s.url.endsWith(".stl")) {
        const g = new STLLoader().parse(buf);
        g.computeVertexNormals();
        obj = new THREE.Mesh(g, mat);
      } else {
        obj = new OBJLoader().parse(new TextDecoder().decode(buf));
        obj.traverse((o) => { if (o.isMesh) o.material = mat; });
      }
      obj.scale.fromArray(s.scale);
      holder.add(obj);
      render();
    }).catch((e) => showError(`mesh ${s.file}: ${e.message}`));
  }
  return holder;
}

function jointGlyph(j) {
  const r = 0.012 * modelSize;
  const mat = new THREE.MeshBasicMaterial({ color: JOINT_COLOR, transparent: true, opacity: 0.85, depthTest: false });
  let geo;
  switch (j.type) {
    case "revolute": geo = new THREE.CylinderGeometry(r * 0.6, r * 0.6, r * 3, 20).rotateX(Math.PI / 2); break;
    case "cylindrical": geo = new THREE.CylinderGeometry(r * 0.4, r * 0.4, r * 6, 16).rotateX(Math.PI / 2); break;
    case "prismatic": geo = new THREE.BoxGeometry(r * 0.6, r * 0.6, r * 6); break;
    case "spherical": geo = new THREE.SphereGeometry(r, 20, 12); break;
    case "fixed": geo = new THREE.BoxGeometry(r * 1.2, r * 1.2, r * 1.2); break;
    default: geo = new THREE.OctahedronGeometry(r);
  }
  const m = new THREE.Mesh(geo, mat);
  m.renderOrder = 10;
  m.position.fromArray(j.position1);
  m.quaternion.copy(q(j.orientation1));
  m.userData.jointName = j.name;
  return m;
}

function marker(color, radius) {
  const m = new THREE.Mesh(new THREE.SphereGeometry(radius, 14, 10),
    new THREE.MeshBasicMaterial({ color, depthTest: false, transparent: true, opacity: 0.9 }));
  m.renderOrder = 11;
  return m;
}

function roadMesh(road) {
  const P = float32(road.points), mu = float32(road.friction), n = road.rows, m = road.cols;
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.BufferAttribute(P, 3));
  const colors = new Float32Array(n * m * 3), base = new THREE.Color(0x9097a3), wet = new THREE.Color(0x4dabf7), c = new THREE.Color();
  for (let k = 0; k < n * m; k++) {
    c.copy(base).lerp(wet, Math.min(1, Math.max(0, (1 - mu[k]) / 0.8)));
    if ((Math.floor(k / m) % 20 === 0) || (k % m) % 10 === 0) c.multiplyScalar(0.93);  // grid lines
    colors.set([c.r, c.g, c.b], 3 * k);
  }
  geo.setAttribute("color", new THREE.BufferAttribute(colors, 3));
  const idx = [];
  for (let i = 0; i + 1 < n; i++) for (let j = 0; j + 1 < m; j++) {
    const a = i * m + j, b = a + m;
    idx.push(a, b, a + 1, a + 1, b, b + 1);
  }
  geo.setIndex(idx);
  geo.computeVertexNormals();
  return new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.95, side: THREE.DoubleSide }));
}

function column(name) { return results ? results.map.get(name) : null; }

function bodyPose(b, k) {
  // position and rotation of body b at output k (the initial state without results); with an
  // eigenmode shown, displaced by it (translation global, rotation in the body frame)
  let pose;
  if (b.cols) {
    const [x, y, z, t0, t1, t2, t3] = b.cols;
    pose = [new THREE.Vector3(x[k], y[k], z[k]), new THREE.Quaternion(t1[k], t2[k], t3[k], t0[k]).normalize()];
  } else pose = [new THREE.Vector3(...b.position), q(b.orientation)];
  const d = state.mode && state.mode.offsets.get(b.name);
  if (d) { pose[0].add(d.R); pose[1].multiply(d.Q); }
  return pose;
}

function build(scene) {
  root.clear();
  updaters = [];
  state.mode = null;
  model = scene;
  // size of the model at t = 0: positions and shape extents of the bodies
  const box = new THREE.Box3();
  for (const b of scene.bodies) {
    box.expandByPoint(new THREE.Vector3(...b.position));
    for (const s of b.shapes) {
      const e = s.size ? Math.max(...s.size) / 2 : (s.radius || 0) + (s.length || 0) / 2;
      box.expandByPoint(new THREE.Vector3(...b.position).addScalar(e));
      box.expandByPoint(new THREE.Vector3(...b.position).addScalar(-e));
    }
  }
  modelSize = Math.max(box.getSize(new THREE.Vector3()).length(), 0.1);
  const byName = new Map();
  bodies = scene.bodies.map((b) => {
    const g = new THREE.Group();
    g.name = b.name;
    g.userData.body = b.name;
    for (const s of b.shapes) g.add(shapeObject(s, b.default_shapes));
    const axes = new THREE.AxesHelper(0.15 * modelSize);
    axes.material.depthTest = false; axes.renderOrder = 12; axes.userData.frame = true;
    g.add(axes);
    root.add(g);
    const names = ["R_x", "R_y", "R_z", "theta0", "theta1", "theta2", "theta3"].map((c) => `${b.name}.${c}`);
    const cols = names.every((c) => column(c)) ? names.map(column) : null;
    let ghost = null, prevCols = null;
    if (reference && names.every((c) => reference.map.has(c))) {  // the reference run: grey, transparent
      prevCols = names.map((c) => reference.map.get(c));
      ghost = new THREE.Group();
      for (const s of b.shapes) ghost.add(shapeObject({ ...s, color: GHOST_COLOR, opacity: 0.3 }, false));
      ghost.userData.ghost = true;
      root.add(ghost);
    }
    const entry = { ...b, group: g, cols, ghost, prevCols, color: b.shapes[0]?.color || "#888" };
    byName.set(b.name, entry);
    return entry;
  });
  const ground = new THREE.Group();
  for (const s of scene.ground_shapes) ground.add(shapeObject(s, false));
  root.add(ground);
  for (const j of scene.joints) {  // on body1, moving with it
    const parent = byName.get(j.body1);
    if (parent) { const m = jointGlyph(j); m.userData.joint = true; parent.group.add(m); }
  }
  for (const p of scene.points || []) {  // hardpoints and frames, on the body that carries them
    const m = new THREE.Mesh(new THREE.OctahedronGeometry(0.007 * modelSize),
      new THREE.MeshBasicMaterial({ color: POINT_COLOR, depthTest: false, transparent: true, opacity: 0.9 }));
    m.renderOrder = 11;
    m.position.fromArray(p.position);
    m.userData.point = true; m.userData.pointName = p.name;
    (byName.get(p.body)?.group || ground).add(m);
  }
  // springs, bushings and tires: drawn in the world from moving points
  const worldPoint = (body, p, k) => {
    const v = new THREE.Vector3(...p);
    if (body === "ground") return v;
    const [R, Q] = bodyPose(byName.get(body), k);
    return v.applyQuaternion(Q).add(R);
  };
  const forceGroup = new THREE.Group(); forceGroup.name = "forces";
  for (const f of scene.forces) {
    if (f.type === "spring_damper") {
      const geo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
      const line = new THREE.Line(geo, new THREE.LineBasicMaterial({ color: SPRING_COLOR, depthTest: false }));
      line.renderOrder = 9;
      const a = marker(SPRING_COLOR, 0.008 * modelSize), b = marker(SPRING_COLOR, 0.008 * modelSize);
      for (const o of [line, a, b]) o.userData.forceName = f.name;
      forceGroup.add(line, a, b);
      updaters.push((k) => {
        const p1 = worldPoint(f.body1, f.point1, k), p2 = worldPoint(f.body2, f.point2, k);
        geo.attributes.position.setXYZ(0, p1.x, p1.y, p1.z);
        geo.attributes.position.setXYZ(1, p2.x, p2.y, p2.z);
        geo.attributes.position.needsUpdate = true;
        geo.computeBoundingSphere();
        a.position.copy(p1); b.position.copy(p2);
      });
    } else if (f.type === "sensor") {
      const m = marker(SENSOR_COLOR, 0.012 * modelSize);
      m.userData.forceName = f.name;
      forceGroup.add(m);
      updaters.push((k) => m.position.copy(worldPoint(f.body1, f.point1, k)));
    } else if (f.type === "bushing") {
      const m = marker(SPRING_COLOR, 0.015 * modelSize);
      m.userData.forceName = f.name;
      forceGroup.add(m);
      updaters.push((k) => m.position.copy(worldPoint(f.body1, f.point1, k)));
    } else if (f.type === "tire") {
      const cx = column(`${f.name}.contact_x`), cy = column(`${f.name}.contact_y`), cz = column(`${f.name}.contact_z`);
      if (!cx) continue;
      const m = marker(TIRE_COLOR, 0.012 * modelSize);
      const geo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
      const line = new THREE.Line(geo, new THREE.LineBasicMaterial({ color: TIRE_COLOR, depthTest: false }));
      line.renderOrder = 9;
      m.userData.forceName = line.userData.forceName = f.name;
      forceGroup.add(m, line);
      updaters.push((k) => {
        const c = worldPoint(f.body1, [0, 0, 0], k);
        m.position.set(cx[k], cy[k], cz[k]);
        geo.attributes.position.setXYZ(0, c.x, c.y, c.z);
        geo.attributes.position.setXYZ(1, cx[k], cy[k], cz[k]);
        geo.attributes.position.needsUpdate = true;
        geo.computeBoundingSphere();
      });
    }
  }
  root.add(forceGroup);
  root.add(forceArrows(scene, worldPoint, byName));
  if (scene.road) root.add(roadMesh(scene.road));
  else {  // a floor grid below everything that moves
    const mb = motionBox(), c = mb.getCenter(new THREE.Vector3());
    const extent = Math.max(mb.getSize(new THREE.Vector3()).length(), 0.1) * 1.5;
    const spacing = Math.pow(10, Math.round(Math.log10(extent / 10))), lines = Math.ceil(extent / spacing / 2) * 2;
    const grid = new THREE.GridHelper(lines * spacing, lines, 0x7d8696, 0xa3abb9);
    grid.rotateX(Math.PI / 2);
    grid.position.set(c.x, c.y, Math.min(mb.min.z, 0));
    root.add(grid);
  }
  traces = new THREE.Group(); traces.name = "traces";
  if (results) for (const b of bodies) if (b.cols) {
    const pts = [], step = Math.max(1, Math.floor(results.n / 4000));
    for (let k = 0; k < results.n; k += step) pts.push(new THREE.Vector3(b.cols[0][k], b.cols[1][k], b.cols[2][k]));
    traces.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: b.color })));
  }
  root.add(traces);
  applyToggles();
  fillLists();
  renderChecks();
  panels.update(scene);
  if (state.selection && !findElement(state.selection.name)) state.selection = null;
  highlight();
  showInspector();
}

function applyToggles() {
  root.traverse((o) => {
    if (o.userData.joint) o.visible = $("show-joints").checked;
    if (o.userData.frame) o.visible = $("show-frames").checked;
    if (o.userData.point) o.visible = $("show-points").checked;
  });
  const f = root.getObjectByName("forces"); if (f) f.visible = $("show-forces").checked;
  const a = root.getObjectByName("arrows"); if (a) a.visible = $("show-arrows").checked;
  $("arrow-scale").disabled = !$("show-arrows").checked;
  if (traces) traces.visible = $("show-trace").checked;
  for (const b of bodies) if (b.ghost) b.ghost.visible = $("show-reference").checked;
  render();
}
for (const id of ["show-joints", "show-forces", "show-points", "show-frames", "show-trace", "show-arrows"]) $(id).addEventListener("change", applyToggles);
$("arrow-scale").addEventListener("input", (e) => { state.arrowScale = Math.pow(10, +e.target.value); refresh(); });

// ---------- force arrows ----------
// Arrows at the points of application, from the force outputs of the elements: force and torque
// vectors (global), spring forces along the spring (on both ends), bushings (outputs in their
// frame on body2), tires (outputs in the contact frame: longitudinal, lateral, normal; the
// normal is drawn upright, without the camber of the road). Length: the largest force of the
// run is 0.3 model sizes (torques likewise), times the scale.
function forceArrows(scene, worldPoint, byName) {
  const group = new THREE.Group();
  group.name = "arrows";
  if (!results) return group;
  const vec = (n, k) => { const x = column(`${n}_x`); return new THREE.Vector3(x[k], column(`${n}_y`)[k], column(`${n}_z`)[k]); };
  const rot = (body, k) => (body === "ground" ? new THREE.Quaternion() : bodyPose(byName.get(body), k)[1]);
  const list = [], add = (f, kind, at) => list.push({ name: f.name, kind, at });
  for (const f of scene.forces) {
    const n = f.name, has = (c) => !!column(`${n}.${c}`);
    if ((f.type === "force" || f.type === "expression_force") && has("force_x")) {
      add(f, "force", (k) => [worldPoint(f.body1, f.point1, k), vec(`${n}.force`, k)]);
    } else if ((f.type === "torque" || f.type === "expression_torque") && has("torque_x")) {
      add(f, "torque", (k) => [worldPoint(f.body1, f.point1, k), vec(`${n}.torque`, k)]);
    } else if (f.type === "spring_damper" && has("force")) {
      const s = column(`${n}.force`);
      const e = (k) => worldPoint(f.body2, f.point2, k).sub(worldPoint(f.body1, f.point1, k)).normalize();
      add(f, "force", (k) => [worldPoint(f.body1, f.point1, k), e(k).multiplyScalar(s[k])]);  // tension pulls body1 to body2
      if (f.body2 !== "ground") add(f, "force", (k) => [worldPoint(f.body2, f.point2, k), e(k).multiplyScalar(-s[k])]);
    } else if (f.type === "bushing" && has("force_x")) {
      const frame = (k) => rot(f.body2, k).multiply(q(f.orientation2));
      add(f, "force", (k) => [worldPoint(f.body1, f.point1, k), vec(`${n}.force`, k).applyQuaternion(frame(k))]);
      add(f, "torque", (k) => [worldPoint(f.body1, f.point1, k), vec(`${n}.torque`, k).applyQuaternion(frame(k))]);
    } else if (f.type !== "tire" && f.type !== "sensor" && f.point1 && has("force_x")) {  // types of extensions
      add(f, "force", (k) => [worldPoint(f.body1, f.point1, k), vec(`${n}.force`, k)]);
    } else if (f.type === "tire" && has("force_x") && has("contact_x")) {
      add(f, "force", (k) => {
        const P = vec(`${n}.contact`, k), C = worldPoint(f.body1, [0, 0, 0], k);
        const a = new THREE.Vector3(0, 1, 0).applyQuaternion(rot(f.body1, k));  // wheel axis
        const ex = P.clone().sub(C).cross(a).normalize();
        const nz = new THREE.Vector3(0, 0, 1).addScaledVector(ex, -ex.z).normalize();  // upright, normal to ex
        const ey = nz.clone().cross(ex), F = vec(`${n}.force`, k);
        return [P, ex.multiplyScalar(F.x).add(ey.multiplyScalar(F.y)).add(nz.multiplyScalar(F.z))];
      });
    }
  }
  const largest = { force: 0, torque: 0 }, step = Math.max(1, Math.floor(results.n / 300));
  for (const a of list) for (let k = 0; k < results.n; k += step) largest[a.kind] = Math.max(largest[a.kind], a.at(k)[1].length());
  for (const a of list) {
    const color = a.kind === "force" ? FORCE_COLOR : TORQUE_COLOR;
    a.helper = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(), 1, color);
    a.helper.userData.forceName = a.name;
    for (const o of [a.helper.line, a.helper.cone]) { o.material.depthTest = false; o.renderOrder = 13; }
    if (a.kind === "torque") {  // a double head
      const c2 = a.helper.cone.clone(); c2.material = a.helper.cone.material; c2.renderOrder = 13;
      a.helper.add(c2); a.second = c2;
    }
    group.add(a.helper);
  }
  updaters.push((k) => {
    for (const a of list) {
      const [p, v] = a.at(k), size = largest[a.kind] ? v.length() / largest[a.kind] : 0;
      const len = size * 0.3 * modelSize * state.arrowScale;
      a.helper.visible = len > 1e-4 * modelSize;
      if (!a.helper.visible) continue;
      a.helper.position.copy(p);
      a.helper.setDirection(v.normalize());
      const head = Math.min(0.3 * len, 0.04 * modelSize);
      a.helper.setLength(len, head, 0.5 * head);
      if (a.second) { a.second.scale.copy(a.helper.cone.scale); a.second.position.y = a.helper.cone.position.y - 0.7 * head; }
    }
  });
  return group;
}
$("show-reference").addEventListener("change", applyToggles);

// ---------- time ----------
let lastFollow = null;
function setTime(time) {
  if (state.mode) { state.mode = null; panels.modeStopped(); }  // back to the motion
  state.time = time;
  const k = results ? index(results.t, time) : 0;
  for (const b of bodies) {
    const [R, Q] = bodyPose(b, k);
    b.group.position.copy(R);
    b.group.quaternion.copy(Q);
    if (b.ghost) {
      const j = index(reference.t, time), [x, y, z, t0, t1, t2, t3] = b.prevCols;
      b.ghost.position.set(x[j], y[j], z[j]);
      b.ghost.quaternion.set(t1[j], t2[j], t3[j], t0[j]).normalize();
    }
  }
  for (const u of updaters) u(k);
  const fb = bodies.find((b) => b.name === state.follow);
  if (fb) {
    const p = fb.group.position.clone();
    if (lastFollow) { const d = p.clone().sub(lastFollow); camera.position.add(d); controls.target.add(d); }
    lastFollow = p;
  } else lastFollow = null;
  if (results) {
    const t = results.t, span = t[t.length - 1] - t[0] || 1;
    $("time").value = Math.round((time - t[0]) / span * 1000);
    $("tlabel").textContent = `t = ${time.toFixed(3)} s`;
  }
  area.setTime(time);
  if (state.selection) showInspector();
  render();
}

// ---------- eigenmodes ----------
// A mode of fdgen.linearize.modes about the results at a time: the bodies move by Re(x e^(i phi))
// (translation global, rotation in the body frame), the largest motion 8 % of the model size
// times the amplitude, one cycle in 1.5 s (not the frequency of the mode).
const MODE_PERIOD = 1.5;
function showMode(mode, time, amplitude = 1) {
  setTime(results ? results.t[index(results.t, time)] : 0);
  if (state.playing) togglePlay();
  const shapes = new Map();
  let tmax = 0, rmax = 0;
  for (const [name, c] of Object.entries(mode.bodies)) {
    const part = (j, i) => new THREE.Vector3(c[j][i], c[j + 1][i], c[j + 2][i]);
    const x = { tRe: part(0, 0), tIm: part(0, 1), rRe: part(3, 0), rIm: part(3, 1) };
    tmax = Math.max(tmax, Math.hypot(x.tRe.length(), x.tIm.length()));
    rmax = Math.max(rmax, Math.hypot(x.rRe.length(), x.rIm.length()));
    shapes.set(name, x);
  }
  const scale = 0.08 * modelSize / Math.max(tmax, 0.5 * modelSize * rmax, 1e-30);
  state.mode = { mode, k0: results ? index(results.t, time) : 0, shapes, scale, amplitude, phase: 0, offsets: new Map() };
  modeFrame();
}
function modeFrame() {
  const m = state.mode, c = Math.cos(m.phase), s = Math.sin(m.phase), A = m.scale * m.amplitude;
  for (const [name, x] of m.shapes) {
    const R = x.tRe.clone().multiplyScalar(c).addScaledVector(x.tIm, -s).multiplyScalar(A);
    const w = x.rRe.clone().multiplyScalar(c).addScaledVector(x.rIm, -s).multiplyScalar(A), angle = w.length();
    const Q = angle > 0 ? new THREE.Quaternion().setFromAxisAngle(w.divideScalar(angle), angle) : new THREE.Quaternion();
    m.offsets.set(name, { R, Q });
  }
  for (const b of bodies) { const [R, Q] = bodyPose(b, m.k0); b.group.position.copy(R); b.group.quaternion.copy(Q); }
  for (const u of updaters) u(m.k0);
  render();
}
function stopMode() { if (state.mode) setTime(state.time); }
const refresh = () => (state.mode ? modeFrame() : setTime(state.time));

$("time").addEventListener("input", (e) => {
  if (!results) return;
  const t = results.t;
  setTime(t[0] + (t[t.length - 1] - t[0]) * e.target.value / 1000);
});
$("speed").addEventListener("change", (e) => { state.speed = +e.target.value; });
function togglePlay() {
  if (!results) return;
  state.playing = !state.playing;
  $("play").textContent = state.playing ? "❚❚" : "▶";
  if (state.playing && state.time >= results.t[results.t.length - 1]) setTime(results.t[0]);
  lastFrame = performance.now();
}
$("play").addEventListener("click", togglePlay);
window.addEventListener("keydown", (e) => {
  if (e.target.tagName === "INPUT" && e.target.type !== "range") return;
  if (e.code === "Space") { e.preventDefault(); togglePlay(); }
  if (!results) return;
  const t = results.t, k = index(t, state.time);
  if (e.code === "ArrowRight") setTime(t[Math.min(t.length - 1, k + 1)]);
  if (e.code === "ArrowLeft") setTime(t[Math.max(0, k - 1)]);
});
function follow(name) {
  // the camera keeps its offset to the body
  state.follow = name;
  $("follow").value = name;
  const b = bodies.find((x) => x.name === name);
  lastFollow = b ? b.group.position.clone() : null;
}
$("follow").addEventListener("change", (e) => follow(e.target.value));

// ---------- lists, plots ----------
// Formulas are derived channels (fdview/channels.js), evaluated on the results and on every
// other run; the plot layout and the formulas are kept per model in the browser.
let formulas = [];  // {name, expr, tree}
const derived = new WeakMap();  // run -> Map(name -> values)

function channel(run, name) {
  if (!run) return null;
  const v = run.map.get(name);
  if (v) return v;
  const f = formulas.find((x) => x.name === name);
  if (!f) return null;
  if (!derived.has(run)) derived.set(run, new Map());
  const cache = derived.get(run);
  if (!cache.has(name)) {
    try { cache.set(name, evaluate(f.tree, run.t, (c) => channel(run, c))); } catch (e) { cache.set(name, null); }
  }
  return cache.get(name);
}
const curveRefs = () => ((model && model.checks && model.checks.references) || []).filter((r) => r.kind === "curve" && r.ref_x);
const area = new PlotArea($("plot"), {
  time: () => (results ? results.t : null),
  get: (n) => channel(results, n),
  others: (n) => compared().map((c) => {
    const y = channel(c.run, n);
    return y ? { id: c.id, t: c.run.t, y, dash: c.dash, reference: c.reference } : null;
  }).filter(Boolean),
  reference: (n) => { const y = channel(reference, n); return y ? { t: reference.t, y } : null; },
  delta: () => !!reference && $("plot-delta").classList.contains("on"),
  live: (n) => { const y = live && channel(live, n); return y ? { t: live.t, y } : null; },
  stale: () => document.body.classList.contains("stale"),
  fan: (n) => (sweep && sweep.shown ? sweep.runs.map((run, i) => {
    const y = run && channel(run, n);
    return y ? { t: run.t, y, color: scaleColor(i, sweep.values.length) } : null;
  }).filter(Boolean) : []),
  sweep: () => sweep && { ...sweep, channel, label: tuneLabel(sweep.path), current: sweepCurrent() },
  references: (n) => curveRefs().filter((r) => r.channel === n && r.x_channel === "t").map((r) => ({ x: r.ref_x, y: r.ref_y })),
  xyReferences: (xn, yn) => curveRefs().filter((r) => r.channel === yn && r.x_channel === xn).map((r) => ({ x: r.ref_x, y: r.ref_y })),
}, { onseek: (time) => setTime(time), onchange: () => { saveLayout(); showChannels(); area.updateTable(); } });

function viewImage() {
  // the 3D view as a PNG data URL (rendered now: the drawing buffer is not kept)
  renderer.render(world, camera);
  return renderer.domElement.toDataURL("image/png");
}
const panels = new Panels({
  static: !!STATIC, area, state, viewImage, showMode, stopMode, select, setTime, render,
  model: () => model, results: () => results,
});

const storeKey = () => `fdview.plots.${model ? model.model : ""}`;
function saveLayout() {
  if (!model) return;
  try { localStorage.setItem(storeKey(), JSON.stringify({ ...area.layout(), formulas: formulas.map(({ name, expr }) => ({ name, expr })) })); } catch (e) { /* no storage */ }
}
function restoreLayout() {
  let saved = null;
  try { saved = JSON.parse(localStorage.getItem(storeKey()) || "null"); } catch (e) { saved = null; }
  formulas = [];
  for (const f of (saved && saved.formulas) || []) try { formulas.push({ ...f, tree: parse(f.expr) }); } catch (e) { /* dropped */ }
  area.restore(saved, (n) => !results || results.map.has(n) || formulas.some((f) => f.name === n));
}

function addFormula(text) {
  // "name = expression" or just the expression (its name is the expression)
  const m = text.match(/^\s*([A-Za-z_][\w.]*)\s*=(?!=)(.*)$/);
  const [name, expr] = m ? [m[1], m[2].trim()] : [text.trim(), text.trim()];
  const tree = parse(expr);
  const deps = [...channelsOf(tree)].filter((c) => c !== "t" && !channel(results, c) && !formulas.some((f) => f.name === c));
  if (deps.length) throw new Error(`unknown channel ${deps[0]}`);
  if (results && results.map.has(name)) throw new Error(`${name} is a result channel`);
  formulas = formulas.filter((f) => f.name !== name).concat([{ name, expr, tree }]);
  for (const r of [results, reference, live, ...runCache.values()]) if (r) derived.delete(r);
  area.addChannel(name);
}

function fillLists() {
  const select = $("follow");
  select.innerHTML = '<option value="">nothing</option>' + bodies.map((b) => `<option>${b.name}</option>`).join("");
  select.value = bodies.some((b) => b.name === state.follow) ? state.follow : "";
  state.follow = select.value;
  lastFollow = null;
  $("bodies").innerHTML = "";
  for (const b of bodies) {
    const l = document.createElement("label");
    l.innerHTML = `<input type="checkbox" checked><span style="display:inline-block;width:10px;height:10px;border-radius:2px;background:${b.color}"></span>${b.name}`;
    l.querySelector("input").addEventListener("change", (e) => { b.group.visible = e.target.checked; render(); });
    $("bodies").appendChild(l);
  }
  showChannels();
}

function showChannels() {
  const box = $("channels");
  box.innerHTML = "";
  if (!results) { box.textContent = "no results"; return; }
  const words = $("filter").value.toLowerCase().split(/\s+/).filter(Boolean);
  const used = new Set(area.selected());
  const names = [...formulas.map((f) => f.name), ...results.names.filter((n) => n !== "t")]
    .filter((n) => words.every((w) => n.toLowerCase().includes(w)));
  for (const n of names.slice(0, 2000)) {
    const d = document.createElement("div");
    const f = formulas.find((x) => x.name === n);
    d.textContent = f ? `ƒ ${n}` : n;
    d.title = f ? `${n} = ${f.expr} (drag onto a plot; right click: delete)` : `${n} ${unit(n) ? "[" + unit(n) + "]" : ""} (click: active plot, or drag onto a plot)`;
    d.draggable = true;
    if (used.has(n)) { d.className = "on"; d.style.background = area.color(n); }
    d.addEventListener("dragstart", (e) => e.dataTransfer.setData("text/plain", n));
    d.addEventListener("click", () => area.addChannel(n));
    if (f) d.addEventListener("contextmenu", (e) => {
      e.preventDefault();
      formulas = formulas.filter((x) => x.name !== n);
      for (const p of area.panes) p.series = p.series.filter((s) => s.name !== n);
      area.build();
    });
    box.appendChild(d);
  }
}
$("filter").addEventListener("input", showChannels);

// toolbar of the plots
function ask(placeholder, onok) {
  // one inline field for formulas and the x channel of an x-y plot
  const box = $("plot-ask"), input = box.querySelector("input"), err = box.querySelector(".err");
  box.style.display = "flex"; input.value = ""; input.placeholder = placeholder; err.textContent = "";
  input.focus();
  input.onkeydown = (e) => {
    if (e.key === "Escape") box.style.display = "none";
    if (e.key !== "Enter") return;
    try { onok(input.value); box.style.display = "none"; } catch (x) { err.textContent = x.message; }
  };
}
$("plot-add-time").addEventListener("click", () => area.addPane("time"));
$("plot-add-fft").addEventListener("click", () => {
  const src = area.panes[area.active];  // the spectrum of the active time plot's channels
  const pane = area.addPane("fft");
  if (src && src.mode === "time") { pane.series = src.series.map((s) => ({ ...s })); area.build(); }
});
$("plot-add-xy").addEventListener("click", () => ask("x channel, e.g. wheel_fl1.R_z", (x) => {
  x = x.trim();
  if (!channel(results, x)) throw new Error(`unknown channel ${x}`);
  area.addPane("xy", x);
}));
$("plot-formula").addEventListener("click", () => ask("name = formula, e.g. v_kmh = chassis1.Rdot_x * 3.6;  der(), int(), hypot(), deg()", addFormula));
$("plot-measure").addEventListener("click", (e) => { e.currentTarget.classList.toggle("on"); area.setMeasure(e.currentTarget.classList.contains("on")); });
$("plot-reset").addEventListener("click", () => { area.xRange = null; for (const p of area.panes) p.view = null; area.draw(); });
function download(name, href) { const a = document.createElement("a"); a.href = href; a.download = name; a.click(); }
$("plot-png").addEventListener("click", () => { const u = area.png(); if (u) download(`${model.model}_plots.png`, u); });
$("plot-csv").addEventListener("click", () => download(`${model.model}_plots.csv`, URL.createObjectURL(new Blob([area.csv()], { type: "text/csv" }))));
$("plot-max").addEventListener("click", () => setPlotHeight(document.body.classList.toggle("plots-max") ? null : plotHeight));

// the split between the 3D view and the plots
let plotHeight = Math.round(window.innerHeight * 0.45);  // default: 45 % of the window
try { plotHeight = +localStorage.getItem("fdview.plotHeight") || plotHeight; } catch (e) { /* default */ }
function setPlotHeight(h) {
  document.body.style.setProperty("--plot-h", h === null ? "calc(100vh - 160px)" : `${h}px`);  // the 3D view follows (ResizeObserver)
}
setPlotHeight(plotHeight);
$("plot-split").addEventListener("pointerdown", (e) => {
  const y0 = e.clientY, h0 = $("plot").getBoundingClientRect().height;
  document.body.classList.remove("plots-max");
  const move = (ev) => { plotHeight = Math.max(120, Math.min(window.innerHeight - 160, h0 - (ev.clientY - y0))); setPlotHeight(plotHeight); };
  const up = () => { window.removeEventListener("pointermove", move); try { localStorage.setItem("fdview.plotHeight", plotHeight); } catch (x) { /* ok */ } };
  window.addEventListener("pointermove", move);
  window.addEventListener("pointerup", up, { once: true });
});

// ---------- checks ----------
const esc = (t) => String(t).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);
const fmtNum = (v) => (typeof v === "number" ? String(+v.toPrecision(4)) : v);

function renderChecks() {
  const box = $("checks"), c = model && model.checks;
  if (!c) { box.textContent = "no checks"; return; }
  const s = c.summary;
  box.innerHTML = `<div class="summary">${s.bodies} bodies · ${s.joints} joints · ${s.forces} forces · ` +
    `${s.degrees_of_freedom} DOF · ${fmtNum(s.total_mass)} kg</div>`;
  const add = (sev, label, text, onclick) => {
    const d = document.createElement("div");
    d.className = "item" + (onclick ? " clickable" : "");
    d.innerHTML = `<span class="sev ${sev}">${label}</span><span>${esc(text)}</span>`;
    if (onclick) d.addEventListener("click", onclick);
    box.appendChild(d);
  };
  for (const f of c.findings) {
    const target = f.elements.find((n) => findElement(n));
    add(f.severity, f.severity.toUpperCase(), f.message, target ? () => select(target) : null);
  }
  for (const r of c.references || []) {
    const ok = r.passed ? "passed" : "failed";
    const detail = r.error ? `not checked: ${r.error}` : r.kind === "curve"
      ? `max deviation ${fmtNum(r.max_deviation)} at ${r.x_channel} = ${fmtNum(r.at_x)} (tolerance ${fmtNum(r.tolerance)})` +
        (r.note ? `; ${r.note}` : "")
      : `${fmtNum(r.value)} against ${fmtNum(r.reference)} (tolerance ${fmtNum(r.tolerance)})`;
    add(ok, ok === "passed" ? "OK" : "FAIL", `${r.name}: ${detail}`, r.kind === "mass" ? null : () => showReference(r));
  }
  if (!c.findings.length && !(c.references || []).length) add("passed", "OK", "no findings");
}

function showReference(r) {
  if (r.kind === "curve") area.showReference(r);
  else {
    area.addChannel(r.channel);
    setTime(r.time ?? 0);
  }
}

// ---------- selection: inspector and highlight ----------
function findElement(name) {
  if (!model) return null;
  const b = model.bodies.find((x) => x.name === name);
  if (b) return { kind: "body", name, data: b };
  const j = model.joints.find((x) => x.name === name);
  if (j) return { kind: "joint", name, data: j };
  const f = model.forces.find((x) => x.name === name);
  if (f) return { kind: "force", name, data: f };
  const p = (model.points || []).find((x) => x.name === name);
  return p ? { kind: "point", name, data: p } : null;
}

function connectedTo(name) {
  // elements and bodies connected to an element: for a body its joints and forces, for a joint or force its bodies
  const e = findElement(name);
  if (!e) return { bodies: new Set(), elements: new Set() };
  if (e.kind === "point") {
    const els = [...model.joints, ...model.forces].filter((x) => e.data.users.includes(x.name));
    return { bodies: new Set([e.data.body, ...e.data.users.filter((u) => model.bodies.some((b) => b.name === u)),
      ...els.flatMap((x) => [x.body1, x.body2])]), elements: new Set([name, ...e.data.users]) };
  }
  if (e.kind !== "body") return { bodies: new Set([e.data.body1, e.data.body2]), elements: new Set([name]) };
  const els = [...model.joints, ...model.forces].filter((x) => x.body1 === name || x.body2 === name);
  return { bodies: new Set([name, ...els.flatMap((x) => [x.body1, x.body2])]), elements: new Set(els.map((x) => x.name)) };
}

function select(name) {
  state.selection = name ? findElement(name) : null;
  highlight();
  showInspector();
  render();
}

function highlight() {
  const sel = state.selection, rel = sel ? connectedTo(sel.name) : null;
  for (const b of bodies) {
    const on = !rel || rel.bodies.has(b.name);
    b.group.traverse((o) => {
      if (!o.isMesh || !o.material.isMeshStandardMaterial) return;
      const m = o.material;
      if (!m.userData.orig) m.userData.orig = { opacity: m.opacity, transparent: m.transparent, depthWrite: m.depthWrite };
      const orig = m.userData.orig;
      m.opacity = on ? orig.opacity : 0.12;
      m.transparent = on ? orig.transparent : true;
      m.depthWrite = on ? orig.depthWrite : false;
      m.emissive.setHex(sel && sel.kind === "body" && sel.name === b.name ? 0x2f4f8f : 0x000000);
      m.needsUpdate = true;
    });
  }
  root.traverse((o) => {
    const name = o.userData.jointName || o.userData.forceName || o.userData.pointName;
    if (!name || !o.material) return;
    const picked = sel && sel.name === name, on = !rel || rel.elements.has(name);
    o.material.opacity = on ? 0.9 : 0.15;
    if (o.isMesh) o.scale.setScalar(picked ? 1.8 : 1);  // markers and glyphs; lines are in world coordinates
    else o.material.color.setHex(picked ? 0xffd43b : lineColor(o));
  });
}

function lineColor(o) {
  const f = model.forces.find((x) => x.name === o.userData.forceName);
  return f && f.type === "tire" ? TIRE_COLOR : SPRING_COLOR;
}

function showInspector() {
  const box = $("inspector"), sel = state.selection;
  if (!sel) { box.style.display = "none"; return; }
  const d = sel.data, k = results ? index(results.t, state.time) : 0;
  const link = (n) => (findElement(n) ? `<span class="link" data-select="${esc(n)}">${esc(n)}</span>` : esc(n));
  let html = `<button class="close" title="close (Esc)">×</button><h3>${esc(sel.name)}</h3>`;
  if (sel.kind === "body") {
    const [R] = bodyPose(bodies.find((b) => b.name === sel.name), k);
    const rel = connectedTo(sel.name);
    html += `<div class="kind">body · ${fmtNum(d.mass)} kg</div>` +
      `<div>position at t = ${state.time.toFixed(3)} s: ${[R.x, R.y, R.z].map(fmtNum).join(", ")} m</div>` +
      `<div>connected by: ${[...rel.elements].map(link).join(", ") || "nothing"}</div>`;
  } else if (sel.kind === "point") {
    html += `<div class="kind">${d.kind === "frame" ? "frame" : "hardpoint"} · on ${link(d.body)}</div>` +
      `<div>at t = 0: ${d.global.map(fmtNum).join(", ")} m</div>` +
      `<div>used by: ${d.users.map(link).join(", ") || "nothing"}</div>`;
  } else if (d.type === "sensor") {
    html += `<div class="kind">sensor</div><div>on ${link(d.body1)}</div>`;
  } else {
    html += `<div class="kind">${sel.kind} · ${esc(d.type)}${d.fix ? " · fixes " + d.fix.join(" ") : ""}</div>` +
      `<div>between ${link(d.body1)} and ${link(d.body2)}</div>`;
  }
  const findings = ((model.checks && model.checks.findings) || []).filter((f) => f.elements.includes(sel.name));
  for (const f of findings) html += `<div><span class="sev ${f.severity}" style="color:#fff;border-radius:3px;padding:0 4px;font-size:10px">${f.severity.toUpperCase()}</span> ${esc(f.message)}</div>`;
  if (results) {
    const chans = results.names.filter((n) => n.startsWith(sel.name + "."));
    if (chans.length) html += `<div style="margin-top:6px">channels: ${chans.slice(0, 40).map((n) =>
      `<span class="link" data-channel="${esc(n)}">${esc(n.slice(sel.name.length + 1))}</span>`).join(" ")}</div>`;
  }
  if (d.tunable && d.tunable.length && !STATIC) {
    // numbers: a click puts them into Tune; formulas (=...): their parameters
    const tune = (path, text) => `<span class="link" data-tune="${esc(path)}" title="${esc(path)}">${esc(text)}</span>`;
    const nums = d.tunable.filter((x) => x.value !== undefined), forms = d.tunable.filter((x) => x.formula);
    if (nums.length) html += `<div style="margin-top:6px">tune: ${nums.slice(0, 40).map((x) =>
      tune(x.path, tuneLabel(x.path).split(".").slice(1).join("."))).join(" ")}</div>`;
    if (forms.length) html += `<div style="margin-top:4px">${forms.slice(0, 20).map((x) =>
      `${esc(tuneLabel(x.path).split(".").slice(1).join("."))} ${esc(x.formula)}` +
      (x.uses.length ? ` (${x.uses.map((u) => tune(`parameters.${u}`, u)).join(", ")})` : "")).join("<br>")}</div>`;
  }
  if (d.source) html += `<div style="margin-top:6px;color:var(--muted)">model file, line ${d.source.line ?? "?"}</div><pre>${esc(d.source.yaml)}</pre>`;
  box.innerHTML = html;
  box.style.display = "block";
  box.querySelector(".close").addEventListener("click", () => select(null));
  box.querySelectorAll("[data-select]").forEach((e) => e.addEventListener("click", () => select(e.dataset.select)));
  box.querySelectorAll("[data-tune]").forEach((e) => e.addEventListener("click", () => panels.pin(e.dataset.tune)));
  box.querySelectorAll("[data-channel]").forEach((e) => e.addEventListener("click", () => {
    area.addChannel(e.dataset.channel);
  }));
}

// a click (not a drag) picks the body, joint or force under the pointer
const raycaster = new THREE.Raycaster();
let downAt = null;
renderer.domElement.addEventListener("pointerdown", (e) => { downAt = [e.clientX, e.clientY]; });
renderer.domElement.addEventListener("pointerup", (e) => {
  if (!downAt || Math.hypot(e.clientX - downAt[0], e.clientY - downAt[1]) > 4) return;
  const r = renderer.domElement.getBoundingClientRect();
  raycaster.setFromCamera(new THREE.Vector2((e.clientX - r.left) / r.width * 2 - 1, -(e.clientY - r.top) / r.height * 2 + 1), camera);
  raycaster.params.Line.threshold = 0.01 * modelSize;
  const pickable = [];
  root.traverse((o) => { if (o.isMesh || o.isLine) pickable.push(o); });
  for (const hit of raycaster.intersectObjects(pickable, false)) {
    let o = hit.object, name = null;
    for (; o && !name; o = o.parent) name = o.userData.jointName || o.userData.forceName || o.userData.pointName || o.userData.body;
    if (name && findElement(name) && isShown(hit.object)) { select(name); return; }
  }
  select(null);
});
const isShown = (o) => { for (; o; o = o.parent) if (!o.visible) return false; return true; };
window.addEventListener("keydown", (e) => { if (e.code === "Escape") select(null); });

// ---------- camera ----------
function motionBox() {
  // all positions of the bodies over the results (sampled), widened by their shapes
  const box = new THREE.Box3(), v = new THREE.Vector3();
  for (const b of bodies) {
    const r = Math.max(0, ...b.shapes.map((s) => new THREE.Vector3(...s.position).length()
      + (s.size ? new THREE.Vector3(...s.size).length() / 2 : (s.radius || 0) + (s.length || 0) / 2)));
    const n = b.cols ? results.n : 1, step = Math.max(1, Math.floor(n / 400));
    for (let k = 0; k < n; k += step) {
      v.copy(bodyPose(b, k)[0]);
      box.expandByPoint(v.clone().addScalar(r)).expandByPoint(v.clone().addScalar(-r));
    }
  }
  return box;
}

function fit() {
  const box = state.follow ? new THREE.Box3() : motionBox();
  if (state.follow) for (const b of bodies) box.expandByObject(b.group);
  if (box.isEmpty()) return;
  const c = box.getCenter(new THREE.Vector3()), r = Math.max(box.getSize(new THREE.Vector3()).length() / 2, 0.05);
  const d = r / Math.tan(THREE.MathUtils.degToRad(camera.fov / 2)) * 1.05;
  controls.target.copy(c);
  camera.position.copy(c).add(new THREE.Vector3(-0.55, -0.75, 0.38).normalize().multiplyScalar(d));
  camera.near = d / 1000; camera.far = d * 1000; camera.updateProjectionMatrix();
  lastFollow = null;
  render();
}
$("fit").addEventListener("click", fit);

// ---------- loop ----------
let needsRender = true, lastFrame = performance.now();
function render() { needsRender = true; }
controls.addEventListener("change", render);
function loop(now) {
  requestAnimationFrame(loop);
  if (state.mode) { state.mode.phase += (now - lastFrame) / 1000 * 2 * Math.PI / MODE_PERIOD; modeFrame(); }
  if (state.playing && results) {
    const t = results.t, end = t[t.length - 1];
    let time = state.time + (now - lastFrame) / 1000 * state.speed;
    if (time > end) time = t[0];
    setTime(time);
  }
  lastFrame = now;
  controls.update();
  if (needsRender) { needsRender = false; renderer.render(world, camera); }
}
requestAnimationFrame(loop);

// ---------- loading, live reload ----------
function showError(text) {
  $("error").style.display = text ? "block" : "none";
  $("error").textContent = text || "";
}

function load(scene, csvText, current = null) {
  const first = model === null, before = model && model.model;
  results = csvText ? parseCSV(csvText) : null;
  if (results && current !== null) runCache.set(current, results);
  live = null;
  build(scene);
  if (before !== scene.model) restoreLayout();  // the plots of this model as last time
  else area.build();  // new results: redraw (formulas are evaluated again on them)
  $("model").textContent = scene.model;
  if (results) {
    const t = results.t;
    setTime(Math.min(Math.max(state.time, t[0]), t[t.length - 1]));
  } else setTime(0);
  if (first) {
    // a model that drives away (a vehicle): follow its heaviest body
    const main = bodies.filter((b) => b.cols).sort((a, b) => b.mass - a.mass)[0];
    if (main) {
      const [x, y, z] = main.cols, n = results.n - 1;
      const travel = Math.hypot(x[n] - x[0], y[n] - y[0], z[n] - z[0]);
      if (travel > 2 * modelSize) follow(main.name);
    }
    fit();
    follow(state.follow);
  }
}

// ---------- parameter sweep (fan) ----------
// One tuned number over several values, simulated in parallel by the server (fdgen.sweep):
// its runs are drawn as a fan of curves in the time plots and give the key figure plots.
let sweep = null;  // {id, path, values, runs: [parsed CSV | null], shown, status, results}

async function syncSweep(s) {
  const sw = s.sweep;
  if (!sw) { if (sweep) { sweep = null; area.build(); } return; }
  if (!sweep || sweep.id !== sw.id) sweep = { ...sw, runs: sw.values.map(() => null), shown: true };
  Object.assign(sweep, { status: sw.status, results: sw.results, error: sw.error, overrides: sw.overrides });
  const load = sw.results.map((r, k) => (r === "ok" && !sweep.runs[k] ? k : null)).filter((k) => k !== null);
  if (!load.length) return;
  const id = sweep.id;
  await Promise.all(load.map(async (k) => {
    const run = parseCSV(await fetch(`api/sweep/${k}.csv`, { cache: "no-store" }).then((r) => r.text()));
    if (sweep && sweep.id === id) sweep.runs[k] = run;
  }));
  area.draw();
}

function sweepCurrent() {
  // the value of the swept number in the current run: tuned, or that of the model file
  if (!sweep) return null;
  if (sweep.path in panels.overrides) return panels.overrides[sweep.path];
  const v = panels.values().get(sweep.path);
  return v === undefined ? null : v;
}

// ---------- runs ----------
// The server keeps every simulation as a run; the current one is `results`. Runs shown in the
// plots (`shown`) and the reference are loaded when needed and kept here.
const runCache = new Map();  // run id -> parsed CSV
const shown = new Set();  // ids of the runs compared in the plots, besides the reference
const DASHES = [[2, 3], [10, 3, 2, 3], [5, 2], [1, 3], [12, 3, 2, 3, 2, 3]];
const REFERENCE_DASH = [7, 4];
let runList = [], currentRun = null, referenceRun = null, live = null;

function runDash(id) {
  if (id === referenceRun) return REFERENCE_DASH;
  const others = [...shown].filter((x) => x !== referenceRun && x !== currentRun).sort((a, b) => a - b);
  return DASHES[Math.max(0, others.indexOf(id)) % DASHES.length];
}

function compared() {
  // the runs drawn besides the current one: the reference first
  const ids = [...new Set([referenceRun, ...shown])].filter((id) => id !== null && id !== currentRun && runCache.has(id));
  return ids.map((id) => ({ id, run: runCache.get(id), dash: runDash(id), reference: id === referenceRun }));
}

async function syncRuns(s) {
  runList = s.runs || [];
  currentRun = s.current;
  const ids = new Set(runList.map((r) => r.id));
  for (const id of [...shown]) if (!ids.has(id)) shown.delete(id);
  for (const id of [...runCache.keys()]) if (!ids.has(id)) runCache.delete(id);
  const need = [...new Set([s.reference, ...shown])].filter((id) => id !== null && ids.has(id) && !runCache.has(id));
  await Promise.all(need.map(async (id) => {
    const text = await fetch(`api/runs/${id}.csv`, { cache: "no-store" }).then((r) => r.text());
    const run = text && parseCSV(text);
    if (run) runCache.set(id, run);
  }));
  const ref = s.reference !== null && s.reference !== currentRun ? runCache.get(s.reference) || null : null;
  if (s.reference !== referenceRun || ref !== reference) {
    referenceRun = s.reference;
    reference = ref;
    $("show-reference").disabled = !reference;
    $("plot-delta").disabled = !reference;
    if (!reference) $("plot-delta").classList.remove("on");
    if (model) { build(model); refresh(); }  // the grey bodies of the reference
  }
  strip.render(runList, currentRun, referenceRun, shown, sweep);
  area.draw();
}

async function post(url, body) {
  const r = await fetch(url, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
  if (!r.ok) throw new Error((await r.json().catch(() => ({}))).error || r.status);
  return r.json();
}
const strip = new RunStrip($("runs"), {
  toggle: (id) => { if (shown.has(id)) shown.delete(id); else shown.add(id); pollSoon(); },
  reference: (id) => post(`api/runs/${id ?? referenceRun}`, { reference: id !== null }).then(pollSoon),
  restore: (run) => panels.restore(run.overrides),
  remove: (id) => post(`api/runs/${id}`, { delete: true }).then(pollSoon),
  dash: runDash,
  sweepShown: () => { if (sweep) { sweep.shown = !sweep.shown; pollSoon(); } },
  sweepStop: () => post("api/sweep", { clear: true }).then(pollSoon),
});
$("plot-add-kpi").addEventListener("click", () => area.addKpi());
$("plot-delta").addEventListener("click", (e) => { if (!reference) return; e.currentTarget.classList.toggle("on"); area.draw(); });

let version = -1, timer = null;
function pollSoon() { clearTimeout(timer); timer = setTimeout(poll, 0); }
async function poll() {
  let s = null;
  try {
    s = await (await fetch("api/state", { cache: "no-store" })).json();
    const busy = s.status === "simulating" || s.status === "loading";
    panels.serverState(s);
    $("status").textContent = s.model_file;
    if (s.version !== version && !busy) {
      version = s.version;
      // an error is shown over the last good model (loaded if the page has none yet)
      showError(s.status === "error" ? s.error + (s.log ? "\n\n" + s.log : "") : "");
      if (s.scene && (s.status !== "error" || model === null)) {
        const [scene, csv] = await Promise.all([
          fetch("api/scene", { cache: "no-store" }).then((r) => r.json()),
          s.results ? fetch("api/results.csv", { cache: "no-store" }).then((r) => r.text()) : Promise.resolve(""),
        ]);
        load(scene, csv, s.current);
      }
    }
    if (s.status === "simulating" && results) {  // the run so far, drawn over the faded results
      const run = parseCSV(await fetch("api/running.csv", { cache: "no-store" }).then((r) => r.text()));
      if (run) { live = run; area.draw(); }
    } else if (live) { live = null; area.draw(); }
    await syncSweep(s);
    await syncRuns(s);
  } catch (e) {
    $("status").textContent = "server not reachable";
  }
  clearTimeout(timer);
  timer = setTimeout(poll, s && (s.status === "simulating" || (s.sweep && s.sweep.status === "running")) ? 300 : 800);
}

$("overlay").textContent = "drag: rotate · right drag: pan · wheel: zoom · click: inspect · space: play";
if (STATIC) {
  $("status").textContent = "exported view";
  load(STATIC.scene, STATIC.csv);
} else poll();
window.fdViewer = { setTime, select, showReference, showMode, stopMode, panels, viewImage, addChannel: (n) => area.addChannel(n), addFormula, plots: area, state, camera, controls, root, bodies: () => bodies, get model() { return model; }, get results() { return results; } };
