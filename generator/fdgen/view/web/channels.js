// Result channels for the plots: units from the channel names, and derived channels from formulas.
//
// A formula combines channels by name (e.g. chassis1.Rdot_x * 3.6, hypot(tire_fl1.force_x,
// tire_fl1.force_y)) with + - * / ^, parentheses, numbers, pi, t and the functions below;
// der(x) is the time derivative (central differences), int(x) the time integral (trapezoids).
// The formula is parsed into a tree (nothing is evaluated as code) and evaluated on arrays.

const UNITS = [
  [/\.R_[xyz]$|\.contact_[xyz]$|\.penetration$|\.road_height$|\.length$|\.distance$/, "m"],
  [/\.Rdot_[xyz]$|\.velocity$/, "m/s"],
  [/\.omega(_body)?_[xyz]$|\.angular_rate$/, "rad/s"],
  [/\.theta[0-3]$|\.theta_norm$|\.slip[01]$|\.friction$/, "–"],
  [/\.T$/, "J"],
  [/\.H_[xyz]$/, "kg m²/s"],
  [/\.force(_[xyz])?$/, "N"],
  [/\.torque(_[xyz])?$|\.moment(_[xyz])?$/, "N m"],
  [/\.angle$/, "rad"],
];

export function unit(name) {
  for (const [re, u] of UNITS) if (re.test(name)) return u;
  return "";
}

const FUNCS = {
  sin: Math.sin, cos: Math.cos, tan: Math.tan, asin: Math.asin, acos: Math.acos, atan: Math.atan,
  atan2: Math.atan2, sqrt: Math.sqrt, abs: Math.abs, exp: Math.exp, log: Math.log, log10: Math.log10,
  min: Math.min, max: Math.max, hypot: Math.hypot, sign: Math.sign,
  deg: (x) => (x * 180) / Math.PI, rad: (x) => (x * Math.PI) / 180,
};

// ---------- parser: formula -> tree ----------
function tokens(src) {
  const out = [], re = /\s*(?:(\d+\.?\d*(?:[eE][-+]?\d+)?|\.\d+(?:[eE][-+]?\d+)?)|([A-Za-z_][\w.]*)|(\*\*|[-+*/^(),]))/y;
  let m;
  re.lastIndex = 0;
  while (re.lastIndex < src.length) {
    const at = re.lastIndex;
    if (!(m = re.exec(src))) {
      if (/^\s*$/.test(src.slice(at))) break;
      throw new Error(`unexpected '${src.slice(at).trim()[0]}' at ${at + 1}`);
    }
    out.push(m[1] ? { num: +m[1] } : m[2] ? { id: m[2] } : { op: m[3] === "**" ? "^" : m[3] });
  }
  return out;
}

export function parse(src) {
  const tk = tokens(src);
  let i = 0;
  const peek = () => tk[i], take = () => tk[i++];
  const expect = (op) => { const t = take(); if (!t || t.op !== op) throw new Error(`'${op}' expected`); };
  function primary() {
    const t = take();
    if (!t) throw new Error("incomplete formula");
    if (t.num !== undefined) return { num: t.num };
    if (t.op === "(") { const e = sum(); expect(")"); return e; }
    if (t.op === "-") return { fn: "neg", args: [power()] };
    if (t.op === "+") return power();
    if (t.id) {
      if (peek() && peek().op === "(") {
        take();
        const args = [];
        if (!(peek() && peek().op === ")")) { args.push(sum()); while (peek() && peek().op === ",") { take(); args.push(sum()); } }
        expect(")");
        if (!(t.id in FUNCS) && t.id !== "der" && t.id !== "int") throw new Error(`unknown function ${t.id}`);
        return { fn: t.id, args };
      }
      if (t.id === "pi") return { num: Math.PI };
      return { ch: t.id };
    }
    throw new Error(`unexpected '${t.op}'`);
  }
  function power() {
    const b = primary();
    if (peek() && peek().op === "^") { take(); return { op: "^", a: b, b: unaryPower() }; }
    return b;
  }
  function unaryPower() {  // right-associative, a unary minus binds weaker: 2^-1
    if (peek() && peek().op === "-") { take(); return { fn: "neg", args: [unaryPower()] }; }
    return power();
  }
  function product() {
    let a = unaryPower();
    while (peek() && (peek().op === "*" || peek().op === "/")) { const op = take().op; a = { op, a, b: unaryPower() }; }
    return a;
  }
  function sum() {
    let a = product();
    while (peek() && (peek().op === "+" || peek().op === "-")) { const op = take().op; a = { op, a, b: product() }; }
    return a;
  }
  const tree = sum();
  if (i < tk.length) throw new Error(`unexpected '${tk[i].op || tk[i].id || tk[i].num}'`);
  return tree;
}

export function channelsOf(tree, out = new Set()) {
  if (tree.ch) out.add(tree.ch);
  for (const k of ["a", "b"]) if (tree[k]) channelsOf(tree[k], out);
  for (const a of tree.args || []) channelsOf(a, out);
  return out;
}

// ---------- evaluation on arrays ----------
// lookup(name) -> Float64Array (the channel) or null; t: the time column.
export function evaluate(tree, t, lookup) {
  const n = t.length;
  const full = (v) => (typeof v === "number" ? new Float64Array(n).fill(v) : v);
  function ev(node) {
    if (node.num !== undefined) return node.num;
    if (node.ch) {
      if (node.ch === "t") return t;
      const v = lookup(node.ch);
      if (!v) throw new Error(`unknown channel ${node.ch}`);
      return v;
    }
    if (node.op) {
      const a = ev(node.a), b = ev(node.b);
      const f = { "+": (x, y) => x + y, "-": (x, y) => x - y, "*": (x, y) => x * y, "/": (x, y) => x / y, "^": Math.pow }[node.op];
      return map([a, b], f);
    }
    if (node.fn === "neg") return map([ev(node.args[0])], (x) => -x);
    if (node.fn === "der" || node.fn === "int") {
      if (node.args.length !== 1) throw new Error(`${node.fn} takes one argument`);
      const y = full(ev(node.args[0])), out = new Float64Array(n);
      if (node.fn === "der") {
        for (let k = 0; k < n; k++) {
          const a = Math.max(0, k - 1), b = Math.min(n - 1, k + 1);
          out[k] = (y[b] - y[a]) / (t[b] - t[a]);
        }
      } else for (let k = 1; k < n; k++) out[k] = out[k - 1] + 0.5 * (y[k] + y[k - 1]) * (t[k] - t[k - 1]);
      return out;
    }
    return map(node.args.map(ev), FUNCS[node.fn]);
  }
  function map(args, f) {
    if (args.every((a) => typeof a === "number")) return f(...args);
    const arrs = args.map(full), out = new Float64Array(n), xs = new Array(args.length);
    for (let k = 0; k < n; k++) { for (let j = 0; j < arrs.length; j++) xs[j] = arrs[j][k]; out[k] = f(...xs); }
    return out;
  }
  return full(ev(tree));
}
