// The model file in an editor beside the 3D view (CodeMirror 6, vendored): open it with a click on
// the file name in the header, on the YAML text in the inspector, on an error, or with the key E.
//
// - ctrl+s saves; the server writes the file and the viewer rebuilds and simulates as after any
//   other save. A file changed meanwhile (another editor, the assistant) is not overwritten: the
//   editor reloads it when there are no unsaved changes, else it asks.
// - Checked while typing (POST api/lint): YAML syntax, the schema, the model rules, each at its
//   line; the findings of the checks at the lines of their elements.
// - Completion (ctrl+space, or while typing) from the schema (api/schema) and the model: the keys
//   an element can have, the values of a type, the bodies, points, frames, measures and
//   parameters of the file; templates for new bodies, joints, forces, ... (they are the "add"
//   menu). Hovering a key shows what it means.
// - The element under the cursor is selected in the 3D view and the inspector; an element
//   selected there is shown here.

import {
  EditorState, StateEffect, StateField, Compartment, EditorView, keymap, lineNumbers, highlightActiveLine,
  highlightActiveLineGutter, drawSelection, Decoration, hoverTooltip, ViewPlugin, MatchDecorator, placeholder,
  defaultKeymap, history, historyKeymap, indentWithTab, syntaxHighlighting, HighlightStyle, indentOnInput,
  bracketMatching, foldGutter, foldKeymap, indentUnit, indentService, autocompletion, completionKeymap, snippetCompletion,
  closeBrackets, closeBracketsKeymap, linter, lintGutter, searchKeymap,
  highlightSelectionMatches, yaml, tags,
} from "fdview/codemirror";

const SECTIONS_OF_ELEMENTS = ["bodies", "joints", "forces", "sensors", "points", "frames"];
const BODY_KEYS = new Set(["body", "body1", "body2", "in"]);
const POINT_KEYS = new Set(["point", "point1", "point2", "location", "from", "mirror"]);
const EXPRESSION_KEYS = new Set(["force", "torque", "motion"]);  // force expressions, driver motion

// ---------- the outline of a model file: sections and their named entries, by indentation ----------
const KEY_LINE = /^(\s*)((?:- +)*)([A-Za-z_][\w-]*)\s*:(?=\s|$)(.*)$/;

export function outline(text) {
  // [{section, name, line, end}] (1-based lines) of the entries of the top-level sections
  const lines = text.split("\n"), out = [];
  let section = null, childIndent = null, last = null;
  const close = (i) => {  // the last entry ends before line i (without trailing blank lines)
    if (!last) return;
    let e = i;
    while (e > last.line && !lines[e - 1].trim()) e--;
    last.end = e;
    last = null;
  };
  lines.forEach((l, i) => {
    if (!l.trim() || l.trimStart().startsWith("#")) return;
    const indent = l.length - l.trimStart().length;
    if (indent === 0) {
      close(i);
      const m = KEY_LINE.exec(l);
      section = m ? m[3] : null;
      childIndent = null;
      return;
    }
    if (!section) return;
    const m = KEY_LINE.exec(l);
    if (!m || m[2]) return;
    if (childIndent === null) childIndent = indent;
    if (indent === childIndent) {
      close(i);
      last = { section, name: m[3], line: i + 1, end: i + 1 };
      out.push(last);
    }
  });
  close(lines.length);
  return out;
}

const namesIn = (items, section) => items.filter((x) => x.section === section).map((x) => x.name);

// ---------- where the cursor is: the keys above it ----------
function keyIndent(m) { return m[1].length + m[2].length; }

function context(doc, pos) {
  // {path: keys from the top to the map the cursor is in ("*" a list item), key position or the
  //  key whose value is typed, the word typed, siblings: keys already in that map}
  const line = doc.lineAt(pos), before = line.text.slice(0, pos - line.from);
  let m = /^(\s*)((?:- +)*)([\w-]*)$/.exec(before), at, key = null, valueText = "";
  if (m) at = { kind: "key", word: m[3], indent: m[1].length + m[2].length, dashes: m[2], spaces: m[1].length };
  else {
    m = /^(\s*)((?:- +)*)([A-Za-z_][\w-]*)\s*:\s*(.*)$/.exec(before);
    if (!m) return null;
    key = m[3]; valueText = m[4];
    at = { kind: "value", word: (/[\w.]*$/.exec(valueText) || [""])[0], indent: keyIndent(m), dashes: m[2], spaces: m[1].length };
  }
  const path = [], siblings = new Map(), val = (k) => k[4].replace(/\s+#.*$/, "").trim();
  let limit = at.indent;
  if (at.dashes) { path.push("*"); limit = at.spaces; }
  for (let n = line.number - 1; n >= 1 && limit > 0; n--) {
    const t = doc.line(n).text;
    if (!t.trim() || t.trimStart().startsWith("#")) continue;
    const k = KEY_LINE.exec(t);
    const spaces = t.length - t.trimStart().length;
    if (!k) {
      if (/^\s*- /.test(t) && spaces < limit && limit <= spaces + 2) { path.push("*"); limit = spaces; }
      else if (spaces < limit) limit = spaces;  // a line of another kind: do not look through it
      continue;
    }
    const c = keyIndent(k);
    if (c === limit && !path.length) siblings.set(k[3], val(k));
    if (c === limit && k[2]) { if (!path.length) siblings.set(k[3], val(k)); path.push("*"); limit = k[1].length; continue; }
    if (c < limit) {
      path.push(k[3]);
      limit = c;
      if (k[2]) { path.push("*"); limit = k[1].length; }
    }
  }
  path.reverse();
  // siblings below the cursor (the same map)
  for (let n = line.number + 1; n <= doc.lines; n++) {
    const t = doc.line(n).text;
    if (!t.trim() || t.trimStart().startsWith("#")) continue;
    const spaces = t.length - t.trimStart().length;
    if (spaces < at.indent) break;
    const k = KEY_LINE.exec(t);
    if (k && keyIndent(k) === at.indent && !k[2]) siblings.set(k[3], val(k));
  }
  if (at.kind === "key" && before.trim()) siblings.delete(at.word);
  return { ...at, key, valueText, path, siblings, lineFrom: line.from };
}

// ---------- the schema ----------
class Schema {
  constructor(root) { this.root = root; }

  deref(s) {
    for (let k = 0; s && s.$ref && k < 20; k++) s = s.$ref.split("/").slice(1).reduce((n, p) => n && n[p], this.root);
    return s || {};
  }

  variants(s) {
    s = this.deref(s);
    const subs = [...(s.anyOf || []), ...(s.oneOf || []), ...(s.allOf || [])];
    return [s, ...subs.flatMap((x) => this.variants(x))];
  }

  child(s, key) {
    // the schema of key in an object (or "*": an item of an array)
    let found = null, description = null;
    for (const v of this.variants(s)) {
      if (key === "*") { if (v.items && !found) found = v.items; continue; }
      if (v.properties && v.properties[key]) { found = found || v.properties[key]; description = description || v.properties[key].description; }
      else if (!found && v.additionalProperties && typeof v.additionalProperties === "object") found = v.additionalProperties;
    }
    if (!found) return null;
    const out = this.deref(found);
    return description && !out.description ? { ...out, description } : out;
  }

  at(path) {
    let s = this.root;
    for (const k of path) { s = s && this.child(s, k); if (!s) return null; }
    return s;
  }

  properties(s) {
    const out = {};
    for (const v of this.variants(s)) for (const [k, p] of Object.entries(v.properties || {})) if (!out[k]) out[k] = p;
    return out;
  }

  required(s) { return new Set(this.variants(s).flatMap((v) => v.required || [])); }

  isNamedMap(s) {
    // bodies, joints, ...: names of the user's choice
    return this.variants(s).some((v) => v.additionalProperties && typeof v.additionalProperties === "object") &&
      !Object.keys(this.properties(s)).length;
  }

  values(s) {
    const out = [];
    for (const v of this.variants(s)) {
      if (v.enum) out.push(...v.enum);
      if (v.const !== undefined) out.push(v.const);
      if (v.type === "boolean") out.push(true, false);
    }
    return [...new Set(out)];
  }

  description(s) {
    const d = this.variants(s).map((v) => v.description).find(Boolean);
    return d || "";
  }
}

// ---------- templates: new elements (the "add" menu) ----------
const TEMPLATES = {
  bodies: [
    ["body (box)", "${name}:\n\tmass: ${1.0}\n\tinertia: [${0.01}, ${0.01}, ${0.01}]\n\tposition: [${0}, ${0}, ${0}]\n\tshapes:\n\t\t- {type: box, size: [${0.2}, ${0.2}, ${0.2}]}"],
    ["body (cylinder)", "${name}:\n\tmass: ${1.0}\n\tinertia: [${0.01}, ${0.01}, ${0.005}]\n\tposition: [${0}, ${0}, ${0}]\n\tshapes:\n\t\t- {type: cylinder, radius: ${0.05}, length: ${0.3}}"],
    ["body (sphere)", "${name}:\n\tmass: ${1.0}\n\tinertia: [${0.004}, ${0.004}, ${0.004}]\n\tposition: [${0}, ${0}, ${0}]\n\tshapes:\n\t\t- {type: sphere, radius: ${0.1}}"],
  ],
  joints: [
    ["revolute joint", "${name}:\n\ttype: revolute\n\tbody1: ${body}\n\tbody2: ${ground}\n\tlocation: [${0}, ${0}, ${0}]\n\t# turns about the z axis of the joint frame; [0.7071, -0.7071, 0, 0]: about global y\n\torientation: [${1}, ${0}, ${0}, ${0}]"],
    ["spherical joint", "${name}:\n\ttype: spherical\n\tbody1: ${body}\n\tbody2: ${ground}\n\tlocation: [${0}, ${0}, ${0}]"],
    ["prismatic joint", "${name}:\n\ttype: prismatic\n\tbody1: ${body}\n\tbody2: ${ground}\n\tlocation: [${0}, ${0}, ${0}]\n\t# slides along the z axis of the joint frame\n\torientation: [${1}, ${0}, ${0}, ${0}]"],
    ["cylindrical joint", "${name}:\n\ttype: cylindrical\n\tbody1: ${body}\n\tbody2: ${ground}\n\tlocation: [${0}, ${0}, ${0}]\n\torientation: [${1}, ${0}, ${0}, ${0}]"],
    ["fixed joint", "${name}:\n\ttype: fixed\n\tbody1: ${body}\n\tbody2: ${ground}\n\tlocation: [${0}, ${0}, ${0}]"],
    ["generic joint", "${name}:\n\ttype: generic\n\tbody1: ${body}\n\tbody2: ${ground}\n\tlocation: [${0}, ${0}, ${0}]\n\tfix: [${dx, dy, dz}]"],
  ],
  forces: [
    ["spring-damper", "${name}:\n\ttype: spring_damper\n\tbody1: ${body}\n\tbody2: ${ground}\n\tpoint1: [${0}, ${0}, ${0}]\n\tpoint2: [${0}, ${0}, ${1}]\n\tstiffness: ${1000}\n\tdamping: ${10}\n\tlength: ${1.0}"],
    ["torsion spring", "${name}:\n\ttype: torsion_spring\n\tbody1: ${body}\n\tbody2: ${ground}\n\tstiffness: ${10}\n\tdamping: ${0.1}"],
    ["bushing", "${name}:\n\ttype: bushing\n\tbody1: ${body}\n\tbody2: ${ground}\n\tlocation: [${0}, ${0}, ${0}]\n\tstiffness: [${1000}, ${1000}, ${1000}]\n\trot_stiffness: [${10}, ${10}, ${10}]"],
    ["constant force", "${name}:\n\ttype: force\n\tbody1: ${body}\n\tvector: [${0}, ${0}, ${-10}]"],
    ["constant torque", "${name}:\n\ttype: torque\n\tbody1: ${body}\n\tvector: [${0}, ${0}, ${1}]"],
    ["expression force", "${name}:\n\ttype: expression_force\n\tbody1: ${body}\n\tdirection: [${0}, ${0}, ${1}]\n\tforce: ${100 * sin(2 * pi * t)}"],
  ],
  sensors: [["sensor", "${name}:\n\tbody1: ${body}\n\tpoint: [${0}, ${0}, ${0}]"]],
  points: [["point", "${name}: [${0}, ${0}, ${0}]"], ["mirrored point", "${name}: {mirror: ${point}}"]],
  parameters: [["parameter", "${name}: ${1.0}"]],
  outputs: [["output", "${name}: ${expression}"]],
};

const STARTER = `# A new model: a pendulum. Change it, ctrl+s saves and simulates.
# Typing shows what can come next (ctrl+space too); under joints: or forces: the templates.
model: my_model
gravity: [0, 0, -9.81]
bodies:
  rod:
    mass: 1.0
    inertia: [0.01, 0.0833, 0.0833]
    position: [0.5, 0, 0]
    shapes:
      - {type: box, size: [1.0, 0.05, 0.05]}
joints:
  hinge:
    type: revolute
    body1: rod
    location: [0, 0, 0]
    orientation: [0.7071, -0.7071, 0, 0]
solver: {type: HHT, end_time: 5.0, step: 1.0e-3, output_step: 1.0e-2}
`;

// ---------- completion ----------
function completions(schema, getModel) {
  return (cx) => {
    const c = context(cx.state.doc, cx.pos);
    if (!c) return null;
    const from = cx.pos - c.word.length;
    if (!cx.explicit && !c.word && c.kind === "key" && !c.dashes) return null;
    const items = outline(cx.state.doc.toString());
    const options = [];
    if (c.kind === "key") {
      const s = schema.at(c.path);
      if (!s) return null;
      const section = c.path.length === 1 ? c.path[0] : null;
      if (schema.isNamedMap(s)) {
        for (const [label, t] of TEMPLATES[section] || [])
          options.push(snippetCompletion(fill(t, items), { label, type: "class", detail: "template", boost: 2 }));
      }
      const req = schema.required(s), type = c.siblings.get("type"), types = schema.values(schema.child(s, "type") || {});
      for (const [k, p] of Object.entries(schema.properties(s))) {
        if (c.siblings.has(k) || !forType(p.description, type, types)) continue;
        const ps = schema.deref(p), d = p.description || schema.description(ps);
        const block = ps.type === "object" || (schema.isNamedMap(ps));
        options.push({ label: k, type: "property", detail: req.has(k) ? "required" : "", boost: req.has(k) ? 1 : 0,
          info: d || undefined, apply: block ? `${k}:\n${" ".repeat(c.indent + 2)}` : `${k}: ` });
      }
      return options.length ? { from, options, validFor: /^[\w-]*$/ } : null;
    }
    // a value
    const parent = schema.at(c.path), s = parent && schema.child(parent, c.key);
    const names = (sec) => namesIn(items, sec);
    const add = (list, type, detail) => { for (const v of list) options.push({ label: String(v), type, detail }); };
    const formula = /=[^#]*$/.test(c.valueText), top = c.path[0];
    if (formula || (top === "parameters" && /=/.test(c.valueText))) {
      add(names("parameters"), "variable", "parameter");
    } else if (EXPRESSION_KEYS.has(c.key) || top === "outputs") {
      add(names("parameters"), "variable", "parameter");
      add(names("measures"), "variable", "measure");
      add(names("splines"), "function", "spline");
      add(names("inputs"), "variable", "input");
      add(["t", "pi"], "constant", "");
      add(["sin", "cos", "tan", "sqrt", "abs", "exp", "log", "min", "max", "atan2", "sign"], "function", "");
    } else if (BODY_KEYS.has(c.key) && top !== "bodies") {
      add(names("bodies"), "variable", "body");
      if (c.key !== "body1") add(["ground"], "keyword", "");
      if (c.key === "in") add(names("frames"), "variable", "frame");
    } else if (POINT_KEYS.has(c.key)) {
      add(names("points"), "variable", "point");
      add(names("frames"), "variable", "frame");
    } else if (c.key === "orientation") {
      add(names("frames"), "variable", "frame");
    } else if (c.key === "measure") {
      add(names("measures"), "variable", "measure");
    } else if (c.key === "channel" || c.key === "x_channel") {
      const m = getModel();
      if (m && m.channels) add(m.channels, "variable", "channel");
    }
    if (s) for (const v of schema.values(s)) options.push({ label: String(v), type: "enum", info: schema.description(s) || undefined, boost: 1 });
    if (!options.length) return null;
    return { from, options, validFor: /^[\w.]*$/ };
  };
}

const FILLER = new Set(["only", "axis", "z", "with", "vector", "and", "plane", "or"]);
function forType(description, type, types) {
  // false when the description of a key names the types it is for ("bushing: ...", "generic
  // only: ...") and the element's type is not one of them
  if (!type || !description || !types.includes(type)) return true;
  const words = description.split(":")[0].split(/[\s,/()]+/).filter(Boolean);
  const named = words.filter((w) => types.includes(w));
  if (!named.length || words.some((w) => !types.includes(w) && !FILLER.has(w))) return true;
  return named.includes(type);
}

function fill(template, items) {
  // a template with the first body of the model where it needs one; the fields numbered (tab
  // goes from one to the next; fields of the same name would be one field)
  const body = namesIn(items, "bodies")[0] || "body", point = namesIn(items, "points")[0] || "point";
  let k = 0;
  return template.replace("${body}", "${" + body + "}").replace("${point}", "${" + point + "}")
    .replace(/\$\{([^}]*)\}/g, (_, text) => `\${${++k}:${text}}`);
}

// ---------- hover: what a key means ----------
function hover(schema) {
  return hoverTooltip((view, pos) => {
    const line = view.state.doc.lineAt(pos), m = KEY_LINE.exec(line.text);
    if (!m) return null;
    const start = line.from + keyIndent(m), end = start + m[3].length;
    if (pos < start || pos > end) return null;
    const c = context(view.state.doc, start);
    if (!c) return null;
    const parent = schema.at(c.path), s = parent && schema.child(parent, m[3]);
    const text = s && schema.description(s);
    if (!text) return null;
    const values = schema.values(s);
    return { pos: start, end, above: true, create: () => {
      const dom = document.createElement("div");
      dom.className = "cm-key-help";
      dom.textContent = text;
      if (values.length) {
        const v = document.createElement("div");
        v.className = "values";
        v.textContent = values.join(" · ");
        dom.appendChild(v);
      }
      return { dom };
    } };
  });
}

// ---------- indentation: that of the line above, one more after "key:" ----------
const indentation = indentService.of((cx, pos) => {
  // the lines above pos (cx.lineAt: as if the line were broken at pos, when Enter is pressed)
  let l = cx.lineAt(pos, -1);
  if (l.from === pos) { if (pos === 0) return 0; l = cx.lineAt(pos - 1, -1); }
  for (;; l = cx.lineAt(l.from - 1, -1)) {
    const text = l.text.replace(/\s+#.*$/, "").replace(/^\s*#.*$/, "");
    if (!text.trim()) { if (l.from === 0) return 0; continue; }
    const m = /^(\s*)((?:- +)*)/.exec(text), base = m[0].length;  // after "- ": the column of the item's keys
    return /:\s*$/.test(text) ? base + 2 : base;
  }
});

// the checks again (new results, an error of the rebuild): the text has not changed
const relint = StateEffect.define();

// ---------- the lines of the selected element ----------
const markElement = StateEffect.define();
const elementLines = StateField.define({
  create: () => Decoration.none,
  update(deco, tr) {
    deco = deco.map(tr.changes);
    for (const e of tr.effects) if (e.is(markElement)) {
      const { from, to } = e.value || {};
      if (!e.value) { deco = Decoration.none; continue; }
      const marks = [];
      for (let n = from; n <= to; n++) marks.push(Decoration.line({ class: "cm-element" }).range(tr.state.doc.line(n).from));
      deco = Decoration.set(marks);
    }
    return deco;
  },
  provide: (f) => EditorView.decorations.from(f),
});

// numbers and formulas: the content of YAML is one token for the parser
const valueMarks = new MatchDecorator({
  regexp: /(=[^,\]}\n#]+)|(?<![\w.])(-?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)(?![\w])/g,
  decoration: (m) => Decoration.mark({ class: m[1] ? "cm-formula" : "cm-number" }),
});
const values = ViewPlugin.fromClass(class {
  constructor(view) { this.decorations = valueMarks.createDeco(view); }
  update(u) { this.decorations = valueMarks.updateDeco(u, this.decorations); }
}, { decorations: (v) => v.decorations });

const style = HighlightStyle.define([
  { tag: tags.definition(tags.propertyName), color: "var(--accent)" },
  { tag: tags.lineComment, color: "var(--muted)", fontStyle: "italic" },
  { tag: tags.string, color: "var(--ok)" },
  { tag: [tags.separator, tags.squareBracket, tags.brace, tags.punctuation], color: "var(--muted)" },
  { tag: tags.meta, color: "var(--muted)" },
]);

const theme = EditorView.theme({
  "&": { height: "100%", fontSize: "12.5px", backgroundColor: "var(--panel)", color: "var(--text)" },
  "&.cm-focused": { outline: "none" },
  ".cm-scroller": { fontFamily: "ui-monospace, Consolas, monospace", lineHeight: "1.5" },
  ".cm-content": { caretColor: "var(--text)" },
  ".cm-cursor": { borderLeftColor: "var(--text)" },
  ".cm-gutters": { backgroundColor: "var(--panel)", color: "var(--muted)", border: "none" },
  ".cm-activeLine": { backgroundColor: "color-mix(in srgb, var(--accent) 6%, transparent)" },
  ".cm-activeLineGutter": { backgroundColor: "color-mix(in srgb, var(--accent) 10%, transparent)" },
  "&.cm-focused .cm-selectionBackground, .cm-selectionBackground, .cm-content ::selection":
    { backgroundColor: "color-mix(in srgb, var(--accent) 25%, transparent) !important" },
  ".cm-element": { boxShadow: "inset 3px 0 0 var(--accent)", backgroundColor: "color-mix(in srgb, var(--accent) 5%, transparent)" },
  ".cm-number": { color: "var(--pending)" },
  ".cm-formula": { color: "var(--running)" },
  ".cm-tooltip": { backgroundColor: "var(--panel)", color: "var(--text)", border: "1px solid var(--line)", borderRadius: "6px" },
  ".cm-tooltip-autocomplete ul li[aria-selected]": { backgroundColor: "var(--accent)", color: "#fff" },
  ".cm-tooltip.cm-completionInfo": { maxWidth: "360px", padding: "6px 8px", fontFamily: "system-ui, sans-serif" },
  ".cm-completionDetail": { color: "var(--muted)", fontStyle: "normal", marginLeft: "8px" },
  ".cm-key-help": { maxWidth: "380px", padding: "6px 8px", fontFamily: "system-ui, sans-serif", fontSize: "12px" },
  ".cm-key-help .values": { marginTop: "4px", color: "var(--accent)", fontFamily: "ui-monospace, Consolas, monospace" },
  ".cm-diagnostic": { fontFamily: "system-ui, sans-serif", fontSize: "12px" },
  ".cm-foldGutter span": { color: "var(--muted)" },
});

// ---------- the editor ----------
export class ModelEditor {
  // opts: {box, head, banner, post(url, body), getJSON(url), onSelect(name), findings() -> [{severity,
  //        message, elements}], model() -> {channels}, onState(text)}
  constructor(opts, schemaRoot) {
    this.o = opts;
    this.schema = new Schema(schemaRoot);
    this.stamp = null;
    this.saved = "";
    this.editable = true;
    this.revealing = false;
    this.selected = null;
    this.serverError = null;
    this.readOnly = new Compartment();
    const lintSource = async (view) => this.lint(view);
    this.view = new EditorView({
      parent: opts.box,
      state: EditorState.create({
        doc: "",
        extensions: [
          lineNumbers(), foldGutter(), lintGutter(), highlightActiveLineGutter(), history(), drawSelection(),
          indentOnInput(), bracketMatching(), closeBrackets(), highlightActiveLine(), highlightSelectionMatches(),
          indentUnit.of("  "), EditorState.tabSize.of(2), yaml(), indentation, syntaxHighlighting(style), values, theme,
          elementLines, placeholder("model: ..."), EditorView.lineWrapping,
          autocompletion({ override: [completions(this.schema, () => this.o.model())], icons: false, activateOnTyping: true }),
          linter(lintSource, { delay: 500, needsRefresh: (u) => u.transactions.some((t) => t.effects.some((e) => e.is(relint))) }), hover(this.schema),
          keymap.of([
            { key: "Mod-s", preventDefault: true, run: () => { this.save(); return true; } },
            ...closeBracketsKeymap, ...completionKeymap, ...searchKeymap, ...foldKeymap, ...historyKeymap,
            ...defaultKeymap, indentWithTab,
          ]),
          EditorView.updateListener.of((u) => this.updated(u)),
          this.readOnly.of([]),
        ],
      }),
    });
  }

  get dirty() { return this.view.state.doc.toString() !== this.saved; }

  async load(keepCursor = true) {
    const src = await this.o.getJSON("api/source");
    this.editable = src.editable;
    this.name = src.name;
    this.setText(src.text, src.stamp, keepCursor);
    if (!src.text.trim() && this.editable) {  // a new model: a pendulum to start from
      this.view.dispatch({ changes: { from: 0, to: this.view.state.doc.length, insert: STARTER } });
    }
    this.status();
  }

  setText(text, stamp, keepCursor) {
    const line = keepCursor ? this.view.state.doc.lineAt(this.view.state.selection.main.head).number : 1;
    this.saved = text;
    this.stamp = stamp;
    this.view.dispatch({
      changes: { from: 0, to: this.view.state.doc.length, insert: text },
      selection: { anchor: lineStart(text, line) },
      effects: this.readOnly.reconfigure(this.editable ? [] : [EditorState.readOnly.of(true)]),
    });
    this.o.banner.hidden = true;
  }

  async save() {
    if (!this.editable) {
      this.status("legacy .fds files are read-only here: freedyn2 convert makes a YAML model file");
      return;
    }
    const text = this.view.state.doc.toString();
    const r = await this.o.post("api/source", { text, stamp: this.stamp });
    if (r.status === 409) { this.conflict(r.body); return; }
    if (!r.ok) { this.status(r.body.error || "not saved"); return; }
    this.saved = text;
    this.stamp = r.body.stamp;
    this.serverError = null;
    this.status("saved · simulating …");
  }

  conflict(current) {
    // the file changed meanwhile: take that one, or write this one over it
    const b = this.o.banner;
    b.hidden = false;
    b.innerHTML = 'the file was changed meanwhile (another editor or the assistant): <span class="link" data-a="load">load it</span> · <span class="link" data-a="keep">keep mine and save</span>';
    b.querySelector('[data-a="load"]').onclick = () => { this.setText(current.text, current.stamp, true); this.status(); };
    b.querySelector('[data-a="keep"]').onclick = () => { this.stamp = current.stamp; b.hidden = true; this.save(); };
  }

  sync(stamp) {
    // the stamp of the model file from the viewer's poll: reload a file changed by someone else
    if (!stamp || stamp === this.stamp || !this.stamp) return;
    if (!this.dirty) { this.load(true).then(() => this.status("reloaded: the file was changed outside")); return; }
    this.o.getJSON("api/source").then((cur) => { if (cur.stamp !== this.stamp) this.conflict(cur); });
  }

  setServerError(text) {
    // the error of the last rebuild (after a save): shown in the editor, at a line when it names one
    this.serverError = text || null;
    if (!this.dirty) this.status(text ? "saved · the model has an error (marked)" : "saved");
    this.view.dispatch({ effects: relint.of(null) });
  }

  status(text) {
    const s = this.o.head;
    s.textContent = text || (!this.editable ? "read-only" : this.dirty ? "● not saved · ctrl+s saves and simulates" : "saved");
    s.classList.toggle("dirty", this.editable && this.dirty && !text);
  }

  updated(u) {
    if (u.docChanged) this.status();
    if (u.selectionSet && !this.revealing && u.view.hasFocus) {
      clearTimeout(this.selTimer);
      this.selTimer = setTimeout(() => this.cursorElement(), 120);
    }
  }

  elementAt(lineNumber) {
    const items = outline(this.view.state.doc.toString());
    return items.find((x) => x.line <= lineNumber && lineNumber <= x.end && SECTIONS_OF_ELEMENTS.includes(x.section)) || null;
  }

  cursorElement() {
    const n = this.view.state.doc.lineAt(this.view.state.selection.main.head).number;
    const e = this.elementAt(n);
    const name = e ? e.name : null;
    if (name === this.selected) return;
    this.selected = name;
    this.mark(e);
    this.o.onSelect(name);
  }

  mark(e) {
    this.view.dispatch({ effects: markElement.of(e ? { from: e.line, to: Math.min(e.end, this.view.state.doc.lines) } : null) });
  }

  reveal(name, focus = false) {
    // show an element selected elsewhere (3D view, inspector, checks)
    const e = name ? outline(this.view.state.doc.toString()).find((x) => x.name === name && SECTIONS_OF_ELEMENTS.includes(x.section)) : null;
    this.selected = name;
    this.mark(e);
    if (!e) return;
    const cur = this.view.state.doc.lineAt(this.view.state.selection.main.head).number;
    this.revealing = true;
    const pos = this.view.state.doc.line(e.line).from;
    this.view.dispatch({
      selection: cur >= e.line && cur <= e.end ? undefined : { anchor: pos + this.view.state.doc.line(e.line).text.search(/\S/) },
      effects: EditorView.scrollIntoView(pos, { y: "start", yMargin: 40 }),
    });
    this.revealing = false;
    if (focus) this.view.focus();
  }

  gotoLine(n) {
    n = Math.max(1, Math.min(n, this.view.state.doc.lines));
    const l = this.view.state.doc.line(n);
    this.view.dispatch({ selection: { anchor: l.from + Math.max(0, l.text.search(/\S/)) },
      effects: EditorView.scrollIntoView(l.from, { y: "center" }) });
    this.view.focus();
  }

  async lint(view) {
    const doc = view.state.doc, text = doc.toString();
    const at = (n) => { const l = doc.line(Math.max(1, Math.min(n, doc.lines))); return { from: l.from + Math.max(0, l.text.search(/\S/)), to: l.to }; };
    let diags = [];
    try {
      const r = await this.o.post("api/lint", { text });
      if (view.state.doc.toString() !== text) return [];  // typed on meanwhile: the next run checks
      diags = (r.body.diagnostics || []).map((d) => ({ ...at(d.line), severity: d.severity, message: d.message, source: "model" }));
    } catch (e) { return []; }
    if (!diags.length && this.serverError) {
      diags.push({ ...at(1), severity: "error", message: this.serverError, source: "simulation" });
    }
    if (!diags.length) {  // the checks of the last results, at the lines of their elements
      const items = outline(text);
      for (const f of this.o.findings()) {
        const e = items.find((x) => f.elements.includes(x.name));
        if (e) diags.push({ ...at(e.line), severity: f.severity === "error" ? "error" : f.severity === "warning" ? "warning" : "info",
          message: f.message, source: "checks" });
      }
    }
    return diags;
  }
}

function lineStart(text, line) {
  let pos = 0;
  for (let n = 1; n < line; n++) { const k = text.indexOf("\n", pos); if (k < 0) return pos; pos = k + 1; }
  return pos;
}

export const _test = { outline, context, Schema, TEMPLATES };
