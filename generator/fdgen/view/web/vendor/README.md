# three.js (vendored)

three.js r170 (npm package `three@0.170.0`), MIT license (LICENSE-three.js), unmodified:
`build/three.module.min.js`, `examples/jsm/controls/OrbitControls.js`,
`examples/jsm/loaders/STLLoader.js`, `examples/jsm/loaders/OBJLoader.js`.
Vendored so that the viewer works offline.

CodeMirror 6 (`codemirror.min.js`, MIT license, LICENSE-codemirror): the editor of the model file
(fdview/editor.js), one ES module bundled with esbuild from the packages in
`tools/codemirror/package.json`: `cd tools/codemirror && npm install && npm run build`
(`entry.js` lists what the editor uses).
