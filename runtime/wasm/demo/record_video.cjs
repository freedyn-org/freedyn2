// Record the demo video: the browser demo with captions, a model zip dropped onto the page, the
// number of network requests while it is translated and simulated (0), a changed value.
//
//   python tools/serve_web_demo.py build-web --no-browser
//   node runtime/wasm/demo/record_video.cjs out_dir model.zip     # needs: npm install playwright
//   ffmpeg -i out_dir/video/*.webm -c:v libx264 -pix_fmt yuv420p demo.mp4
//
// DEMO_URL: the page (default http://localhost:8000/); CAPTION_MODEL: the caption for the zip.
// The model should have a body "slider" and "damping: 4" for the last part (as slider_arm).
const { chromium } = require('playwright');
const S = process.argv[2], ZIP = process.argv[3];
(async () => {
  const b = await chromium.launch();
  const ctx = await b.newContext({ viewport: { width: 1280, height: 720 }, recordVideo: { dir: S + '/video', size: { width: 1280, height: 720 } } });
  const p = await ctx.newPage();
  let net = 0, counting = false;
  p.on('request', (r) => { if (counting && /^https?:/.test(r.url())) net++; });
  const caption = (text) => p.evaluate((t) => {
    let c = document.getElementById('fdcap');
    if (!c) { c = document.createElement('div'); c.id = 'fdcap';
      c.style.cssText = 'position:fixed;left:16px;bottom:16px;width:402px;min-height:150px;padding:14px 16px;box-sizing:border-box;' +
        'background:#0B0B0C;color:#EDEDE8;font:500 16px/1.5 "JetBrains Mono",monospace;z-index:99;transition:opacity .3s;' +
        'display:flex;align-items:center';
      document.body.appendChild(c); }
    c.style.opacity = t ? 1 : 0; if (t) c.innerHTML = t;
  }, text);
  const done = async () => { await p.waitForTimeout(150); await p.waitForFunction(() => /^done/.test(document.getElementById('status').textContent) || document.getElementById('status').className === 'error', null, { timeout: 120000 }); };
  const viewer = async (fn, arg) => { const fr = await (await p.$('iframe')).contentFrame(); await fr.waitForFunction(() => window.fdViewer && window.fdViewer.model); return fr.evaluate(fn, arg); };
  const play = async (from, to, ms) => { const steps = Math.round(ms / 40); for (let i = 0; i <= steps; i++) { await viewer((t) => window.fdViewer.setTime(t), from + (to - from) * i / steps); await p.waitForTimeout(40); } };
  const cam = () => viewer(() => { const v = window.fdViewer; v.camera.position.set(0.15, -1.9, 0.25); v.controls.target.set(0.15, 0, -0.25); v.controls.update(); });

  await p.goto(process.env.DEMO_URL || 'http://localhost:8000/');
  await caption('freedyn2 for Onshape:<br>multibody dynamics, computed in the browser');
  await done(); await p.waitForTimeout(2500);
  await caption(process.env.CAPTION_MODEL || 'A CAD assembly, exported as YAML + STL parts in one zip, dropped onto the page');
  await p.waitForTimeout(2500);
  await p.setInputFiles('#file', ZIP); await done();
  await viewer(() => window.fdViewer.addChannel('slider.R_x')); await cam();
  const tm = await p.textContent('#timings');
  const sim = /simulation \(wasm\)\s+(\d+ ms)/.exec(tm)[1];
  const steps = /done: (\d+) steps/.exec(await p.textContent('#status'))[1];
  await caption(`${steps} time steps solved in ${sim} by the freedyn2 solver, compiled to WebAssembly`);
  await play(0, 3, 6000);
  await caption('Change a value in the model (damping 4 → 40), simulate again');
  const y = await p.inputValue('#yaml');
  await p.evaluate(() => { const t = document.getElementById('yaml'); const i = t.value.indexOf('damping: 4'); t.focus(); t.setSelectionRange(i, i + 10); t.scrollTop = t.scrollHeight; });
  await p.waitForTimeout(1500);
  await p.fill('#yaml', y.replace('damping: 4', 'damping: 40'));
  await p.evaluate(() => { const t = document.getElementById('yaml'); const i = t.value.indexOf('damping: 40'); t.setSelectionRange(i, i + 11); t.scrollTop = t.scrollHeight; });
  await p.waitForTimeout(1200);
  net = 0; counting = true;
  await p.click('#run'); await done();
  counting = false;
  await viewer(() => window.fdViewer.addChannel('slider.R_x')); await cam();
  await caption(`Network requests while translating and simulating: ${net}.<br>The model never leaves the browser. No server, no install.`);
  await play(0, 3, 6000);
  await caption('Free. Open source (Apache-2.0).<br>Published and maintained by SevenSim.');
  await p.waitForTimeout(4500);
  console.log('network requests during simulate:', net, '| wasm time:', sim);
  await ctx.close(); await b.close();
})();
