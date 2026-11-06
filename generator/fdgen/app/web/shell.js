// The app shell around a viewer page served by the freedyn2 app (fdgen.app.server): a small
// notice when a new release is available and the errors of the app. The viewer is unchanged.
(function () {
  "use strict";
  const box = document.createElement("div");
  box.id = "fd-app-notice";
  box.style.cssText = "display:none;gap:8px;align-items:center;padding:1px 4px 1px 10px;border-radius:0;font-size:12px;" +
    "border:1px solid var(--accent);color:var(--text);white-space:nowrap;overflow:hidden;flex:0 1 auto;min-width:0";
  const header = document.querySelector("header"), file = document.getElementById("status");
  if (header && file) header.insertBefore(box, file);  // in the header, before the file name
  else {
    box.style.cssText += ";position:fixed;left:50%;top:6px;transform:translateX(-50%);z-index:50;background:var(--panel)";
    document.body.appendChild(box);
  }
  const visible = () => box.style.display !== "none";
  const setVisible = (on) => { box.style.display = on ? "flex" : "none"; };
  let hidden = "";
  try { hidden = sessionStorage.getItem("fd.update.hide") || ""; } catch (e) { /* no storage */ }
  let errorsSeen = 0;

  function show(html, error) {
    box.innerHTML = html + '<button style="border:0;background:none;color:var(--muted);font-size:14px;padding:0 4px">×</button>';
    box.style.borderColor = error ? "var(--error)" : "var(--accent)";
    box.style.color = error ? "var(--error)" : "var(--text)";
    box.querySelector("button").onclick = () => {
      setVisible(false);
      if (!error && status && status.update) {
        hidden = status.update.latest;
        try { sessionStorage.setItem("fd.update.hide", hidden); } catch (e) { /* no storage */ }
      }
    };
    setVisible(true);
  }
  const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);
  let status = null;
  async function poll() {
    try {
      status = await (await fetch("/api/app/status", { cache: "no-store" })).json();
      if (status.errors.length > errorsSeen) {
        errorsSeen = status.errors.length;
        show(esc(status.errors[status.errors.length - 1].message), true);
      } else if (status.errors.length < errorsSeen) errorsSeen = status.errors.length;
      else if (!visible() && status.update && status.update.newer && hidden !== status.update.latest) {
        const u = status.update;
        show(`freedyn2 ${esc(u.latest)} is available · <a href="${esc(u.url || status.releases)}" target="_blank" rel="noopener" style="color:var(--accent)">release page</a>`, false);
      }
    } catch (e) { /* the app ended */ }
    setTimeout(poll, 15000);
  }
  setTimeout(poll, 1500);
})();
