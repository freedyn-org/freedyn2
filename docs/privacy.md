# Privacy policy

freedyn2 does not collect, store or transmit personal data. Models, results, notes and
settings stay on your computer (in the folders you choose and in `%APPDATA%\freedyn2` and
`%LOCALAPPDATA%\freedyn2` on Windows, `~/.config`, `~/.local/share` and `~/.cache` below
`freedyn2` on Linux).

The program connects to the internet in one case only:

- **Update check.** The app asks GitHub at most once a day whether a newer release exists
  (`api.github.com`, the public release information of this repository). The request carries
  no identifier of you or your installation; GitHub sees your IP address, as with any web
  request ([GitHub's privacy statement](https://docs.github.com/en/site-policy/privacy-policies/github-general-privacy-statement)).
  Turn it off with `freedyn2 config update_check false` or the environment variable
  `FREEDYN2_NO_UPDATE_CHECK=1`.

The local server of the viewer listens on `127.0.0.1` only and is not reachable from other
computers.

**Assistants.** `freedyn2 mcp` serves an assistant you run yourself (for example Claude
Desktop). FreeDyn sends nothing to the assistant's provider; what your assistant sends is
governed by that provider's terms.

**This website** is hosted on GitHub Pages; GitHub may log visits
([GitHub's privacy statement](https://docs.github.com/en/site-policy/privacy-policies/github-general-privacy-statement)).
Formulas are rendered with MathJax, loaded from the jsDelivr CDN. The site uses no cookies,
no analytics and no external fonts.

Questions: open an issue at [github.com/freedyn-org/freedyn2](https://github.com/freedyn-org/freedyn2/issues).
