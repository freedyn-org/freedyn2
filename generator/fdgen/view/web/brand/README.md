# brand assets of the pages (copies)

The brand of freedyn2 lives in `brand/` at the top of the repository. The files here are what the
app and the viewer need of it, copied by `python tools/sync_brand.py` so that the package is
self-contained and works offline: `tokens.css`, `favicon.svg`, `lockup-small-{light,dark}.svg`
(do not edit them here; `tests/python/test_app.py` checks that they equal `brand/`). `fonts.css`
belongs to this folder only.

`fonts/`: JetBrains Mono and Inter from the npm packages `@fontsource/jetbrains-mono@5.3.0` and
`@fontsource/inter@5.3.0` (`python tools/sync_brand.py --fonts`, SHA-512 checked), latin subset,
weights 400 and 600, woff2, unmodified. SIL Open Font License 1.1: `fonts/OFL-JetBrainsMono.txt`,
`fonts/OFL-Inter.txt`.
