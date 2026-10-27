# Working with an assistant

FreeDyn 2.0 comes with an [MCP](https://modelcontextprotocol.io) server: through it an AI
assistant – Claude Code, Claude Desktop or any other MCP client – reads the conventions,
writes model files, validates, simulates, reads and compares results, runs variant studies,
opens the viewer for you and reads the notes you leave there. You check and decide.

## Connect it

=== "Claude Code"

    ```sh
    claude mcp add freedyn2 -- uvx freedyn2 mcp      # or: -- freedyn2 mcp  if installed
    ```

=== "Claude Desktop"

    In `claude_desktop_config.json`:

    ```json
    {"mcpServers": {"freedyn2": {"command": "uvx", "args": ["freedyn2", "mcp"]}}}
    ```

=== "Other clients"

    Start `freedyn2 mcp` (stdio). Paths in tool calls are absolute or relative to the
    directory the server was started in.

`uvx` runs FreeDyn without installing it first; with an installed FreeDyn use the
`freedyn2` command instead.

## How a session goes

> *"Build a quarter car: 400 kg body, 40 kg wheel, a 25 kN/m spring, a 1800 N s/m damper and a
> 200 kN/m tire, driving over a 4 cm bump at 10 m/s. Make spring, damper and tire parameters.
> Then show it to me."*

The assistant reads the `guide` (conventions, element keys, an example), writes the YAML file,
calls `validate` – which reports degrees of freedom, mass and findings – fixes what the checks
find, `simulate`s, `check`s the results and opens the `view`er for you. From then on every
save of the file by the assistant reloads your viewer.

> *"Which damper gives the smallest tire load variation without the body acceleration going
> over 15 m/s²? Sweep it from 500 to 5000."*

`sweep` simulates the values in parallel and returns a table of key figures; the assistant
can narrow the range and sweep again, and show you the fan in the viewer.

> *(a note in the viewer, with a picture)* *"The wheel lifts off at the bump – is the tire
> force allowed to go negative?"*

The assistant reads the open `notes` (and the picture with `note_image`), changes the model –
here: a tire force that cannot pull – and `answer_note`s; you see the answer under your note.

## The tools

| tool | what it does |
|---|---|
| `guide`, `schema` | the conventions (global input at t = 0, quaternions, SI units, …), element keys, an example; the JSON Schema |
| `validate` | validate a model file and check it: degrees of freedom, mass, findings |
| `simulate` | build and run; a run id per run, the solver log, the checks of the results |
| `check` | plausibility checks of a model and its results, the references of its `checks` section |
| `channels`, `results` | the channel names of a run; statistics and a thinned table of channels |
| `compare` | two runs (before and after a change), the largest differences first |
| `sweep` | a variant study: one number at several values, in parallel; a key figure of channels per value |
| `modes`, `locus` | eigenfrequencies and damping about the state of a run; the same for several values of one number |
| `view` | the viewer in your browser, reloading on every save |
| `screenshot` | a picture of the viewer, for the assistant itself (needs `pip install playwright` and `playwright install chromium`) |
| `notes`, `note_image`, `answer_note` | what you wrote in the viewer, with its context and picture; the answer |

## Good to know

- **The model file is the truth.** Values you tune with sliders live in the viewer until they
  are written into the file; ask the assistant to take them over (*→ assistant* in Tune sends
  them as a note).
- **Parameters make models tunable.** Ask for the values you will want to vary as
  `parameters` and for the elements to use them as formulas (`stiffness: =k_spring`); they get
  sliders, and `sweep` and `locus` work on them.
- **Checks are cheap; use them.** A model the assistant wrote has to be checkable faster than
  it was written: the findings, the references in the `checks` section (total mass from the
  weight list, measured curves) and a look at the viewer are what tells you whether to trust
  it.
- Notes are files next to the model (`<model>.notes/`), so an assistant working in the
  repository without MCP finds them too.
