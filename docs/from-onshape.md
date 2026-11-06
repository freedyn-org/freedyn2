# Coming from Onshape

`freedyn2 import-onshape` reads an Onshape assembly with the Onshape REST API and writes it as
a [model file](reference/model-file.md): bodies with Onshape's mass properties, joints from
the mates, the parts' geometry for the [viewer](viewer.md). Springs, forces, drivers and solver
settings are added in the YAML afterwards.

!!! note "Prototype"
    The importer is new. Revolute, slider, cylindrical, ball, planar, parallel and pin slot
    mates are imported; the others are listed in the file, not imported.

## API keys

Create a pair of API keys at [dev-portal.onshape.com/keys](https://dev-portal.onshape.com/keys)
(read access is enough) and set them as environment variables:

```sh
export ONSHAPE_ACCESS_KEY=...   # Windows: set ONSHAPE_ACCESS_KEY=...
export ONSHAPE_SECRET_KEY=...
```

The keys see every document your Onshape account sees. Keep them out of model files and
repositories.

## Importing

Open the assembly in the browser and copy its address:

```sh
freedyn2 import-onshape "https://cad.onshape.com/documents/<document>/w/<workspace>/e/<element>"
freedyn2 import-onshape "<address>" -o models/crank.yaml --force
```

| option | |
|---|---|
| `-o FILE` | the YAML file, or a directory for `<assembly name>.yaml` in it (default: the current directory) |
| `--no-meshes` | no STL files of the parts (the viewer then draws the inertia boxes) |
| `--force` | overwrite an existing file (without it, `import-onshape` stops) |
| `--record DIR` | save the API responses in `DIR`, also when the import fails (for bug reports; they contain names and geometry of the document, no API keys) |

A version (`/v/...`) or a configuration (`?configuration=...`) in the address is imported as
it is. Enterprise addresses (`https://<company>.onshape.com/...`) work the same way.

It writes three things next to each other:

- `crank.yaml`, the model. Comments name the Onshape parts of every body and list what was not
  imported.
- `crank_meshes/`, an STL file per part (in meters), placed at the part in the assembly.
- `crank.onshape.json`, the Onshape reference of every part (document, microversion, element,
  part id, configuration), to fetch the exact geometry again later.

## What becomes what

| Onshape | freedyn2 |
|---|---|
| parts connected by Fastened mates or a mate group | one body: masses, centers of mass and inertia tensors added up, its principal axes are the body frame |
| fixed parts (and what is fastened to them) | the ground (shapes only) |
| Revolute | `revolute` |
| Slider | `prismatic` |
| Cylindrical | `cylindrical` |
| Ball | `spherical` |
| Planar | `generic`, `fix: [dz, ax, ay]` |
| Parallel | `generic`, `fix: [ax, ay]` |
| Pin slot | `generic`, `fix: [dy, dz, ax, ay]` in the frame of the slot |
| Tangent, Width, mate relations (gear, rack and pinion, screw, linear) | not imported, listed in the file and as warnings |

- The joint frame is the mate connector of the first mated entity (joint axis z, as in
  Onshape) at the assembly's current position, which is t = 0 of the simulation.
- Mass properties are Onshape's, computed on the exact geometry. A body none of whose parts
  has a material is an error naming the parts: assign a material in Onshape, or fasten the part
  to one with mass. Massless parts in a body with mass (screws, standard content) are fine.
- Sub-assemblies are flattened: their mates become joints between their parts.
- The geometry is for the viewer only; the simulation does not use it.
