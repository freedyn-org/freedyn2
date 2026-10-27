# Tires and roads

## The tire element

`type: tire` puts a tire on `body1`, a wheel turning about its **y axis** (orient the wheel
body accordingly). The contact is a point on the road below the wheel center, found every
step on the road's tangent plane; the wheel may lift off.

```yaml
forces:
  tire_fl:
    type: tire
    body1: wheel_fl
    radius: 0.31                  # unloaded radius
    tire: {kz: 250000, cz: 300}   # override defaults; see below
```

The built-in tire is a passenger car tire (roughly 205/55 R16) with vertical spring and
damper, Magic Formula longitudinal and lateral forces with load dependence and combined slip,
aligning moment and rolling resistance. Its coefficients can be overridden under `tire:`:

`r0 kz cz Fz0 Cx pDx1 pDx2 pEx1 pKx1 pKx3 Cy pDy1 pDy2 pEy1 pKy1 pKy2 rBx1 rBy1 t0 qBz qsy1 v_eps w_eps eps_z`

Other tire models (complete Magic Formula data sets from `.tir` files, for instance) come as
[extensions](../extensions.md): they add their keys to the tire and run on the same roads.

Channels: `.force_x/y/z` (longitudinal, lateral, vertical, in the contact frame), `.slip0`
(longitudinal slip), `.slip1` (slip angle), `.friction`, `.penetration`, `.contact_x/y/z`,
`.road_height`.

## Roads

The `road` section is shared by all tires of a model.

=== "Analytic road"

    A sum of up to 16 harmonic waves, a slope and a friction coefficient, with an optional
    patch of other friction (split-µ):

    ```yaml
    road:
      harmonics:
        - {amplitude: 0.01, wavelength: 20, heading: 0.0}
        - {amplitude: 0.003, wavelength: 2.5, heading: 0.4, phase: 1.2}
      slope: [0.0, 0.005]          # dh/dx, dh/dy
      friction: 1.0
      patch: {friction: 0.3, x: [60, 100], y: [0, 5], edge: 0.5}
    ```

=== "OpenCRG file"

    ```yaml
    road:
      crg: track.crg               # ASAM OpenCRG height grid
      crg_friction: track_mu.crg   # optional: a grid of friction coefficients
      friction: 1.0                # else a constant friction
      position: [0, 0, 0]          # placement of the CRG frame
      heading: 0.0
    ```

    Read and evaluated with the official ASAM OpenCRG library: all data formats, curved
    reference lines, slope and banking, NaN values, border modes, modifiers, include files. A
    2 km road on a 2 cm grid costs as much per step as a small one.
    `freedyn2_road road.crg < points` prints height, gradient and friction at points, to check a
    road and its placement.

<!-- keys: road -->
