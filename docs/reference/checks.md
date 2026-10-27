# Checks and references

A model has to be checkable faster than it can be built. `freedyn2 check`, the viewer's
*Checks* panel and the assistant's `validate` and `check` tools report the same findings:

```sh
freedyn2 check model.yaml             # the model alone
freedyn2 check model.yaml --run       # and its results
freedyn2 check model.yaml result.csv  # and these results
```

The exit status is 1 if there is an error or a failed reference – usable in CI.

## Findings

| | |
|---|---|
| **errors** | inertia no real body has (I1 + I2 < I3), a spring whose points coincide, non-finite results |
| **likely slips** | gravity, lengths or masses not in SI units; a joint far from both its bodies; more constraints than degrees of freedom (redundant joints); a tire deep in the road or in the air at the start; drift of the orientation |
| **good to know** | degrees of freedom and total mass; bodies not connected to the ground or not at rest at the start; parameters no expression uses |

## References

The `checks` section holds what the results must agree with – from the weight list, a
measurement, a test rig:

```yaml
checks:
  mass: {total: 1430, tolerance: 5}                        # [kg], from the weight list
  values:
    - {name: ride height, channel: chassis1.R_z, time: 0.0, value: 0.536, tolerance: 0.002}
  curves:
    - {name: toe fl, channel: toe_fl, x_channel: travel_fl, file: kc_rig.csv, x: travel, y: toe, tolerance: 0.001}
```

- `values`: a channel at a time.
- `curves`: a channel over time, or over another channel (`x_channel`) – e.g. toe over wheel
  travel of a kinematics & compliance rig – against a CSV file. Reported: the largest
  deviation and where.

Every check is run with every simulation in the viewer; failed references are red in the
*Checks* panel, and a click plots the curve with its reference.

<!-- keys: checks -->
