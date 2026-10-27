"""The OpenCRG test track of examples/car_on_crg.yaml (fdgen.crg.write_crg):

  python examples/data/make_test_track.py      # writes examples/data/test_track.crg

A straight road, 220 m long and 8 m wide, 10 cm grid, along global x from the origin
(v: lateral, left positive). From the start:

  30 m      a speed bump over the full width, 6 cm high, 3 m long
  60-100 m  washboard, 8 mm amplitude, 1.2 m wavelength
  130 m     a pothole in the left wheel track only (v = 0.8), 5 cm deep, 0.8 m long
  160-200 m banking, 4 % falling to the right (ramped in and out over 10 m)
"""

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "generator"))

from fdgen.crg import write_crg  # noqa: E402

DU, LENGTH = 0.1, 220.0
V = np.round(np.arange(-4.0, 4.0 + 1e-9, 0.1), 6)


def bump(u, start, length, height):
    s = np.clip((u - start) / length, 0.0, 1.0)
    return 0.5 * height * (1 - np.cos(2 * np.pi * s))


def ramp(u, a, b, edge):
    """1 between a and b, cosine ramps of length edge outside."""
    up = np.clip((u - (a - edge)) / edge, 0, 1)
    down = np.clip(((b + edge) - u) / edge, 0, 1)
    return 0.5 * (1 - np.cos(np.pi * np.minimum(up, down)))


def track():
    u = np.arange(0.0, LENGTH + 1e-9, DU)
    U, Vg = np.meshgrid(u, V, indexing="ij")
    z = bump(U, 30.0, 3.0, 0.06)
    z += 0.008 * np.sin(2 * np.pi * U / 1.2) * ramp(U, 62.0, 98.0, 2.0)
    r2 = ((U - 130.0) / 0.4) ** 2 + ((Vg - 0.8) / 0.25) ** 2  # pothole: 0.8 m long, 0.5 m wide
    z -= 0.05 * np.where(r2 < 1, 0.5 * (1 + np.cos(np.pi * np.sqrt(np.minimum(r2, 1)))), 0.0)
    bank = 0.04 * ramp(u, 170.0, 190.0, 10.0)  # dz/dv > 0: the left side higher, falling to the right
    return u, z, bank


def main(path=ROOT / "examples" / "data" / "test_track.crg"):
    u, z, bank = track()
    write_crg(path, DU, V, z, bank=bank, fmt="KRBI", comment="FreeDyn 2.0 test track (make_test_track.py)")
    print(f"wrote {path} ({Path(path).stat().st_size / 1e3:.0f} kB, {len(u)} x {len(V)} points)")


if __name__ == "__main__":
    main(*sys.argv[1:2])
