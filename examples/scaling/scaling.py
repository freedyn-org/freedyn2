"""Scaling of legacy FreeDyn and FreeDyn 2.0 with the model size: N cars in one model.

  python examples/scaling/scaling.py --legacy build/legacy/build/FreeDyn [--cars 1 2 4 8]
         [--end-time 5] [--markdown scaling.md]

Each model holds N independent copies of the car of make_car.py (13 bodies, 60 constraint
equations, 16 expression forces and torques per car). Both programs run the same .fds as given,
single-threaded; reported are the wall-clock times (legacy: one program run; FreeDyn 2.0:
cold build and simulation), the FreeDyn 2.0 time per step, and the largest deviation between
the two results (position / rotation / velocity, as in freedyn2 compare), which shows that
both compute the same motion.
"""

import argparse
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "legacy"))
sys.path.insert(0, str(HERE))

import benchmark  # noqa: E402
import make_car  # noqa: E402


def main(argv=None):
    from fdgen import cli

    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--legacy", type=Path, required=True)
    ap.add_argument("--runtime")
    ap.add_argument("--cars", type=int, nargs="+", default=[1, 2, 4, 8])
    ap.add_argument("--end-time", type=float, default=5.0)
    ap.add_argument("--markdown", type=Path)
    args = ap.parse_args(argv)
    runtime = cli.find_runtime(args.runtime)
    hdr = ("| cars | bodies | constraints | expression forces | legacy [s] | v2 build cold + simulation [s] "
           "| v2 per step [us] | speed-up simulation | deviation v2 - legacy (pos / rot / vel) |")
    lines = [hdr, "|" + "---|" * (hdr.count("|") - 1)]
    for n in args.cars:
        print(f"{n} car(s) ...", file=sys.stderr, flush=True)
        with tempfile.TemporaryDirectory() as tmp:
            work = Path(tmp)
            fds = work / f"car_{n}.fds"
            fds.write_text(make_car.model(n, args.end_time), encoding="latin-1")
            t_legacy, legacy = benchmark.run_legacy(args.legacy.resolve(), fds, work / "legacy", 1)
            build, t_v2, v2 = benchmark.run_v2(runtime, fds, work / "v2", 1, cold=True)
        dev = np.max(benchmark.deviations(v2, legacy, benchmark._scales(legacy)), axis=0)
        lines.append(f"| {n} | {13 * n} | {60 * n} | {16 * n} | {t_legacy:.2f} | {build:.2f} + {t_v2:.2f} "
                     f"| {1e6 * t_v2 / v2['n_steps']:.0f} | {t_legacy / t_v2:.1f}x "
                     f"| {' / '.join(f'{x:.1e}' for x in dev)} |")
        print(lines[-1], file=sys.stderr, flush=True)
    table = "\n".join(lines)
    print(table)
    if args.markdown:
        args.markdown.write_text(table + "\n")


if __name__ == "__main__":
    main()
