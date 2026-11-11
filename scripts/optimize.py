"""Optimize a keyframe table with CMA-ES, robust over 10 robot models (see backflip/optimize.py).

    python scripts/optimize.py k7_405                         # ~10 min on 2 cores -> presets/k7_405_opt.json
    python scripts/optimize.py k7_405 --gens 60 --pop 24 --jobs 8
    python scripts/optimize.py k7_405 --no-timing             # poses only
    python scripts/optimize.py k7_405 --frames 4 5 6 7        # only the flight and landing frames
    python scripts/optimize.py k7_405 --volts 11.1            # tune for a weaker battery

Always try a new table on the real robot over something soft first: the sim is good at telling which of two
tables turns further, less good at the exact angle (see docs/03-simulation.md).
"""
import argparse
import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from backflip import FITTED, Preset  # noqa: E402
from backflip.optimize import Space, robust_set, run_cmaes, save_history  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("table")
    ap.add_argument("--gens", type=int, default=30)
    ap.add_argument("--pop", type=int, default=16)
    ap.add_argument("--sigma", type=float, default=0.15, help="initial step, as a fraction of each variable's box")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 1)
    ap.add_argument("--volts", type=float, default=12.0)
    ap.add_argument("--range", type=int, default=40, help="front-leg pose box, +- ticks")
    ap.add_argument("--rear-range", type=int, default=60, help="rear-leg pose box, +- ticks")
    ap.add_argument("--no-timing", action="store_true")
    ap.add_argument("--frames", type=int, nargs="*")
    ap.add_argument("--name", help="name of the new table")
    ap.add_argument("--out", help="default presets/<table>_opt.json")
    a = ap.parse_args()
    base = Preset.load(a.table)
    sp = Space(base, a.range, a.rear_range, timing=not a.no_timing, frames=a.frames)
    best, J, hist = run_cmaes(sp, robust_set(FITTED, a.volts), gens=a.gens, pop=a.pop, sigma=a.sigma,
                              seed=a.seed, jobs=a.jobs)
    best.name = a.name or f"{base.name} (CMA-ES)"
    best.description = (f"{base.name} optimized with CMA-ES ({a.gens} generations x {a.pop}) on 10 robot models at "
                         f"{a.volts:g} V; objective J = {J:.1f}. Not yet tried on the real robot.")
    out = Path(a.out) if a.out else ROOT / "presets" / f"{Path(a.table).stem}_opt.json"
    best.save(out)
    hist_path = ROOT / "data" / "optimizer" / f"{out.stem}.history.json"
    hist_path.parent.mkdir(parents=True, exist_ok=True)
    save_history(hist, hist_path)
    print("wrote", out)
    print(best.table())


if __name__ == "__main__":
    main()
