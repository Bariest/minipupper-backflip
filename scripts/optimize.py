"""Optimize a keyframe table with CMA-ES, robust over 10 robot models (see backflip/optimize.py).

    python scripts/optimize.py k7_405                         # ~10 min on 2 cores -> presets/k7_405_opt.json
    python scripts/optimize.py k7_405 --gens 60 --pop 24 --jobs 8
    python scripts/optimize.py k7_405 --no-timing             # poses only
    python scripts/optimize.py k7_405 --frames 4 5 6 7        # only the flight and landing frames
    python scripts/optimize.py k7_405 --volts 11.1            # tune for a weaker battery
    python scripts/optimize.py k7_405 --objective landing --frames 5 6 7 --timing-frames 4 5 6 7 \
           --volts 12 12.5 13 13.5 --range 60 --rear-range 80 --name "Backflip K7 405 L2"
                                                              # the landing only, robust from a tired to a full battery

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
from backflip.optimize import Space, landing_set, robust_set, run_cmaes, save_history  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("table")
    ap.add_argument("--gens", type=int, default=30)
    ap.add_argument("--pop", type=int, default=16)
    ap.add_argument("--sigma", type=float, default=0.15, help="initial step, as a fraction of each variable's box")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 1)
    ap.add_argument("--volts", type=float, nargs="+", default=[12.0],
                    help="battery voltage(s); with --objective landing every fitted model is flown at each")
    ap.add_argument("--objective", choices=["flip", "landing"], default="flip",
                    help="flip: turn far, paws first, upright (10 robot models); landing: rock back little after "
                         "the paws touch, soft rear impact, upright (6 fitted models x each --volts)")
    ap.add_argument("--range", type=int, default=40, help="front-leg pose box, +- ticks")
    ap.add_argument("--rear-range", type=int, default=60, help="rear-leg pose box, +- ticks")
    ap.add_argument("--no-timing", action="store_true")
    ap.add_argument("--frames", type=int, nargs="*", help="frames whose poses may change (default: all)")
    ap.add_argument("--timing-frames", type=int, nargs="*", help="frames whose move/hold may change (default: --frames)")
    ap.add_argument("--name", help="name of the new table")
    ap.add_argument("--out", help="default presets/<table>_opt.json")
    a = ap.parse_args()
    base = Preset.load(a.table)
    sp = Space(base, a.range, a.rear_range, timing=not a.no_timing, frames=a.frames, timing_frames=a.timing_frames)
    robots = landing_set(a.volts) if a.objective == "landing" else robust_set(FITTED, a.volts[0])
    best, J, hist = run_cmaes(sp, robots, gens=a.gens, pop=a.pop, sigma=a.sigma, seed=a.seed, jobs=a.jobs,
                              kind=a.objective)
    best.name = a.name or f"{base.name} (CMA-ES)"
    best.description = (f"{base.name} optimized with CMA-ES ({a.gens} generations x {a.pop}, objective '{a.objective}') "
                         f"on {len(robots)} robot models at {'/'.join(f'{v:g}' for v in a.volts)} V; J = {J:.1f}. "
                         f"Not yet tried on the real robot.")
    out = Path(a.out) if a.out else ROOT / "presets" / f"{Path(a.table).stem}_opt.json"
    best.save(out)
    hist_path = ROOT / "data" / "optimizer" / f"{out.stem}.history.json"
    hist_path.parent.mkdir(parents=True, exist_ok=True)
    save_history(hist, hist_path)
    print("wrote", out)
    print(best.table())


if __name__ == "__main__":
    main()
