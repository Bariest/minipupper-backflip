"""Simulate a keyframe table and report the flip.

    python scripts/simulate.py k7_405                      # the fitted robot, 12 V, carpet
    python scripts/simulate.py k7_405 --robust             # on all 10 robot models the optimizer uses
    python scripts/simulate.py k7_405 s3 --volts 11.1      # several tables, a lower battery
    python scripts/simulate.py k7_405 --floor grass
    python scripts/simulate.py k7_405 --video k7.mp4       # slow-motion video (needs ffmpeg; or .gif)
"""
import argparse
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from backflip import FITTED, FLOORS, Preset, Robot, score, simulate  # noqa: E402
from backflip.optimize import describe, robust_set  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("tables", nargs="+", help="preset names (presets/<name>.json) or .json files")
    ap.add_argument("--volts", type=float, default=12.0)
    ap.add_argument("--floor", default="carpet", choices=list(FLOORS))
    ap.add_argument("--robust", action="store_true", help="every robot model of the robust set")
    ap.add_argument("--video", help="write a side-by-side slow-motion video (.mp4 or .gif)")
    ap.add_argument("--slow", type=float, default=4.0)
    a = ap.parse_args()
    rb = Robot()
    results = []
    for name in a.tables:
        p = Preset.load(name)
        if a.robust:
            outs = [simulate(p, sm, fl, rb).metrics for sm, fl in robust_set(FITTED, a.volts)]
            print(f"{p.name:28s} {describe(outs)}")
            for (sm, fl), o in zip(robust_set(FITTED, a.volts), outs):
                print(f"   strength {sm.strength:4.2f} damping {sm.damping:3.1f} kp {sm.kp:3.1f} latency {sm.latency_ms:3.0f} ms "
                      f"speed cap {sm.slew_dps or '-':>4} floor grip {fl.friction:3.1f}: turned {o['rot_td']:5.1f} deg, "
                      f"{o['first_touch']:5s} first, {'upright' if o['upright'] else 'FELL'}   score {score(o):6.1f}")
        r = simulate(p, FITTED.at(a.volts), FLOORS[a.floor], rb)
        results.append((p, r))
        print(f"{p.name:28s} fitted robot, {a.volts:g} V, {a.floor}: {r.summary()}")
    if a.video:
        os.environ.setdefault("MUJOCO_GL", "egl" if sys.platform.startswith("linux") else "glfw")
        from backflip import render
        render.video([r for _, r in results], [p.name for p, _ in results], a.video, slow=a.slow)
        print("wrote", a.video)


if __name__ == "__main__":
    main()
