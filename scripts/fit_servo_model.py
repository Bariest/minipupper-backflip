"""Fit the sim's servo model to real flips (system identification, see backflip/sysid.py).

    python scripts/fit_servo_model.py                 # full grid, ~1300 models: ~25 min on 2 cores
    python scripts/fit_servo_model.py --quick         # 24 models, ~1 min
    python scripts/fit_servo_model.py --recordings flip_81 flip_82 flip_83 --volts 12

The recordings must say which table they played (data/recordings/index.csv, or --table for all of them).
Writes data/servo_fit.json; copy the best model / ensemble into backflip/robot.py (FITTED, ENSEMBLE) to use it.
"""
import argparse
import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from backflip import Preset  # noqa: E402
from backflip import recording as rec  # noqa: E402
from backflip.sysid import FULL_GRID, QUICK_GRID, fit  # noqa: E402

DEFAULT = [f"flip_{i}" for i in range(81, 95)]          # K7 405 g x3 and O1 x11, no shoes, 12 V


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--recordings", nargs="*", default=DEFAULT)
    ap.add_argument("--table", help="the table every recording played (default: from index.csv)")
    ap.add_argument("--volts", type=float, default=12.0)
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 1)
    ap.add_argument("--out", default=str(ROOT / "data" / "servo_fit.json"))
    a = ap.parse_args()
    idx = {r["file"]: r["preset"] for r in rec.index()}
    recs, count = [], {}
    for n in a.recordings:
        key = a.table or idx[n]
        count[key] = count.get(key, 0) + 1
    for n in a.recordings:
        key = a.table or idx[n]
        recs.append((rec.load(n), Preset.load(key), 1.0 / count[key]))   # every table weighs the same
    print(f"{len(recs)} recordings: " + ", ".join(f"{k} x{v}" for k, v in count.items()))
    res, ens = fit(recs, QUICK_GRID if a.quick else FULL_GRID, a.volts, a.jobs)
    print(f"{len(res)} models; best:")
    for (sm, fl), e, _ in res[:8]:
        print(f"  error {e:5.1f}   strength {sm.strength:4.2f} damping {sm.damping:3.1f} kp {sm.kp:3.1f} "
              f"latency {sm.latency_ms:3.0f} ms speed cap {sm.slew_dps or '-'}  floor grip {fl.friction}")
    (sm, fl), e, per = res[0]
    J = dict(volts=a.volts, best=dict(servo=sm.to_dict(), friction=fl.friction, error=e,
                                      per_recording={k: {kk: round(vv, 1) for kk, vv in v.items()} for k, v in per.items()}),
             ensemble=[dict(servo=s.to_dict(), friction=f.friction) for s, f in ens],
             ranked=[dict(servo=s.to_dict(), friction=f.friction, error=round(err, 2)) for (s, f), err, _ in res[:50]],
             recordings=a.recordings)
    Path(a.out).write_text(json.dumps(J, indent=1))
    print("wrote", a.out)


if __name__ == "__main__":
    main()
