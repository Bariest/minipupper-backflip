"""Measure real flips from their telemetry, and compare them with the sim.

    python scripts/analyze_recording.py flip_81 flip_82 flip_83          # the numbers
    python scripts/analyze_recording.py flip_81 --sim k7_405 --plot cmp.png   # + sim vs real plot
    python scripts/analyze_recording.py --all                            # every recording in data/recordings
    python scripts/analyze_recording.py my_flip.csv                      # your own recording
"""
import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from backflip import Preset, simulate  # noqa: E402
from backflip import recording as rec  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recordings", nargs="*")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--sim", help="preset the robot played: also simulate it and compare")
    ap.add_argument("--plot", help="write a sim-vs-real figure (png)")
    a = ap.parse_args()
    names = [r["file"] for r in rec.index()] if a.all else a.recordings
    if not names:
        ap.error("give recordings or --all")
    labels = {r["file"]: r for r in rec.index()}
    print(f"{'recording':12s} {'table':8s} {'turned at td':>12s} {'push->td':>9s} {'peak rate':>10s} "
          f"{'twist roll/yaw':>15s} {'peak I':>8s}  end")
    R = []
    for n in names:
        r = rec.load(n)
        R.append(r)
        m = r.metrics
        lab = labels.get(r.name, {}).get("preset", "?")
        print(f"{r.name:12s} {lab:8s} {m['rot_td']:9.0f} deg {m['push_to_td_ms']:6.0f} ms {m['peak_rate_dps']:6.0f} d/s "
              f"{m['roll_td']:+6.0f} / {m['yaw_td']:+4.0f} deg {m['peak_current_ma']:5.0f} mA  "
              f"{'upright' if m['upright'] else 'FELL'}")
    if a.sim:
        s = simulate(Preset.load(a.sim))
        print(f"sim {a.sim}: {s.summary()}")
        if a.plot:
            from backflip.plots import sim_vs_real
            sim_vs_real(s, R, a.plot, title=f"{Preset.load(a.sim).name}: sim vs {len(R)} real flips")
            print("wrote", a.plot)


if __name__ == "__main__":
    main()
