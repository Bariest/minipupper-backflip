"""Quick start: simulate the recommended flip, compare it with a real one, and nudge a frame.

    python examples/quickstart.py
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from backflip import ENSEMBLE, Preset, simulate  # noqa: E402
from backflip import recording  # noqa: E402

flip = Preset.load("k7_405")                 # presets/k7_405.json
print(flip.table())

result = simulate(flip)                      # the servo model fitted to the real robot, 12 V, carpet
print("sim:  ", result.summary())

real = recording.load("flip_81")             # data/recordings/flip_81.csv - the same table on the robot
print(f"real:  turned {real.metrics['rot_td']:.0f} deg at touchdown, "
      f"{'ended upright' if real.metrics['upright'] else 'fell'}")

# a what-if: push 5 degrees harder with the rear thighs in frame 3 (17 ticks; the left mirror the right)
harder = flip.copy("rear push +5 deg")
harder.frames[3][7] -= 17
harder.frames[3][10] += 17
for p in (flip, harder):
    outs = [simulate(p, servo).metrics for servo in ENSEMBLE]
    print(f"{p.name:22s} on 6 fitted models: turned {min(o['rot_td'] for o in outs):.0f}-"
          f"{max(o['rot_td'] for o in outs):.0f} deg, upright {sum(o['upright'] for o in outs)}/6")
