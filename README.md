# Mini Pupper 2 backflip

**A backflip for a 405 g quadruped with stock hobby servos.** This repo covers the physics, a MuJoCo simulator fitted to real flips, a CMA-ES trajectory optimizer, analysis tools for the robot's telemetry, and a 75-line firmware player. It also contains everything learned from 34 recorded flips.

<p align="center">
  <img src="docs/figures/flip.gif" width="420" alt="simulated K7 405 g backflip, slow motion"><br>
  <img src="docs/figures/sequence.png" width="900" alt="the flip, one pose every ~80 ms">
</p>

| | |
|---|---|
| robot | Mini Pupper 2, 405 g with battery, 12 stock servos, 3S pack (~12 V) |
| controller | open-loop keyframes: 9 poses, 650 ms from the first push to the end of the landing |
| real result | **K7 405 g: 272° turned at touchdown (262-280°), 3 / 3 landed upright** |
| sim accuracy | servo tracks within 6° RMS, body rotation within 9° RMS of the real flips |
| optimizer | CMA-ES on 10 robot models: paws-first landings 2 / 10 → 10 / 10 in the sim |

## Quick start

```bash
git clone <this repo> && cd minipupper-backflip
pip install -r requirements.txt

python examples/quickstart.py                       # simulate K7 405 g, compare with a real flip
python scripts/simulate.py k7_405 --robust          # on all 10 robot models
python scripts/simulate.py k7_405 --video k7.mp4    # slow-motion video (needs ffmpeg)
python scripts/analyze_recording.py --all           # measure the 34 real flips
python scripts/optimize.py k7_405                   # CMA-ES, ~10 min on 2 cores
python scripts/export_c.py                          # tables -> firmware/presets/*.h
python -m pytest                                    # tests, including C player == Python player
```

Rendering needs an OpenGL context. On a headless Linux machine, set `MUJOCO_GL=egl`.

## How it works

**1. The physics.** All of the spin is loaded in during the ~200 ms push. In the air, the rotation is just spin rate × flight time (≈ 1000 °/s × 0.2 s). The Mini Pupper flip touches down on its front paws at 250-280° and rolls over them for the rest. The push is limited by servo *speed*, not current, so battery voltage is the biggest knob: about 24° per volt. → [docs/01](docs/01-how-a-backflip-works.md)

![servo limits](docs/figures/servo_limits.png)

**2. Keyframes.** Nine poses: crouch, front push, rear push, tuck, reach, catch, sink, stand. A tiny player ramps the servos between them in straight lines. It is anchored to whatever pose the robot is standing in, and it refuses to play if any joint would pass its end stop. → [docs/02](docs/02-keyframe-flip.md)

![keyframes](docs/figures/keyframes.png)

**3. A simulator fitted to the robot.** MuJoCo model of the 405 g robot, including the knee's rod linkage. The servo model (stall torque, back-EMF, gains, 40 ms latency) is fitted by system identification to 14 real flips. Several models fit about equally well, so all six are kept as an ensemble. → [docs/03](docs/03-simulation.md)

![sim vs real](docs/figures/sim_vs_real.png)

**4. Trajectory optimization.** CMA-ES searches 42 pose and timing variables in a small box around a working flip. It maximizes `0.6 × mean + 0.4 × worst` of the flip score over 10 robot models, so the result has to work on all of them. → [docs/04](docs/04-trajectory-optimization.md)

![optimizer](docs/figures/optimizer.png)

**5. The real robot.** 34 recorded flips (K7 405 g, O1, S1-S7, with and without rubber shoes, on carpet and on artificial grass) and what they taught us about shoes, floors, twist, calibration and voltage. → [docs/05](docs/05-real-robot.md)

![real flips](docs/figures/real_flips.png)

**6. Your robot.** The same method, step by step, for another quadruped. → [docs/06](docs/06-port-to-your-robot.md)

## Repository

```
backflip/            Python package
  robot.py             MuJoCo model, servo map, ServoModel (fitted values + ensemble), floors
  keyframes.py         Preset: load/save, the player, rail check, knee cap, C export
  sim.py               simulate() -> FlipResult (time series + touchdown, flight, upright...), score()
  recording.py         load real telemetry, measure it the same way
  sysid.py             fit the servo model to recordings
  optimize.py          CMA-ES over a keyframe table, robust over many robot models
  render.py            video / GIF / chronophotograph
  plots.py             the figures
scripts/             command-line tools (simulate, optimize, fit, analyze, export, figures)
presets/             keyframe tables (JSON)            -> presets/README.md
firmware/            C player + generated preset headers + a host build for testing
model/               MuJoCo model (MJCF + meshes)
data/recordings/     34 real flips, 200 Hz telemetry   -> data/recordings/README.md
docs/                the guide, with figures
tests/               pytest: C/Python player equality, sim and fit sanity
examples/            quickstart.py
```

## Safety

A backflip is hard on the robot. Fly over a firm floor with 1 m of clear space, keep the battery charged, and put a cushion behind the robot the first time you try a new table. Never flip on a table edge. See [docs/05](docs/05-real-robot.md#before-you-flip).

## Acknowledgements

Mini Pupper 2 hardware by MangDang. Physics by [MuJoCo](https://mujoco.org). CMA-ES via [`cmaes`](https://github.com/CyberAgentAILab/cmaes).

## License

MIT, see [LICENSE](LICENSE). The robot meshes in `model/meshes` come from the Mini Pupper 2 CAD and remain under their original license.
