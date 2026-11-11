# Presets

Keyframe tables (`*.json`, loaded by `backflip.Preset.load`). Frames are servo ticks relative to the stand pose. Servos 1..12 = FR hip/thigh/knee, FL, RR, RL. 1 tick = 0.3027°, times are in ms. The front-knee cap (470 ticks) and the firmware's +6 ms landing shift on frame 5 are already included.

| file | table | on the real robot |
|---|---|---|
| `k7_405.json` | **Backflip K7 405 g** | **recommended**: 405 g robot, stock servos, 12 V, bare paws. Turned 272° at touchdown, 3 / 3 upright |
| `k7_405_opt.json` | K7 405 g after CMA-ES | sim: paws first on 10 / 10 robot models (K7 405 g: 2 / 10). **Not yet tried on the robot** |
| `o1.json` | O1 | 253°, 10 / 11 upright: paws first more often, vaults more |
| `s3.json` | S3 | for rubber shoes on carpet: 237°, 6 / 6 upright. Weaker without shoes |
| `g1.json` | G1 | re-timed for artificial grass in the sim. Not tried on the robot |
| `k7_360.json` | K7 | the original flip for the 360 g robot (no battery) at ~14.5 V, kept for comparison |

```
python scripts/simulate.py k7_405 k7_405_opt --robust     # compare on the 10 robot models
python scripts/export_c.py                                # -> firmware/presets/*.h
```
