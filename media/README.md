# Videos of real flips

Phone videos (60 fps, cropped and re-encoded to 960x540), each with its telemetry in `data/recordings`.

| video | recording | table | notes | sync: video time = flip time + |
|---|---|---|---|---|
| `flip_115_no_screen.mp4` | `flip_115` | K7 405 g | 2026-11-11, display removed, grass mat | 2.62 s |
| `flip_116_with_screen.mp4` | `flip_116` | K7 405 g | 2026-11-11, with display, grass mat | 2.60 s |
| `flip_l2_no_screen.mp4` | - | **K7 405 L2** | 2026-11-11, display removed, grass mat; no telemetry | sim touchdown at 6.45 s in the video |
| `flip_l2_with_screen.mp4` | - | **K7 405 L2** | 2026-11-11, with display, grass mat; no telemetry | sim touchdown at 4.55 s in the video |
| `sim2real_flip_115.mp4` | `flip_115` | K7 405 g / L2 | flip 115 next to the sim (K7 405 g and L2), 3x slower | - |

The sync offset was found from the touchdown: the frame where the front paws touch vs the IMU's touchdown time.
`backflip.render.sim_vs_real_video()` uses it to play the video next to the simulation (`docs/figures/sim2real.gif`). The L2 videos have no telemetry: they are synced by putting the simulated touchdown on the video frame where the front paws touch (`render.sim2real_grid`, `docs/figures/hero_l2.gif`).
