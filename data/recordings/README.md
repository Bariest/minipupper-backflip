# Recordings

36 real backflips of one Mini Pupper 2: 405 g, stock servos, 3S at about 12 V, Strong gains, 1500 mA limit. `index.csv` says which table each one played, on which floor, and with or without shoes.

## Format

The firmware's telemetry log at 200 Hz:

```
# pupper_telem 1
# hz=200
# imu=bmi270
# mark_s=1.493600       # t at which frame 0 was armed (the analysis puts t = 0 here)
t_s,pos1,...,pos12,cur1,...,cur12,ax,ay,az,gx,gy,gz
```

| column | unit | notes |
|---|---|---|
| `t_s` | s | |
| `pos1..12` | deg | the servo's raw angle (it runs opposite to the firmware's servo angle) |
| `cur1..12` | mA | motor current |
| `ax ay az` | g | IMU frame: x left, y forward, z down |
| `gx gy gz` | °/s | the backflip's rotation is −gx |

Load one with `backflip.recording.load("flip_81")`, or a file path. To add your own, drop the CSV here and add a row to `index.csv`.
