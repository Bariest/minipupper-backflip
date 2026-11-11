# 6. Doing it on your robot

The method carries over to any small quadruped with position-controlled hobby servos. Here it is in order, with the step this repo does for each.

## 0. Can it flip at all?

Do a quick budget before writing any code. The robot needs roughly 250° of rotation by touchdown:

```
rotation ≈ angle at take-off + spin rate × flight time
flight time ≈ 2 × sqrt(2 g h)/g, h = how high the centre of mass rises
```

Mini Pupper reaches about 45° + 1000 °/s × 0.2 s with its centre of mass rising 7.7 cm. The spin rate is limited by how fast the rear thighs can swing while pushing, and that is close to the servo's no-load speed at your battery voltage. If the no-load speed is under ~600 °/s, or the robot weighs much more per unit of servo torque, expect to need a higher voltage first.

## 1. Get the model right

* Mass and its distribution matter most. Weigh the robot and the battery, and check the front/rear split on two kitchen scales.
* Model each leg's linkage the way the servo actually drives it. Mini Pupper's knee servo drives the shin through a rod, which is modelled as a tendon (`model/minipupper4.xml`).
* Keep the collision geometry simple: capsules and spheres. Paw size and friction matter. Shell detail doesn't.
* Write one function that turns your firmware's servo command into the model's joint angles (`backflip/robot.py: ServoMap`), so tables run in the sim unchanged.

## 2. Record, then fit the servo

Add 200 Hz logging of servo positions, currents and the IMU to your firmware, with a mark when the motion starts (CSV format: [`data/recordings/README.md`](../data/recordings/README.md)). Play any rough first table and record 3-10 attempts, even failed ones. Then fit stall torque, back-EMF, gains and latency to them ([section 3](03-simulation.md)). Latency is the parameter people forget, and it is worth 40 ms (40°) on Mini Pupper.

## 3. Start from a working shape

The nine-frame structure (crouch, front push, rear push, tuck, reach, catch, sink, stand) is a good starting point for any quadruped. Begin from `presets/k7_405.json` scaled to your servos' ranges, or pose the robot by hand in each phase and read the servo positions back. Then hand-tune in the sim until it gets past about 200° and lands on something.

## 4. Optimize, robustly

Fit an ensemble, not one model, and optimize the worst case over it ([section 4](04-trajectory-optimization.md)). Keep the search box small around your hand-made table.

## 5. Play it open-loop, but safely

Copy `firmware/backflip_player.c/.h` into your firmware and export the table (`scripts/export_c.py`). Then:

* anchor the table to the pose the robot is standing in, and **refuse** if any joint would pass its end stop;
* switch to stiff servo gains for the flip and restore them afterwards;
* time the loop from a hardware timer, because a frame late by 10 ms changes the landing;
* record the flip and compare it with the sim.

## 6. Iterate on the real robot

Expect the first real flips to land 10-30° away from the sim. Use the recordings to re-fit (step 2), not to hand-tweak angles. Then re-optimize. Two or three rounds of record → fit → optimize got Mini Pupper from falling back to landing on its paws.

## Where to go next

* **Closed-loop landing.** Detect the touchdown from the IMU (a > 3 g) and trigger the catch frame then, instead of on a timer. The full firmware has an experimental "learned landing timing" that shifts frame 5 by the measured error.
* **Reinforcement learning.** A policy trained in this sim with randomized servo models, the same ensemble idea taken further, could correct twist mid-air with the hips. The fitted model and the recordings here are a good starting point for that.
