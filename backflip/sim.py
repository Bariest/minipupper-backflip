"""Play a keyframe table on the simulated robot and measure the flip.

The loop is the firmware's: every 5 ms a new set of 12 servo targets (delayed by the servo latency) and five
1 ms physics steps.  The measurements are the ones that can also be taken on the real robot from its IMU, so sim
and real can be compared on the same numbers (see backflip/recording.py).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Optional

import mujoco
import numpy as np

from .keyframes import Preset
from .robot import FITTED, FLUSH_MS, NOMINAL_VOLTS, Floor, Robot, ServoModel

G = 9.81
STAND_MS = 300            # the sim stands this long before frame 0 is armed
HOLD_END_MS = 600         # ... and keeps going this long after the last frame


@dataclass
class FlipResult:
    """Time series (one row per 5 ms) and the flip's key numbers."""
    t: np.ndarray                 # s, 0 = frame 0 armed (the same zero as a recording's mark)
    cmd: np.ndarray               # commanded servo angles, deg from the stand (N x 12)
    servo: np.ndarray             # servo angles reached, deg from the stand (N x 12)
    torque: np.ndarray            # servo torque, N m (N x 12)
    pitch: np.ndarray             # how far the body has turned backwards, deg (unwrapped; 360 = a full flip)
    roll: np.ndarray              # deg
    z: np.ndarray                 # body height, m
    x: np.ndarray                 # body travel, m (+ = forwards)
    feet: np.ndarray              # paws on the floor
    body: np.ndarray              # something other than a paw touches the floor
    accel_g: np.ndarray           # what an accelerometer on the body reads, |a| in g
    qpos: np.ndarray              # the full MuJoCo state (positions) for rendering (N x nq)
    metrics: Dict[str, float] = field(default_factory=dict)

    def summary(self) -> str:
        m = self.metrics
        return (f"turned {m['rot_td']:5.1f} deg at touchdown ({1000 * m['t_td']:4.0f} ms), "
                f"flight {m['flight_ms']:3.0f} ms, apex {100 * m['apex_m']:4.1f} cm, first touch {m['first_touch']}, "
                f"{'LANDED upright' if m['upright'] else 'did NOT land upright'}")


def _pitch_angle(R: np.ndarray) -> float:
    return math.degrees(math.atan2(R[2, 0], R[0, 0]))


def simulate(preset: Preset, servo: ServoModel = FITTED, floor: Floor = Floor(), robot: Optional[Robot] = None,
             stand_ms: int = STAND_MS, hold_end_ms: int = HOLD_END_MS) -> FlipResult:
    rb = robot or Robot()
    rb.configure(servo, floor)
    m, d = rb.m, rb.d
    t, cmd = preset.servo_degrees(stand_ms, hold_end_ms)
    if servo.slew_dps:                                  # the servo's own speed limit on its set point
        step = servo.slew_dps * (servo.volts / NOMINAL_VOLTS) * FLUSH_MS / 1000.0
        c = cmd.copy()
        for k in range(1, len(c)):
            c[k] = c[k - 1] + np.clip(cmd[k] - c[k - 1], -step, step)
        cmd = c
    lag = int(round(servo.latency_ms / FLUSH_MS))
    nsub = int(round(FLUSH_MS / (m.opt.timestep * 1000)))
    rb.reset()
    n = len(cmd)
    S, TQ = np.zeros((n, 12)), np.zeros((n, 12))
    P, RL, Z, X, F, B, A = (np.zeros(n) for _ in range(7))
    Q = np.zeros((n, m.nq))
    F = F.astype(int)
    B = B.astype(bool)
    rot, prev = 0.0, None
    v_prev = d.qvel[rb.root_v:rb.root_v + 3].copy()
    for k in range(n):
        rb.command(cmd[max(0, k - lag)])
        for _ in range(nsub):
            mujoco.mj_step(m, d)
        R = d.xmat[rb.base].reshape(3, 3)
        a = _pitch_angle(R)
        if prev is not None:
            rot += (a - prev + 180.0) % 360.0 - 180.0
        prev = a
        P[k] = rot
        RL[k] = math.degrees(math.atan2(R[2, 1], R[2, 2])) if abs(rot) < 90 else np.nan
        Q[k] = d.qpos
        Z[k] = d.qpos[rb.root_q + 2]
        X[k] = d.qpos[rb.root_q]
        S[k] = rb.servo_angles()
        TQ[k] = d.actuator_force
        F[k], B[k] = rb.contacts()
        v = d.qvel[rb.root_v:rb.root_v + 3].copy()
        A[k] = np.linalg.norm((v - v_prev) / (FLUSH_MS / 1000.0) + np.array([0.0, 0.0, G])) / G
        v_prev = v
    off = P[t < 0.5].mean()
    P -= off
    R = d.xmat[rb.base].reshape(3, 3)
    res = FlipResult(t=t, cmd=cmd, servo=S, torque=TQ, pitch=P, roll=RL, z=Z, x=X - X[0], feet=F, body=B, accel_g=A, qpos=Q)
    res.metrics = measure(res, upright_end=float(R[2, 2]), offset=off)
    return res


def landing_metrics(t, pitch, accel_g, t_td: float) -> Dict[str, float]:
    """How the landing went after the front paws touch (shared by the sim and the real recordings):
    overshoot  how far it rocks past where it ends up, deg: the body rolls over the front paws and, with spin left,
               the rear slams down and it sits back nose-up before settling (real K7 405 g at full power: 40-47 deg)
    impact2_g  the hardest jolt 50-350 ms after the touchdown: the rear legs coming down, g"""
    end = pitch[t > t[-1] - 0.3].mean()
    k = (t >= t_td) & (t < t_td + 0.8)
    k2 = (t >= t_td + 0.05) & (t < t_td + 0.35)
    return dict(rot_max_after=float(pitch[k].max()), overshoot=float(max(0.0, pitch[k].max() - end)),
                impact2_g=float(accel_g[k2].max()) if k2.any() else 0.0, rot_settled=float(end))


def measure(r: FlipResult, upright_end: float, offset: float = 0.0) -> Dict[str, float]:
    """The flip's numbers, detected the way the robot's IMU would see them:
    touchdown = the first jolt over 3 g once it has been airborne and turned past 150 deg."""
    air = np.cumsum((r.feet == 0) & ~r.body)               # flushes with nothing on the floor so far
    raw = r.pitch + offset                                  # turned since the very start
    late = (air >= 6) & (raw > 150)
    i_td = np.where(late & (r.accel_g > 3.0))[0]
    i_td = int(i_td[0]) if len(i_td) else len(r.t) - 1
    touch = np.where(late & ((r.feet > 0) | r.body))[0]
    first = "none" if not len(touch) else ("body" if r.body[touch[0]] else "paws")
    i_land = int(touch[0]) if len(touch) else i_td
    on = np.where(((r.feet > 0) | r.body) & (np.arange(len(r.t)) < i_land))[0]
    i_off = int(on[-1]) + 1 if len(on) else 0                # take-off: the start of the last flight before landing
    rate = np.gradient(r.pitch, r.t)
    return dict(
        **landing_metrics(r.t, r.pitch, r.accel_g, float(r.t[i_td])),
        rot_td=float(r.pitch[i_td]), t_td=float(r.t[i_td]), first_touch=first,
        upright=bool(upright_end > 0.85 and abs(raw[-1] - 360.0) < 60.0), upright_end=upright_end,
        rot_end=float(r.pitch[-1]), t_takeoff=float(r.t[i_off]),
        flight_ms=float(1000.0 * (r.t[i_land] - r.t[i_off])),
        apex_m=float(r.z[i_off:i_land + 1].max() - r.z[0]) if i_land > i_off else 0.0,
        peak_rate_dps=float(rate[: i_td + 1].max()), rot_takeoff=float(r.pitch[i_off]),
        body_ms=float(FLUSH_MS * r.body[i_land:].sum()), peak_g=float(r.accel_g[i_land:i_land + 40].max()),
    )


def landing_score(m: Dict[str, float]) -> float:
    """Score for tuning the LANDING only (the push frames stay as they are): stand up, paws first, rock back as
    little as possible after the paws touch, and bring the rear legs down softly.  Higher is better, 0 is perfect."""
    s = -1.0 * max(0.0, m["overshoot"] - 10.0) - 3.0 * max(0.0, m["impact2_g"] - 6.0)
    if m["first_touch"] != "paws":
        s -= 25.0
    if not m["upright"]:
        s -= 200.0
    return s


def score(metrics: Dict[str, float], target: float = 355.0, over: float = 390.0) -> float:
    """One number for "how good is this flip" (higher is better): turn as far as possible before the paws touch
    (up to `target`), paws first, and stand up at the end.  The same rule the tables in presets/ were tuned with."""
    r = metrics["rot_td"]
    s = min(r, target) - 2.0 * max(0.0, r - 372.0) - 25.0 * max(0.0, r - over)
    if metrics["first_touch"] != "paws":
        s -= 40.0
    if not metrics["upright"]:
        s -= 200.0
    return s
