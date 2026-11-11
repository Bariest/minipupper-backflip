"""Real flips: read a recording and measure it the same way the sim is measured.

A recording is the robot's 200 Hz telemetry CSV (web page -> "Record the next flip"):

    # mark_s=1.4936          <- the moment frame 0 was armed (t = 0 here)
    t_s, pos1..pos12 (deg, the servo's raw angle), cur1..cur12 (mA), ax ay az (g), gx gy gz (deg/s)

IMU axes on Mini Pupper 2 (BMI270): x = left, y = forward, z = down.  So the backflip's rotation (nose up and over)
is -gx, roll is +gy and yaw is -gz.  The servo's raw angle runs opposite to the firmware's servo angle.
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict

import numpy as np

DATA_DIR = Path(__file__).resolve().parent.parent / "data" / "recordings"


@dataclass
class Recording:
    name: str
    t: np.ndarray            # s, 0 = the flip started
    servo: np.ndarray        # servo angles, deg from the stand before the flip (N x 12), firmware sign
    current: np.ndarray      # mA (N x 12)
    accel: np.ndarray        # g (N x 3)
    gyro: np.ndarray         # deg/s (N x 3)
    pitch: np.ndarray        # how far it has turned backwards, deg (integrated gyro)
    roll: np.ndarray         # deg (integrated; only meaningful while it flies)
    yaw: np.ndarray          # deg (integrated)
    meta: Dict[str, str] = field(default_factory=dict)
    metrics: Dict[str, float] = field(default_factory=dict)

    @property
    def accel_g(self) -> np.ndarray:
        return np.linalg.norm(self.accel, axis=1)


def load(path_or_name) -> Recording:
    p = Path(path_or_name)
    if not p.exists():
        p = DATA_DIR / f"{path_or_name}.csv"
    txt = p.read_text()
    meta = dict(re.findall(r"^#\s*(\w+)=([^\s#]+)", txt, re.M))
    mark = float(meta.get("mark_s", 0.0))
    lines = [ln for ln in txt.splitlines() if ln and not ln.startswith("#")]
    cols = lines[0].split(",")
    A = np.array([[float(x) for x in ln.split(",")] for ln in lines[1:]])
    D = {c: A[:, i] for i, c in enumerate(cols)}
    t = D["t_s"] - mark
    dt = np.gradient(D["t_s"])
    pos = np.stack([D[f"pos{i}"] for i in range(1, 13)], 1)
    servo = -(pos - pos[t < 0].mean(0))
    integ = lambda w: np.cumsum(w * dt)
    pitch = integ(-D["gx"])
    pitch -= pitch[t < 0.5].mean()
    r = Recording(name=p.stem, t=t, servo=servo, current=np.stack([D[f"cur{i}"] for i in range(1, 13)], 1),
                  accel=np.stack([D["ax"], D["ay"], D["az"]], 1), gyro=np.stack([D["gx"], D["gy"], D["gz"]], 1),
                  pitch=pitch, roll=integ(D["gy"]), yaw=integ(-D["gz"]), meta=meta)
    r.metrics = measure(r)
    return r


def measure(r: Recording) -> Dict[str, float]:
    """spin start = the first time the rotation rate passes 200 deg/s; push = the first jolt over 2 g in the 150 ms
    before that (the crouch earlier in the wind-up can jolt too); touchdown = the first jolt over 3 g once it has
    turned past 150 deg."""
    a = r.accel_g
    rate = -r.gyro[:, 0]
    i_spin = int(np.where((r.t > 0.5) & (rate > 200))[0][0])
    i_push = np.where((r.t > r.t[i_spin] - 0.15) & (a > 2.0))[0]
    i_push = int(i_push[0]) if len(i_push) else i_spin
    td = np.where((r.t > r.t[i_push]) & (r.pitch > 150) & (a > 3.0))[0]
    i_td = int(td[0]) if len(td) else len(r.t) - 1
    end = r.t > r.t[-1] - 0.3
    # twist: roll and yaw picked up between the spin start and the touchdown
    tw = lambda x: float(x[i_td] - x[i_spin])
    from .sim import landing_metrics
    return dict(**landing_metrics(r.t, r.pitch, a, float(r.t[i_td])),
                t_push=float(r.t[i_push]), t_td=float(r.t[i_td]), rot_td=float(r.pitch[i_td]),
                push_to_td_ms=float(1000 * (r.t[i_td] - r.t[i_push])), peak_rate_dps=float(rate[:i_td + 1].max()),
                roll_td=tw(r.roll), yaw_td=tw(r.yaw), peak_g=float(a[i_td:i_td + 40].max()),
                upright=bool(r.accel[end, 2].mean() < -0.8), rot_end=float(r.pitch[-1]),
                peak_current_ma=float(np.abs(r.current[(r.t > 0) & (r.t < r.t[i_td])]).max()))


def index() -> list:
    """data/recordings/index.csv as a list of dicts"""
    rows = (DATA_DIR / "index.csv").read_text().strip().splitlines()
    head = rows[0].split(",")
    return [dict(zip(head, r.split(","))) for r in rows[1:]]
