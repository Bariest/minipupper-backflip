"""System identification: find the servo model that makes the sim fly like the real robot.

The sim's servo has a handful of unknowns (stall torque, back-EMF, gains, latency, a speed cap) and the floor has a
grip.  None of them can be read off a datasheet with enough accuracy for a flip, so they are fitted: replay the
exact keyframe table the robot played, in the sim, for every model on a grid, and compare with the recording.

Error per recording (lower is better):
    servo  RMS of the 8 thigh + knee servo tracks from the push to the touchdown, deg  - how fast the legs move
    pitch  RMS of the body rotation curve, deg                                          - how hard it was thrown
    td_ms  touchdown time, sim - real, ms                                               - the two numbers that
    td_deg how far it had turned at the touchdown, sim - real, deg                      - decide a flip
    error = servo + 0.5 pitch + 0.3 |td_ms| + 0.3 |td_deg|
"""
from __future__ import annotations

import itertools
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np

from .keyframes import Preset
from .recording import Recording
from .robot import Floor, Robot, ServoModel
from .sim import FlipResult, simulate

LEG = [1, 2, 4, 5, 7, 8, 10, 11]            # thigh + knee columns


def error(sim: FlipResult, rec: Recording) -> Tuple[float, Dict[str, float]]:
    m = rec.metrics
    t0 = m["t_push"] - 0.05
    ms = (rec.t > t0) & (rec.t < m["t_td"])
    mp = (rec.t > t0) & (rec.t < m["t_td"] + 0.05)
    es = float(np.sqrt(np.mean([(np.interp(rec.t[ms], sim.t, sim.servo[:, i]) - rec.servo[ms, i]) ** 2
                                for i in LEG])))
    ep = float(np.sqrt(np.mean((np.interp(rec.t[mp], sim.t, sim.pitch) - rec.pitch[mp]) ** 2)))
    dt = 1000.0 * (sim.metrics["t_td"] - m["t_td"])
    dr = sim.metrics["rot_td"] - m["rot_td"]
    return es + 0.5 * ep + 0.3 * abs(dt) + 0.3 * abs(dr), dict(servo=es, pitch=ep, td_ms=dt, td_deg=dr)


def grid(spec: Dict[str, Sequence]) -> List[Tuple[ServoModel, Floor]]:
    """spec: lists of values for strength, damping, kp, kv, latency_ms, slew_dps, friction"""
    keys = list(spec)
    out = []
    for v in itertools.product(*[spec[k] for k in keys]):
        d = dict(zip(keys, v))
        fr = d.pop("friction", 0.7)
        out.append((ServoModel(**d), Floor(friction=fr)))
    return out


FULL_GRID = dict(strength=[0.6, 0.75, 0.9, 1.05, 1.2, 1.4], damping=[1.0, 1.3, 1.7], slew_dps=[None, 500, 750],
                 latency_ms=[30, 40, 50], kp=[0.7, 1.0, 1.5, 2.0], friction=[0.7, 1.0])
QUICK_GRID = dict(strength=[0.75, 0.9, 1.2], damping=[1.0, 1.3], latency_ms=[30, 40], kp=[1.0, 2.0], friction=[0.7])


class Fitter:
    """Scores servo models against recordings.  recordings: [(Recording, Preset it played, weight)]."""

    def __init__(self, recordings: Iterable[Tuple[Recording, Preset, float]], volts: float = 12.0):
        self.recs = list(recordings)
        self.volts = volts
        self.robot = Robot()

    def score(self, model: Tuple[ServoModel, Floor]):
        sm, fl = model
        sm = sm.at(self.volts)
        cache, tot, wsum, per = {}, 0.0, 0.0, {}
        for rec, pre, w in self.recs:
            if pre.name not in cache:
                cache[pre.name] = simulate(pre, sm, fl, self.robot)
            e, info = error(cache[pre.name], rec)
            tot += w * e
            wsum += w
            per[rec.name] = info
        return tot / wsum, per


_F: Dict[str, Fitter] = {}


def _init(recs, volts):
    _F["f"] = Fitter(recs, volts)


def _job(model):
    e, per = _F["f"].score(model)
    return model, e, per


def fit(recordings, spec=FULL_GRID, volts: float = 12.0, jobs: int = 1, ensemble: int = 6):
    """-> (ranked [(model, error, per-recording)], ensemble of different models within 25 % of the best)"""
    models = grid(spec)
    if jobs > 1:
        from multiprocessing import Pool
        with Pool(jobs, initializer=_init, initargs=(recordings, volts)) as pool:
            res = pool.map(_job, models, chunksize=4)
    else:
        _init(recordings, volts)
        res = [_job(m) for m in models]
    res.sort(key=lambda r: r[1])
    ens = []
    for (sm, fl), e, _ in res:
        if e > 1.25 * res[0][1] or len(ens) >= ensemble:
            break
        if all(abs(sm.strength - q.strength) > 0.1 or sm.slew_dps != q.slew_dps or abs(sm.latency_ms - q.latency_ms) > 5
               or sm.kp != q.kp for q, _ in ens):
            ens.append((sm, fl))
    return res, ens
