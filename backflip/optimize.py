"""Trajectory optimization of a keyframe table with CMA-ES, robust over several servo models.

Decision variables (all optional, see Space):
    pose   for every frame from 1 to N-2 and each of front thigh, front knee, rear thigh, rear knee: the target in
           ticks from the stand (the left legs mirror the right, the hips stay put)  ->  4 numbers per frame
    timing move_ms and hold_ms of the same frames                                  ->  2 numbers per frame
Each is searched within a box around the starting table, so the result stays a small, explainable edit of a
flip that already works on the real robot.

Objective: the flip is played on a set of robot models (the fitted ensemble + the best fit with weaker / stronger
servos, a slower servo and a different floor) and scored with backflip.sim.score;
    J = 0.6 * mean + 0.4 * worst  - rail penalty
so a table that works on one model and falls on another loses to one that works on all.  The rail penalty keeps
every joint inside the servo's 0..1023 range from an all-centred stand (servos at 155 deg = calibration offset 0).
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass, replace
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from .keyframes import Preset
from .robot import ENSEMBLE, ENSEMBLE_FLOORS, FITTED, Floor, Robot, ServoModel
from .sim import landing_score, score, simulate

PAIRS = {"front_thigh": (1, 4), "front_knee": (2, 5), "rear_thigh": (7, 10), "rear_knee": (8, 11)}


def robust_set(base: ServoModel = FITTED, volts: float = 12.0) -> List[Tuple[ServoModel, Floor]]:
    """the robots a table must work on: the fitted ensemble + the best fit at -8 % / +8 % strength, 8 ms more
    latency, and on a grippier floor"""
    out = [(sm.at(volts), fl) for sm, fl in zip(ENSEMBLE, ENSEMBLE_FLOORS)]
    b = base.at(volts)
    out += [(replace(b, strength=b.strength * 0.92), Floor(0.7)), (replace(b, strength=b.strength * 1.08), Floor(0.7)),
            (replace(b, latency_ms=b.latency_ms + 8), Floor(0.7)), (b, Floor(1.0))]
    return out


def landing_set(volts=(12.0, 12.6, 13.2)) -> List[Tuple[ServoModel, Floor]]:
    """the robots a LANDING must work on: the 6 fitted models at several battery voltages. The higher ones are the
    strong flips (fresh battery / power supply) that turn ~290 deg before the paws touch and then rock back hard."""
    return [(sm.at(v), fl) for v in volts for sm, fl in zip(ENSEMBLE, ENSEMBLE_FLOORS)]


OBJECTIVES = {"flip": score, "landing": landing_score}


@dataclass
class Space:
    base: Preset
    pose_range: int = 40              # ticks (+-12 deg) around the start, front legs
    rear_range: int = 60              # ticks (+-18 deg), rear legs: they do the push
    joints: Sequence[str] = tuple(PAIRS)
    timing: bool = True
    frames: Optional[Sequence[int]] = None
    timing_frames: Optional[Sequence[int]] = None   # frames whose move/hold may change (default: = frames)

    def __post_init__(self):
        n = self.base.n
        self.opt_frames = [f for f in range(1, n - 1) if self.frames is None or f in self.frames]
        tf = self.frames if self.timing_frames is None else self.timing_frames
        self.time_frames = [f for f in range(1, n - 1) if self.timing and (tf is None or f in tf)]
        names, lo, hi, x0 = [], [], [], []
        for f in sorted(set(self.opt_frames) | set(self.time_frames)):
            for k in (self.joints if f in self.opt_frames else ()):
                r = PAIRS[k][0]
                v = self.base.frames[f][r]
                rng = self.rear_range if k.startswith("rear") else self.pose_range
                names.append(f"f{f}.{k}")
                x0.append(v)
                lo.append(v - rng)
                hi.append(v + rng)
            if f in self.time_frames:
                for what, arr in (("move", self.base.move_ms), ("hold", self.base.hold_ms)):
                    v = arr[f]
                    names.append(f"f{f}.{what}")
                    x0.append(v)
                    lo.append(max(15 if what == "move" else 0, int(v * 0.5)))
                    hi.append(int(v * 1.6) + 30)
        self.names, self.x0 = names, np.array(x0, float)
        self.lo, self.hi = np.array(lo, float), np.array(hi, float)

    @property
    def dim(self) -> int:
        return len(self.names)

    def to_unit(self, x):
        return (np.asarray(x) - self.lo) / (self.hi - self.lo)

    def from_unit(self, u):
        return self.lo + np.clip(u, 0, 1) * (self.hi - self.lo)

    def build(self, x, name: Optional[str] = None) -> Preset:
        p = self.base.copy(name or self.base.name + " (optimized)")
        v = dict(zip(self.names, np.clip(x, self.lo, self.hi)))
        for f in self.opt_frames:
            for k in self.joints:
                r, l = PAIRS[k]
                d = int(round(v[f"f{f}.{k}"]))
                p.frames[f][r], p.frames[f][l] = d, -d
        for f in self.time_frames:
            if True:
                p.move_ms[f] = int(round(v[f"f{f}.move"]))
                p.hold_ms[f] = int(round(v[f"f{f}.hold"]))
        return p.cap_front_knees()


def rail_penalty(p: Preset, margin: int = 6) -> float:
    """ticks a joint would pass the end of its range from an all-centred (511) stand, x 2"""
    bad = 0
    for fr in p.frames:
        for v in fr:
            a = 511 + v
            bad += max(0, margin - a) + max(0, a - (1023 - margin))
    return 2.0 * bad


class Objective:
    def __init__(self, robots: Optional[List[Tuple[ServoModel, Floor]]] = None, kind: str = "flip"):
        self.robots = robots or robust_set()
        self.robot = Robot()
        self.score = OBJECTIVES[kind]

    def evaluate(self, p: Preset, detail: bool = False):
        outs = [simulate(p, sm, fl, self.robot).metrics for sm, fl in self.robots]
        sc = [self.score(o) for o in outs]
        J = 0.6 * float(np.mean(sc)) + 0.4 * float(min(sc)) - rail_penalty(p)
        return (J, outs) if detail else J


def describe(outs: List[Dict]) -> str:
    r = [o["rot_td"] for o in outs]
    ov = [o["overshoot"] for o in outs]
    return (f"turned {np.mean(r):.0f} deg at touchdown ({min(r):.0f}-{max(r):.0f}), "
            f"rocks back {np.mean(ov):.0f} deg (max {max(ov):.0f}), "
            f"paws first {sum(o['first_touch'] == 'paws' for o in outs)}/{len(outs)}, "
            f"upright {sum(o['upright'] for o in outs)}/{len(outs)}")


_W: Dict[str, object] = {}


def _winit(space: Space, robots, kind="flip"):
    _W["space"], _W["obj"] = space, Objective(robots, kind)


def _weval(u):
    sp = _W["space"]
    return _W["obj"].evaluate(sp.build(sp.from_unit(u)))


def run_cmaes(space: Space, robots=None, gens: int = 30, pop: int = 16, sigma: float = 0.15, seed: int = 0,
              jobs: int = 1, log: Callable[[str], None] = print, kind: str = "flip"):
    """-> (best Preset, best J, history [(gen, evals, best J so far, mean J of the generation)])"""
    from cmaes import CMA
    robots = robots or robust_set()
    obj = Objective(robots, kind)
    J0, outs0 = obj.evaluate(space.build(space.x0), detail=True)
    log(f"start: J = {J0:.1f}   {describe(outs0)}   ({space.dim} variables, {len(robots)} robot models)")
    u0 = space.to_unit(space.x0)
    opt = CMA(mean=u0, sigma=sigma, bounds=np.array([[0.0, 1.0]] * space.dim), seed=seed, population_size=pop)
    best_u, best_J, hist, evals = u0, J0, [(0, 1, J0, J0)], 1
    pool = None
    if jobs > 1:
        from multiprocessing import Pool
        pool = Pool(jobs, initializer=_winit, initargs=(space, robots, kind))
    else:
        _winit(space, robots, kind)
    t0 = time.time()
    try:
        for g in range(1, gens + 1):
            U = [opt.ask() for _ in range(opt.population_size)]
            Js = pool.map(_weval, U) if pool else [_weval(u) for u in U]
            opt.tell([(u, -J) for u, J in zip(U, Js)])          # cmaes minimizes
            evals += len(U)
            i = int(np.argmax(Js))
            if Js[i] > best_J:
                best_J, best_u = Js[i], U[i]
            hist.append((g, evals, best_J, float(np.mean(Js))))
            log(f"gen {g:3d}  best {best_J:6.1f}  this gen {max(Js):6.1f} / mean {np.mean(Js):6.1f}  "
                f"({time.time() - t0:.0f} s)")
    finally:
        if pool:
            pool.close()
    best = space.build(space.from_unit(best_u))
    J1, outs1 = obj.evaluate(best, detail=True)
    log(f"best:  J = {J1:.1f}   {describe(outs1)}")
    return best, J1, hist


def save_history(hist, path):
    with open(path, "w") as f:
        json.dump(dict(columns=["generation", "evaluations", "best_J", "mean_J"], rows=hist), f)
