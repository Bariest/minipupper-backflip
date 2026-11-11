"""Keyframe tables: the backflip as a list of poses with timings, and the player that turns it into servo targets.

A table has N frames.  Frame 0 is the stand.  Each frame is 12 servo targets in *ticks relative to the stand*
(1 tick = 0.3027 deg; the left legs are mirrored, so a left servo's delta is minus the right one's).
When frame f starts ("is armed"), every servo ramps in a straight line from where it was told to be to the frame's
target in ``move_ms[f]``; frame f + 1 is armed ``move_ms[f] * blend_pct[f] / 100 + hold_ms[f]`` later.
That is exactly what the firmware player (firmware/backflip_player.c) does - tests/test_player.py checks the two
produce the same tick stream.
"""
from __future__ import annotations

import copy
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Sequence

import numpy as np

from .robot import DEG_PER_TICK, FLUSH_MS, SERVO_NAMES, TICK_MAX, TICK_MIN

PRESET_DIR = Path(__file__).resolve().parent.parent / "presets"
FRONT_KNEES = (2, 5)              # 0-based columns of servos 3 and 6
FRONT_KNEE_FOLD_MAX = 470         # ticks (142 deg) - see docs/05-real-robot.md "servo range"


@dataclass
class Preset:
    name: str
    frames: List[List[int]]                    # N x 12, ticks relative to the stand
    move_ms: List[int]
    hold_ms: List[int]
    blend_pct: List[int] = field(default_factory=list)
    stand_ticks: List[int] = field(default_factory=lambda: [511] * 12)   # absolute stand the table was made on
    description: str = ""

    def __post_init__(self):
        n = len(self.frames)
        if not self.blend_pct:
            self.blend_pct = [100] * n
        assert all(len(f) == 12 for f in self.frames), "every frame needs 12 servo values"
        assert len(self.move_ms) == len(self.hold_ms) == len(self.blend_pct) == n, "one timing per frame"

    # ---- io -----------------------------------------------------------------------------------------------
    @staticmethod
    def load(path_or_name) -> "Preset":
        p = Path(path_or_name)
        if not p.exists():
            p = PRESET_DIR / f"{path_or_name}.json"
        J = json.loads(p.read_text())
        return Preset(name=J["name"], frames=J["frames"], move_ms=J["move_ms"], hold_ms=J["hold_ms"],
                      blend_pct=J.get("blend_pct", []), stand_ticks=J.get("stand_ticks", [511] * 12),
                      description=J.get("description", ""))

    def to_json(self) -> str:
        rows = ",\n    ".join(json.dumps(f) for f in self.frames)
        head = dict(name=self.name, description=self.description,
                    units="frames: ticks relative to the stand, servos 1..12 (FR hip/thigh/knee, FL, RR, RL); "
                          "1 tick = 0.3027 deg; times in ms")
        body = json.dumps(head, indent=1)[:-2]
        return (body + f',\n "stand_ticks": {json.dumps(self.stand_ticks)},\n "frames": [\n    {rows}\n ],\n'
                f' "move_ms": {json.dumps(self.move_ms)},\n "hold_ms": {json.dumps(self.hold_ms)},\n'
                f' "blend_pct": {json.dumps(self.blend_pct)}\n}}\n')

    def save(self, path):
        Path(path).write_text(self.to_json())

    def copy(self, name: Optional[str] = None) -> "Preset":
        q = copy.deepcopy(self)
        if name:
            q.name = name
        return q

    # ---- facts ------------------------------------------------------------------------------------------
    @property
    def n(self) -> int:
        return len(self.frames)

    def duration_ms(self) -> int:
        return int(sum(m * b // 100 + h for m, b, h in zip(self.move_ms, self.blend_pct, self.hold_ms)))

    def arm_times_ms(self) -> List[int]:
        t, out = 0, []
        for m, b, h in zip(self.move_ms, self.blend_pct, self.hold_ms):
            out.append(t)
            t += m * b // 100 + h
        return out

    def rail_check(self, stand_ticks: Sequence[int]) -> List[tuple]:
        """frames that leave the servo range 0..1023 when played from this stand -> [(frame, servo, ticks over)]"""
        bad = []
        for f, fr in enumerate(self.frames):
            for i in range(12):
                v = stand_ticks[i] + fr[i]
                over = TICK_MIN - v if v < TICK_MIN else (v - TICK_MAX if v > TICK_MAX else 0)
                if over:
                    bad.append((f, i + 1, int(over)))
        return bad

    def cap_front_knees(self, cap: int = FRONT_KNEE_FOLD_MAX) -> "Preset":
        """fold the front knees at most `cap` ticks from the stand (an all-centred stand has 511 each way)"""
        q = self.copy()
        for fr in q.frames:
            for i in FRONT_KNEES:
                fr[i] = int(np.clip(fr[i], -cap, cap))
        return q

    def table(self) -> str:
        """a readable table (degrees) of the right-side servos (the left mirror them)"""
        cols = [1, 2, 7, 8]
        s = "frame  move  hold  | " + " ".join(f"{SERVO_NAMES[c]:>10s}" for c in cols) + "   (deg from stand)\n"
        for f, fr in enumerate(self.frames):
            s += (f"{f:5d} {self.move_ms[f]:5d} {self.hold_ms[f]:5d}  | "
                  + " ".join(f"{fr[c] * DEG_PER_TICK:10.0f}" for c in cols) + "\n")
        return s

    # ---- the player -------------------------------------------------------------------------------------
    def play(self, stand_ms: int = 300, hold_end_ms: int = 600, stand: Optional[Sequence[int]] = None):
        """-> (ticks, arms): absolute servo ticks, one row of 12 per 5 ms flush, starting with `stand_ms` of
        standing; arms[f] = the row at which frame f was armed.  Same arithmetic as the firmware player."""
        ref = [float(v) for v in (stand if stand is not None else self.stand_ticks)]
        target = [[ref[i] + fr[i] for i in range(12)] for fr in self.frames]
        goal = list(ref)
        out, arms, t = [], [], 0
        for _ in range(stand_ms // FLUSH_MS):
            t += FLUSH_MS
            out.append(list(goal))
        f, t_arm = 0, t
        arms.append(len(out))
        frm_from, frm_to, frm_ms = list(goal), target[0], self.move_ms[0]
        t_next = t + self.move_ms[0] * self.blend_pct[0] // 100 + self.hold_ms[0]
        end_t = None
        while True:
            t += FLUSH_MS
            if end_t is None and t >= t_next:
                if f + 1 >= self.n:
                    end_t = t
                else:
                    f += 1
                    arms.append(len(out))
                    frm_from, frm_to, frm_ms, t_arm = list(goal), target[f], self.move_ms[f], t
                    t_next = t + self.move_ms[f] * self.blend_pct[f] // 100 + self.hold_ms[f]
            dt = t - t_arm
            for i in range(12):
                p = frm_to[i]
                if frm_ms and dt < frm_ms:
                    p = frm_from[i] + (frm_to[i] - frm_from[i]) * dt / frm_ms
                goal[i] = float(int(min(float(TICK_MAX), max(float(TICK_MIN), p)) + 0.5))
            out.append(list(goal))
            if end_t is not None and t >= end_t + hold_end_ms:
                break
        return np.array(out), arms

    def servo_degrees(self, stand_ms: int = 300, hold_end_ms: int = 600):
        """-> (t, deg): time (s, 0 = the flip starts) and the 12 commanded servo angles (deg from the stand)"""
        ticks, _ = self.play(stand_ms, hold_end_ms)
        deg = (ticks - np.array(self.stand_ticks, float)) * DEG_PER_TICK
        t = np.arange(1, len(ticks) + 1) * FLUSH_MS / 1000.0 - stand_ms / 1000.0
        return t, deg

    # ---- export -----------------------------------------------------------------------------------------
    def to_c(self, symbol: str) -> str:
        """a C header for firmware/backflip_player.h"""
        rows = "\n".join("    { " + ", ".join(f"{v:5d}" for v in fr) + " }," for fr in self.frames)
        lst = lambda a: ", ".join(str(int(v)) for v in a)
        guard = symbol.upper() + "_H"
        return (f"/* {self.name} - generated by backflip.keyframes.Preset.to_c(); do not edit by hand.\n"
                f" * {self.description}\n"
                f" * Frames: servo ticks relative to the stand, servos 1..12 = FR hip/thigh/knee, FL, RR, RL. */\n"
                f"#ifndef {guard}\n#define {guard}\n#include \"backflip_player.h\"\n\n"
                f"static const int16_t {symbol}_frames[{self.n}][12] = {{\n{rows}\n}};\n"
                f"static const uint16_t {symbol}_move_ms[{self.n}]   = {{ {lst(self.move_ms)} }};\n"
                f"static const uint16_t {symbol}_hold_ms[{self.n}]   = {{ {lst(self.hold_ms)} }};\n"
                f"static const uint8_t  {symbol}_blend_pct[{self.n}] = {{ {lst(self.blend_pct)} }};\n\n"
                f"static const bf_table_t {symbol} = {{\n    \"{self.name}\", {self.n},\n"
                f"    {symbol}_frames, {symbol}_move_ms, {symbol}_hold_ms, {symbol}_blend_pct\n}};\n\n"
                f"#endif\n")


def mirror_check(p: Preset) -> int:
    """largest |right + left| difference in ticks (0 for a perfectly mirrored table)"""
    worst = 0
    for fr in p.frames:
        for r, l in ((1, 4), (2, 5), (7, 10), (8, 11), (0, 3), (6, 9)):
            worst = max(worst, abs(fr[r] + fr[l]))
    return worst
