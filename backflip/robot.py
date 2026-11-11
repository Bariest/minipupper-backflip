"""The robot: MuJoCo model, servo numbering, servo-angle -> joint-angle map, and the servo model.

Servo numbering (firmware ids, 1-based), per leg hip / thigh / knee:

    front right 1 2 3   front left 4 5 6   rear right 7 8 9   rear left 10 11 12

A *servo angle* in this package is what the firmware commands, in degrees, relative to the calibrated
stand pose (0 = standing).  Firmware ticks convert as ``deg = (tick - stand_tick) * DEG_PER_TICK``.
"""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path
from typing import Optional

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
MODEL_DIR = ROOT / "model"
SCENE = MODEL_DIR / "scene.xml"
SERVO_MAP = MODEL_DIR / "servo_map.json"

FLUSH_MS = 5                      # the firmware sends new servo targets every 5 ms
SCALE_DD = 3100.0                 # servo range: 1024 ticks = 310.0 deg
DEG_PER_TICK = SCALE_DD / 10240.0  # 0.3027 deg per tick
TICK_MIN, TICK_MAX = 0, 1023
NOMINAL_VOLTS = 12.0              # the servo model's "strength 1.0" is a 3S pack (~12 V)

LEG_ORDER = ("FR", "FL", "RR", "RL")
JOINT_ORDER = [f"{leg}_{j}" for leg in LEG_ORDER for j in ("hip", "thigh", "calf")]
SERVO_NAMES = [f"{leg} {j}" for leg in LEG_ORDER for j in ("hip", "thigh", "knee")]
THIGHS = [1, 4, 7, 10]            # 0-based column of each leg's thigh servo
KNEES = [2, 5, 8, 11]


class ServoMap:
    """Firmware servo angles (deg, 0 = stand) -> the 12 MuJoCo joint angles (rad).

    hip:   a plain rotation.
    thigh: firmware IK angle th1 = th1n + sign * servo.
    knee:  the knee servo sets the *shin's* angle to the vertical through a rod (th2 = 90 deg - shin);
           the model's knee is a hinge between thigh and shin, so its angle is shin - thigh.
    """

    def __init__(self, path: Path = SERVO_MAP):
        cfg = json.loads(Path(path).read_text())
        self.th1n = cfg["firmware"]["th1n_deg"]
        self.th2n = cfg["firmware"]["th2n_deg"]
        self.legs = cfg["legs"]

    def joint_angles(self, servo_deg) -> np.ndarray:
        q = np.zeros(12)
        for k, leg in enumerate(LEG_ORDER):
            L = self.legs[leg]
            s = L["servos"]
            a_hip, a_th, a_kn = (servo_deg[s[n] - 1] for n in ("hip", "thigh", "knee"))
            ax, z = L["axis_sign"], L["zero_pose_deg"]
            th1 = self.th1n + L["ik_sign"]["thigh"] * a_th
            th2 = self.th2n + L["ik_sign"]["knee"] * a_kn
            a1, a2 = th1, 90.0 - th2
            q[3 * k + 0] = math.radians(ax["hip"] * L["hip_dir"] * a_hip)
            q[3 * k + 1] = math.radians(-ax["thigh"] * (a1 - z["thigh"]))
            q[3 * k + 2] = math.radians(-ax["knee"] * ((a2 - a1) - (z["shin"] - z["thigh"])))
        return q

    def bind(self, m: mujoco.MjModel):
        acts = [m.actuator(i).name for i in range(m.nu)]
        self.act_to_q = [JOINT_ORDER.index(a) for a in acts]
        self.act_tendon = []                      # knee actuators drive a tendon: thigh and knee hinge together
        for i, a in enumerate(acts):
            if m.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_TENDON:
                leg = a.split("_")[0]
                ax = self.legs[leg]["axis_sign"]
                k = LEG_ORDER.index(leg)
                self.act_tendon.append((i, 3 * k + 1, -ax["thigh"], 3 * k + 2, -ax["knee"]))

    def set_ctrl(self, d: mujoco.MjData, q: np.ndarray):
        for i, qi in enumerate(self.act_to_q):
            d.ctrl[i] = q[qi]
        for i, it, ct, ik, ck in self.act_tendon:
            d.ctrl[i] = ct * q[it] + ck * q[ik]


@dataclass(frozen=True)
class Floor:
    """Contact model of the ground.  ``timeconst`` / ``dampratio`` are MuJoCo's solref: a larger time constant
    is a softer floor (it sinks in and gives energy back slowly)."""
    friction: float = 0.7
    timeconst: float = 0.01
    dampratio: float = 1.0

    @staticmethod
    def named(name: str) -> "Floor":
        return FLOORS[name]


FLOORS = {
    "carpet": Floor(0.7, 0.01, 1.0),        # fitted on the indoor flips (system identification) - the default
    "grippy": Floor(1.0, 0.01, 1.0),        # rubber shoes / high-grip mat (one of the fitted models used 1.0)
    "slippery": Floor(0.4, 0.01, 1.0),      # what-if: bare paws on a polished floor (not fitted)
    "grass": Floor(1.0, 0.06, 2.5),         # artificial grass: soft and damped; reproduces S3's three failed
                                            # grass flips (flips 112-114), not fitted on anything else
}


@dataclass(frozen=True)
class ServoModel:
    """How the sim's servos behave, as multipliers on the nominal MuJoCo servo.

    The sim servo is a position actuator:  torque = clip(kp * err - kv * speed, +-umax) - damping * speed.
    A DC motor's stall torque scales with the supply voltage, so ``volts`` scales ``umax``; the top speed is
    umax / damping, so it scales too.
    """
    strength: float = 0.9          # stall torque  x 0.8 N m
    damping: float = 1.0           # back-EMF      x 0.0381 N m s/rad   (top speed = strength / damping)
    kp: float = 2.0                # position gain x 8 N m/rad
    kv: float = 1.0                # velocity gain x 0.05 N m s/rad
    latency_ms: float = 40.0       # command -> motion delay (SPI + the servo's own loop)
    slew_dps: Optional[float] = None  # the servo's own speed cap on its set point at 12 V, deg/s (None: none)
    volts: float = 12.0
    mass: float = 1.0              # body mass x

    def to_dict(self) -> dict:
        return asdict(self)

    @staticmethod
    def from_dict(d: dict) -> "ServoModel":
        return ServoModel(**{k: d[k] for k in ServoModel.__dataclass_fields__ if k in d})

    def at(self, volts: float) -> "ServoModel":
        return replace(self, volts=volts)


# The servo model fitted to 14 real flips of the 405 g robot on the stock servos at 12 V (scripts/fit_servo_model.py,
# data/servo_fit.json).  FITTED is the best single model; ENSEMBLE are other, different models that fit almost as well
# - a keyframe table is only trusted when it works on all of them.
FITTED = ServoModel()
ENSEMBLE = [
    ServoModel(0.9, 1.0, 2.0, 1.0, 40, None),
    ServoModel(1.2, 1.3, 1.5, 1.0, 40, None),
    ServoModel(1.2, 1.3, 2.0, 1.0, 40, None),
    ServoModel(1.4, 1.0, 2.0, 1.0, 30, 750),
    ServoModel(1.4, 1.0, 1.5, 1.0, 30, 750),
    ServoModel(0.9, 1.0, 1.5, 1.0, 40, None),
]
ENSEMBLE_FLOORS = [Floor(0.7), Floor(0.7), Floor(1.0), Floor(1.0), Floor(1.0), Floor(0.7)]


@dataclass
class Robot:
    """One MuJoCo model + data, re-tuned in place for a ServoModel and a Floor.
    display=False: the robot without its 10 g display (screen) at the front."""
    display: bool = True
    m: mujoco.MjModel = field(init=False)
    d: mujoco.MjData = field(init=False)

    def __post_init__(self):
        self.m = mujoco.MjModel.from_xml_path(str(SCENE))
        self.d = mujoco.MjData(self.m)
        self.map = ServoMap()
        self.map.bind(self.m)
        m = self.m
        self.base = m.body("base").id
        self.root_q = m.joint("root").qposadr[0]
        self.root_v = m.joint("root").dofadr[0]
        self.floor_geoms = {i for i in range(m.ngeom) if m.geom_bodyid[i] == 0}
        self.foot_geoms = {i for i in range(m.ngeom) if "foot" in (m.geom(i).name or "")}
        self._nom = dict(gain=m.actuator_gainprm[:, 0].copy(), bias=m.actuator_biasprm[:, :3].copy(),
                         frc=m.actuator_forcerange.copy(), fric=m.geom_friction.copy(), mass=m.body_mass.copy(),
                         inertia=m.body_inertia.copy(), dof_damp=m.dof_damping.copy(),
                         ten_damp=m.tendon_damping.copy())
        if not self.display:
            self._nom["mass"][m.body("display_mass").id] = 0.0
            self._nom["inertia"][m.body("display_mass").id] = 1e-12
            for g in range(m.ngeom):                                   # and hide the screen in renders
                if m.body(int(m.geom_bodyid[g])).name == "base" and int(m.geom_matid[g]) >= 0 and \
                        m.material(int(m.geom_matid[g])).name in ("display_frame", "screen", "face"):
                    m.geom_rgba[g, 3] = 0.0
        self._key = m.key("stand").id
        # servo angle (deg) -> actuator length is affine; measure it once to read the servo positions back
        c0 = self._ctrl_for(np.zeros(12))
        self._c0, self._slope = c0, np.zeros(12)
        for i in range(12):
            e = np.zeros(12)
            e[i] = 1.0
            self._slope[i] = (self._ctrl_for(e) - c0)[i]
        self.servo_model, self.floor = None, None
        self.configure(FITTED, Floor())

    def _ctrl_for(self, servo_deg) -> np.ndarray:
        d = mujoco.MjData(self.m)
        self.map.set_ctrl(d, self.map.joint_angles(servo_deg))
        return d.ctrl.copy()

    def configure(self, sm: ServoModel, floor: Floor = Floor()):
        m, n = self.m, self._nom
        v = sm.volts / NOMINAL_VOLTS
        m.actuator_forcerange[:] = n["frc"] * sm.strength * v
        m.actuator_gainprm[:, 0] = n["gain"] * sm.kp
        m.actuator_biasprm[:, 1] = n["bias"][:, 1] * sm.kp
        m.actuator_biasprm[:, 2] = n["bias"][:, 2] * sm.kv
        m.dof_damping[:] = n["dof_damp"]
        m.tendon_damping[:] = n["ten_damp"]
        for i in range(m.nu):                   # back-EMF on the dof / tendon each servo drives
            if m.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_JOINT:
                m.dof_damping[m.jnt_dofadr[m.actuator_trnid[i, 0]]] *= sm.damping
            else:
                m.tendon_damping[m.actuator_trnid[i, 0]] *= sm.damping
        m.body_mass[:] = n["mass"] * sm.mass
        m.body_inertia[:] = n["inertia"] * sm.mass
        m.geom_friction[:, 0] = n["fric"][:, 0] * floor.friction
        m.geom_solref[:, 0] = floor.timeconst
        m.geom_solref[:, 1] = floor.dampratio
        self.servo_model, self.floor = sm, floor
        return self

    def reset(self):
        mujoco.mj_resetDataKeyframe(self.m, self.d, self._key)
        mujoco.mj_forward(self.m, self.d)

    def command(self, servo_deg):
        self.map.set_ctrl(self.d, self.map.joint_angles(servo_deg))

    def servo_angles(self) -> np.ndarray:
        """where the 12 servos are now, in servo degrees (what the servo's encoder would report)"""
        return (self.d.actuator_length - self._c0) / self._slope

    def contacts(self):
        """-> (number of foot contacts, True if anything else touches the floor)"""
        feet, body = 0, False
        for c in self.d.contact[: self.d.ncon]:
            g1, g2 = c.geom1, c.geom2
            if g1 in self.foot_geoms or g2 in self.foot_geoms:
                feet += 1
            elif g1 in self.floor_geoms or g2 in self.floor_geoms:
                body = True
        return feet, body
