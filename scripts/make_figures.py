"""Make every figure in docs/figures (about 2 minutes; the renders need an OpenGL context - on a headless Linux
box this script uses EGL).

    python scripts/make_figures.py            # all
    python scripts/make_figures.py voltage    # some: keyframes anatomy sim_vs_real servo voltage optimizer
                                              #       real floors sysid render landing sim2real hero
"""
import json
import os
import sys
from pathlib import Path

if sys.platform.startswith("linux"):
    os.environ.setdefault("MUJOCO_GL", "egl")
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
import numpy as np  # noqa: E402

from backflip import ENSEMBLE, ENSEMBLE_FLOORS, FITTED, FLOORS, Floor, Preset, Robot, simulate  # noqa: E402
from backflip import plots  # noqa: E402
from backflip import recording as rec  # noqa: E402
from backflip.optimize import Objective, robust_set  # noqa: E402
from backflip.sysid import Fitter  # noqa: E402

FIG = ROOT / "docs" / "figures"
K7 = Preset.load("k7_405")
RB = Robot()


def fig_keyframes():
    plots.keyframes(K7, FIG / "keyframes.png")


def fig_anatomy():
    plots.anatomy(simulate(K7, robot=RB), K7, FIG / "anatomy.png")


def fig_sim_vs_real():
    recs = [rec.load(f"flip_{i}") for i in (81, 82, 83)]
    band = [simulate(K7, sm, fl, RB) for sm, fl in zip(ENSEMBLE, ENSEMBLE_FLOORS)]
    plots.sim_vs_real(simulate(K7, robot=RB), recs, FIG / "sim_vs_real.png",
                      "K7 405 g: the fitted sim against the three real flips (flips 81-83, 12 V)", band=band)


def fig_servo():
    r = simulate(K7, robot=RB)
    arms = [a / 1000 for a in K7.arm_times_ms()]
    k = (r.t >= arms[3]) & (r.t <= arms[4])                          # frame 3: the rear push
    sp, tq = [], []
    damp = RB.m.dof_damping[RB.m.jnt_dofadr[RB.m.joint("RR_thigh").id]]
    for c in (7, 10):
        w = RB._slope[c] * np.gradient(r.servo[:, c], r.t)            # joint speed, rad/s
        tau = r.torque[:, c] * np.sign(w) - damp * np.abs(w)           # net torque along the motion
        sp += list(np.abs(w[k]) * 57.2958)
        tq += list(tau[k])
    umax = float(RB.m.actuator_forcerange[7, 1])
    recs = [rec.load(f"flip_{i}") for i in (81, 82, 83)]
    plots.servo_envelope(dict(speed=sp, torque=tq), recs, FIG / "servo_limits.png", umax, damp,
                         umax * 14.5 / 12.0, 14.5)


def fig_voltage():
    volts = np.arange(10.5, 14.51, 0.5)
    rot = [[simulate(K7, sm.at(v), fl, RB).metrics["rot_td"] for sm, fl in zip(ENSEMBLE, ENSEMBLE_FLOORS)]
           for v in volts]
    real = [rec.load(f"flip_{i}").metrics["rot_td"] for i in (81, 82, 83)]
    slope = np.polyfit(volts, np.array(rot)[:, 0], 1)[0]
    plots.voltage(volts, rot, 12.0, real, FIG / "voltage.png", per_volt=slope)
    print(f"  voltage: {slope:.1f} deg per volt (best fit)")


def fig_optimizer():
    p = ROOT / "presets" / "k7_405_opt.json"
    if not p.exists():
        print("  (no presets/k7_405_opt.json yet: run scripts/optimize.py k7_405)")
        return
    opt = Preset.load(p)
    hist = json.loads((ROOT / "data" / "optimizer" / "k7_405_opt.history.json").read_text())["rows"]
    obj = Objective(robust_set())
    _, before = obj.evaluate(K7, detail=True)
    _, after = obj.evaluate(opt, detail=True)
    labels = ["fit 1", "fit 2", "fit 3", "fit 4", "fit 5", "fit 6", "-8 %\nstrength", "+8 %\nstrength",
              "+8 ms\nlatency", "grippy\nfloor"]
    plots.optimizer(hist, before, after, labels, FIG / "optimizer.png")


def fig_real():
    rows = []
    for r in rec.index():
        rows.append(dict(r, metrics=rec.load(r["file"]).metrics))
    plots.real_flips(rows, FIG / "real_flips.png")


def fig_floors():
    res = {nm: simulate(K7, FITTED, FLOORS[nm], RB) for nm in ("carpet", "grippy", "slippery", "grass")}
    plots.floors(res, FIG / "floors.png")


def fig_sysid():
    recs = [(rec.load(f"flip_{i}"), K7, 1 / 3) for i in (81, 82, 83)] + \
           [(rec.load(f"flip_{i}"), Preset.load("o1"), 1 / 11) for i in range(84, 95)]
    F = Fitter(recs)
    S = np.array([0.6, 0.75, 0.9, 1.05, 1.2, 1.4])
    L = np.array([20, 30, 40, 50, 60])
    E = np.array([[F.score((FITTED.__class__(**{**FITTED.to_dict(), "strength": s, "latency_ms": float(l)}),
                            Floor(0.7)))[0] for l in L] for s in S])
    plots.landscape(S, L, E, (FITTED.strength, FITTED.latency_ms), FIG / "sysid.png")


def fig_render():
    from backflip import render
    r = simulate(K7, robot=RB)
    render.chronophoto(r, [2.30, 2.50, 2.60, 2.68, 2.76, 2.84, 3.30], FIG / "sequence.png", spread=0.25,
                       distance=1.05, elevation=-8, height=900, width=2200)
    render.video([r], ["K7 405 g (sim)"], FIG / "flip.gif", slow=3.0, t0=2.25, t1=3.35, width=480, height=360,
                 fps=25, distance=0.42, elevation=-14)


def fig_landing():
    from backflip.optimize import Objective, landing_set
    volts = (11.5, 12.0, 12.5, 13.0, 13.5)
    L2 = Preset.load("k7_405_l2")
    _, before = Objective(landing_set(volts), "landing").evaluate(K7, detail=True)
    _, after = Objective(landing_set(volts), "landing").evaluate(L2, detail=True)
    like_today = (ENSEMBLE[2].at(12.5), ENSEMBLE_FLOORS[2])        # the fitted model that matches flips 115-116
    plots.landing([rec.load("flip_115"), rec.load("flip_116")], simulate(K7, *like_today, RB), before, after, volts,
                  FIG / "landing.png", sim_after=simulate(L2, *like_today, RB),
                  older=[rec.load(f"flip_{i}") for i in (81, 82, 83)])


def fig_sim2real():
    from backflip import render
    like_today = (ENSEMBLE[2].at(12.5), ENSEMBLE_FLOORS[2])
    sims = [simulate(K7, *like_today, RB), simulate(Preset.load("k7_405_l2"), *like_today, RB)]
    render.sim_vs_real_video(ROOT / "media" / "flip_115_no_screen.mp4", 2.62, sims,
                             ["sim: K7 405 g", "sim: K7 405 L2 (new landing)"], FIG / "sim2real.gif",
                             real_label="real: K7 405 g (flip 115)", t0=2.36, t1=3.40, width=384, height=216,
                             step=2, colors=128)


def fig_hero():
    """the README's headline: K7 405 L2 on the real robot, without and with the display, next to the sim"""
    from backflip import Robot, render
    like = (ENSEMBLE[2].at(12.5), ENSEMBLE_FLOORS[2])
    L2 = Preset.load("k7_405_l2")
    rows = []
    for disp, mp4, td_video, rl in ((False, "flip_l2_no_screen.mp4", 6.45, "real: K7 405 L2, no display"),
                                    (True, "flip_l2_with_screen.mp4", 4.55, "real: K7 405 L2, with display")):
        rb = Robot(display=disp)
        res = simulate(L2, *like, rb)
        off = td_video - res.metrics["t_td"]                 # sync the two at the touchdown (front paws)
        rows.append((ROOT / "media" / mp4, off, res, rl, "sim: K7 405 L2, " + ("with display" if disp else "no display"), rb))
    render.sim2real_grid(rows, FIG / "hero_l2.gif", t0=2.42, t1=3.42, distance=0.45, colors=112)


ALL = dict(keyframes=fig_keyframes, anatomy=fig_anatomy, sim_vs_real=fig_sim_vs_real, servo=fig_servo,
           voltage=fig_voltage, optimizer=fig_optimizer, real=fig_real, floors=fig_floors, sysid=fig_sysid,
           render=fig_render, landing=fig_landing, sim2real=fig_sim2real, hero=fig_hero)


def main():
    FIG.mkdir(parents=True, exist_ok=True)
    for name in sys.argv[1:] or ALL:
        print(name)
        ALL[name]()


if __name__ == "__main__":
    main()
