"""Figures for the docs (scripts/make_figures.py makes all of them).  One style everywhere: light surface,
recessive grid, thin lines, a fixed colour order, direct labels where there are few series."""
from __future__ import annotations

from typing import Dict, List, Optional, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from .keyframes import Preset  # noqa: E402
from .robot import DEG_PER_TICK  # noqa: E402

SURFACE, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
C = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
GOOD, CRITICAL = "#0ca30c", "#d03b3b"
BLUES = ["#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#2a78d6", "#1c5cab", "#104281", "#0d366b"]

# what each frame of the K7-family tables does (frame 0 = the stand)
K7_PHASES = ["stand", "crouch", "front push", "rear push", "tuck", "reach", "catch", "sink", "stand up"]


def style():
    plt.rcParams.update({
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
        "font.family": "DejaVu Sans", "font.size": 10.5, "axes.titlesize": 12, "axes.titleweight": "bold",
        "axes.titlelocation": "left", "axes.labelcolor": INK2, "axes.edgecolor": AXIS, "axes.linewidth": 0.8,
        "xtick.color": MUTED, "ytick.color": MUTED, "xtick.labelcolor": INK2, "ytick.labelcolor": INK2,
        "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6, "axes.spines.top": False,
        "axes.spines.right": False, "lines.linewidth": 2.0, "legend.frameon": False, "legend.fontsize": 9.5,
        "text.color": INK, "figure.dpi": 110, "savefig.dpi": 150, "savefig.bbox": "tight",
    })


style()


def _save(fig, path):
    fig.savefig(path)
    plt.close(fig)
    return path


def _frames(axes, preset: Preset, t0: float, t1: float, labels=K7_PHASES):
    """shade every other frame on all axes (x in ms) and name the frames above the first axes; names of
    neighbouring short frames go on two rows so they do not collide"""
    arms = [a / 1000.0 for a in preset.arm_times_ms()] + [preset.duration_ms() / 1000.0]
    row, prev_short = 0, False
    for f in range(preset.n):
        a, b = max(arms[f], t0), min(arms[f + 1], t1)
        if b <= a:
            continue
        if f % 2:
            for ax in axes:
                ax.axvspan(a * 1000, b * 1000, color="#f0efec", zorder=0, lw=0)
        short = arms[f + 1] - arms[f] < 0.09
        row = (row + 1) % 2 if short and prev_short else 0
        prev_short = short
        name = labels[f] if labels and f < len(labels) else ""
        axes[0].text((a + b) / 2 * 1000, 1.02 + 0.09 * row, f"{f} {name}", ha="center", va="bottom", fontsize=8,
                     color=INK2, transform=axes[0].get_xaxis_transform())


# ---------------------------------------------------------------------------------------------------------------
def keyframes(preset: Preset, path, t0=2.15, t1=3.35):
    """the commanded servo angles of the right legs (the left mirror them), frame by frame"""
    t, deg = preset.servo_degrees()
    fig, axes = plt.subplots(2, 1, figsize=(10, 6.2), sharex=True)
    for ax, (cols, leg) in zip(axes, (((1, 2), "Front legs"), ((7, 8), "Rear legs"))):
        for k, (c, nm) in enumerate(zip(cols, ("thigh", "knee"))):
            m = (t >= t0) & (t <= t1)
            ax.plot(t[m] * 1000, deg[m, c], color=C[k], label=nm)
            ax.annotate(nm, (t[m][-1] * 1000, deg[m, c][-1]), xytext=(4, 0), textcoords="offset points",
                        color=INK2, fontsize=9, va="center")
        ax.axhline(0, color=AXIS, lw=0.8)
        ax.set_ylabel("deg from stand")
        ax.set_title(leg, pad=34 if leg.startswith("Front") else 6)
    _frames(axes, preset, t0, t1)
    for a in axes:
        a.set_xlim(t0 * 1000, t1 * 1000 + 60)
    axes[1].set_xlabel(f"time since the flip started, ms   (the first {t0:.2f} s, standing and crouching, not shown)")
    fig.suptitle(f"{preset.name}: what the servos are told to do", x=0.01, ha="left", fontweight="bold",
                 fontsize=13)
    fig.tight_layout()
    return _save(fig, path)


def anatomy(res, preset: Preset, path, t0=2.2, t1=3.3):
    """one simulated flip: rotation, height, contacts, and the rear thigh's command vs where it got"""
    m = res.metrics
    fig, ax = plt.subplots(4, 1, figsize=(10, 9), sharex=True, gridspec_kw=dict(height_ratios=[3, 2, 0.8, 2.2]))
    T = res.t * 1000
    k = (res.t >= t0) & (res.t <= t1)
    ax[0].plot(T[k], res.pitch[k], color=C[0])
    for y in (90, 180, 270, 360):
        ax[0].axhline(y, color=GRID, lw=0.8, zorder=0)
    ax[0].set_ylabel("body turned, deg")
    ax[0].set_yticks([0, 90, 180, 270, 360])
    for tt, lab in ((m["t_takeoff"], "take-off"), (m["t_td"], f"touchdown: {m['rot_td']:.0f} deg")):
        for a in ax:
            a.axvline(tt * 1000, color=MUTED, lw=1, ls="--")
        ax[0].annotate(lab, (tt * 1000, 330 if lab == "take-off" else 140), xytext=(5, 0),
                       textcoords="offset points", color=INK2, fontsize=9)
    ax[0].set_title("Anatomy of a simulated flip (fitted robot, 12 V)", pad=34)
    ax[1].plot(T[k], (res.z[k] - res.z[0]) * 100, color=C[2])
    ax[1].set_ylabel("body height, cm\n(from the stand)")
    ax[1].axhline(0, color=AXIS, lw=0.8)
    feet = res.feet[k] > 0
    body = res.body[k]
    ax[2].fill_between(T[k], 0.55, 0.95, where=feet, color=C[0], step="mid", lw=0)
    ax[2].fill_between(T[k], 0.05, 0.45, where=body, color=C[1], step="mid", lw=0)
    ax[2].set_yticks([0.25, 0.75])
    ax[2].set_yticklabels(["body / knee\non floor", "paws on floor"])
    ax[2].set_ylim(0, 1)
    ax[2].grid(False)
    ax[3].plot(T[k], res.cmd[k, 7], color=MUTED, lw=1.5, ls="--", label="rear thigh: commanded")
    ax[3].plot(T[k], res.servo[k, 7], color=C[0], label="rear thigh: reached")
    ax[3].plot(T[k], res.cmd[k, 8], color=MUTED, lw=1.5, ls=":", label="rear knee: commanded")
    ax[3].plot(T[k], res.servo[k, 8], color=C[1], label="rear knee: reached")
    ax[3].set_ylabel("servo, deg from stand")
    ax[3].legend(ncol=2, loc="upper right")
    ax[3].set_xlabel("time since the flip started, ms")
    _frames(list(ax), preset, t0, t1)
    ax[0].set_xlim(t0 * 1000, t1 * 1000)
    fig.tight_layout()
    return _save(fig, path)


def sim_vs_real(sim, recs: List, path, title: str, band: Optional[List] = None, t0=2.3, t1=3.2):
    """pitch and the rear-thigh / rear-knee servo tracks: real flips vs the sim"""
    fig, ax = plt.subplots(1, 3, figsize=(14, 4.4))
    panels = (("body turned, deg", None), ("rear thigh, deg", 7), ("rear knee, deg", 8))
    for a, (lab, col) in zip(ax, panels):
        for n, r in enumerate(recs):
            k = (r.t >= t0) & (r.t <= t1)
            y = r.pitch[k] if col is None else r.servo[k, col]
            a.plot(r.t[k] * 1000, y, color=INK2 if n == 0 else MUTED, lw=1.2, alpha=0.9,
                   label="real flips" if n == 0 else None)
        if band:
            ys = np.array([b.pitch if col is None else b.servo[:, col] for b in band])
            k = (band[0].t >= t0) & (band[0].t <= t1)
            a.fill_between(band[0].t[k] * 1000, ys[:, k].min(0), ys[:, k].max(0), color=C[0], alpha=0.15, lw=0,
                           label="sim, 6 fitted models")
        k = (sim.t >= t0) & (sim.t <= t1)
        a.plot(sim.t[k] * 1000, sim.pitch[k] if col is None else sim.servo[k, col], color=C[0], label="sim, best fit")
        a.set_title(lab)
        a.set_xlabel("ms since the flip started")
    for r in recs:
        ax[0].plot(r.metrics["t_td"] * 1000, r.metrics["rot_td"], "o", color=INK2, ms=5)
    ax[0].plot(sim.metrics["t_td"] * 1000, sim.metrics["rot_td"], "o", color=C[0], ms=7, mec=SURFACE, mew=1.5)
    ax[0].annotate("dots: touchdown", (sim.metrics["t_td"] * 1000, sim.metrics["rot_td"]), xytext=(25, -45),
                   textcoords="offset points", color=INK2, fontsize=9,
                   arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8))
    ax[0].legend(loc="upper left")
    fig.suptitle(title, x=0.01, ha="left", fontweight="bold", fontsize=13)
    fig.tight_layout()
    return _save(fig, path)


def servo_envelope(points: Dict, recs: List, path, umax: float, damping: float, umax_hi: float, volts_hi: float):
    """left: rear-thigh torque vs speed in the push, against the motor's torque-speed line.
    right: the real rear servos' current against the 1500 mA cap."""
    fig, ax = plt.subplots(1, 2, figsize=(13, 4.6))
    w = np.linspace(0, 25, 50)
    ax[0].plot(w * 57.3, np.maximum(umax - damping * w, 0), color=INK2, lw=1.5)
    ax[0].plot(w * 57.3, np.maximum(umax_hi - damping * w, 0), color=MUTED, lw=1.5, ls="--")
    ax[0].annotate("motor limit at 12 V", (300, umax - damping * 300 / 57.3), xytext=(10, 8),
                   textcoords="offset points", color=INK2, fontsize=9)
    ax[0].annotate(f"at {volts_hi:g} V", (900, umax_hi - damping * 900 / 57.3), xytext=(6, 6),
                   textcoords="offset points", color=MUTED, fontsize=9)
    ax[0].scatter(points["speed"], points["torque"], s=34, color=C[0], edgecolor=SURFACE, lw=0.8, zorder=5,
                  label="sim: rear thighs during the push, every 5 ms")
    ax[0].set_xlabel("servo speed, deg/s")
    ax[0].set_ylabel("torque pushing, N m")
    ax[0].set_ylim(0, umax_hi * 1.08)
    ax[0].set_xlim(0, 1450)
    ax[0].legend(loc="upper right")
    ax[0].set_title("The push runs along the motor's speed limit")
    for n, r in enumerate(recs):
        k = (r.t > 2.2) & (r.t < 2.9)
        I = np.abs(r.current[k][:, [6, 7, 8, 9, 10, 11]]).max(1)
        ax[1].plot(r.t[k] * 1000, I, color=[C[0], C[1], C[2]][n % 3], lw=1.3, label=r.name)
    ax[1].axhline(1500, color=CRITICAL, lw=1.2)
    ax[1].annotate("current cap 1500 mA", (2210, 1500), xytext=(0, 5), textcoords="offset points", color=INK2, fontsize=9)
    ax[1].set_ylim(0, 1700)
    ax[1].set_xlabel("ms since the flip started")
    ax[1].set_ylabel("largest rear-servo current, mA")
    ax[1].set_title("... while the current stays under its cap (real flips)")
    ax[1].legend(loc="upper right")
    fig.tight_layout()
    return _save(fig, path)


def voltage(volts, rot, real_volts, real_rot, path, per_volt=None):
    """rot: (len(volts), n_models) rotation at touchdown"""
    fig, ax = plt.subplots(figsize=(8.5, 4.6))
    rot = np.asarray(rot)
    ax.fill_between(volts, rot.min(1), rot.max(1), color=C[0], alpha=0.15, lw=0, label="sim, range of 6 fitted models")
    ax.plot(volts, rot[:, 0], color=C[0], marker="o", ms=5, label="sim, best fit")
    ax.plot([real_volts] * len(real_rot), real_rot, "D", color=INK2, ms=6, label="real flips at ~12 V")
    ax.axhspan(270, 330, color="#e8f4e8", zorder=0, lw=0)
    ax.annotate("lands on its paws comfortably", (volts[0], 300), color=INK2, fontsize=9, va="center")
    ax.set_xlabel("battery voltage under load, V   (3S pack: 12.6 full, ~11.1 nominal)")
    ax.set_ylabel("turned when it touches down, deg")
    ax.set_title(f"K7 405 g: every volt is worth ~{per_volt:.0f} deg of rotation (sim)" if per_volt else
                 "K7 405 g: rotation at touchdown vs battery voltage (sim)")
    ax.legend(loc="lower right")
    fig.tight_layout()
    return _save(fig, path)


def optimizer(hist, before: List[Dict], after: List[Dict], labels: List[str], path):
    fig, ax = plt.subplots(1, 2, figsize=(14, 4.6), gridspec_kw=dict(width_ratios=[1, 1.5]))
    H = np.array(hist)
    ax[0].plot(H[:, 1], H[:, 3], color=MUTED, lw=1.2, label="mean of each generation")
    ax[0].plot(H[:, 1], H[:, 2], color=C[0], label="best so far")
    ax[0].set_xlabel("flips simulated (x10 robot models each)")
    ax[0].set_ylabel("objective J (higher is better)")
    ax[0].set_title("CMA-ES convergence")
    ax[0].legend(loc="lower right")
    x = np.arange(len(labels))
    for k, (outs, nm, col) in enumerate(((before, "start (K7 405 g)", MUTED), (after, "optimized", C[0]))):
        r = [o["rot_td"] for o in outs]
        ax[1].bar(x + (k - 0.5) * 0.38, r, 0.36, color=col, label=nm)
        for i, o in enumerate(outs):
            mk = "" if o["upright"] else "fell"
            if o["first_touch"] == "paws":
                mk = ("paws " + mk).strip()
            if mk:
                ax[1].text(x[i] + (k - 0.5) * 0.38, r[i] + 3, mk, ha="center", va="bottom", fontsize=7, color=INK2,
                           rotation=90)
    ax[1].set_xticks(x)
    ax[1].set_xticklabels(labels, fontsize=8)
    ax[1].set_ylim(200, 330)
    ax[1].set_ylabel("turned at touchdown, deg")
    ax[1].set_title("Every robot model, before and after (label: paws touched first / fell)")
    ax[1].legend(loc="upper left", ncol=2)
    fig.tight_layout()
    return _save(fig, path)


def real_flips(rows: List[Dict], path):
    """rows: dict(file, preset, surface, shoes, metrics)"""
    order = ["k7_405", "o1", "S1", "O2", "S2", "s3", "S5", "S6", "S7", "s3 grass", "k7_405 grass"]
    names = {"k7_405": "K7\n405 g", "o1": "O1", "s3": "S3", "s3 grass": "S3\ngrass", "k7_405 grass": "K7 405 g\ngrass mat"}
    groups = {k: [] for k in order}
    for r in rows:
        key = r["preset"] + (" grass" if "grass" in r["surface"] else "")
        if key in groups:
            groups[key].append(r)
    fig, ax = plt.subplots(1, 2, figsize=(14, 4.6))
    for i, k in enumerate(order):
        g = groups[k]
        jit = np.linspace(-0.16, 0.16, len(g)) if len(g) > 1 else [0]
        for j, r in zip(jit, g):
            m = r["metrics"]
            ok = m["upright"]
            for a, y in ((ax[0], m["rot_td"]), (ax[1], abs(m["roll_td"]))):
                a.plot(i + j, y, "o" if ok else "X", color=C[0] if ok else CRITICAL, ms=7 if ok else 8,
                       mec=SURFACE, mew=0.8)
        if g:
            ax[0].plot([i - 0.25, i + 0.25], [np.mean([r["metrics"]["rot_td"] for r in g])] * 2, color=INK2, lw=1.5)
    for a in ax:
        a.set_xticks(range(len(order)))
        a.set_xticklabels([names.get(k, k) for k in order])
    ax[0].set_ylabel("turned when the paws hit, deg")
    ax[0].set_title(f"{sum(len(g) for g in groups.values())} real flips: rotation at touchdown (bar = mean)")
    ax[1].set_ylabel("roll picked up in the air, deg")
    ax[1].set_title("Twist at touchdown")
    ax[0].plot([], [], "o", color=C[0], label="ended upright")
    ax[0].plot([], [], "X", color=CRITICAL, label="fell")
    ax[0].legend(loc="lower left")
    ax[0].annotate("rubber shoes", (1.6, ax[0].get_ylim()[1]), xytext=(0, -12),
                   textcoords="offset points", color=INK2, fontsize=9)
    ax[0].annotate("no shoes", (9.6, ax[0].get_ylim()[1]), xytext=(0, -12),
                   textcoords="offset points", color=INK2, fontsize=9)
    for a in ax:
        a.axvline(1.5, color=AXIS, lw=1, ls=":")
        a.axvline(9.5, color=AXIS, lw=1, ls=":")
    fig.tight_layout()
    return _save(fig, path)


def floors(results: Dict[str, object], path, t0=2.3, t1=3.3):
    fig, ax = plt.subplots(figsize=(10, 4.8))
    ends = []
    for n, (name, r) in enumerate(results.items()):
        k = (r.t >= t0) & (r.t <= t1)
        col = [C[0], C[1], C[2], C[6]][n]
        ax.plot(r.t[k] * 1000, r.pitch[k], color=col)
        m = r.metrics
        y = r.pitch[k][-1]
        while any(abs(y - e) < 16 for e in ends):          # keep the end labels apart
            y -= 16
        ends.append(y)
        txt = (f"{name}: {m['rot_td']:.0f} deg at touchdown, {'upright' if m['upright'] else 'FELL'}"
               if m["first_touch"] != "none" else f"{name}: the paws slip, it never turns over")
        ax.annotate(txt, (r.t[k][-1] * 1000, y), xytext=(6, 0), textcoords="offset points", color=INK2,
                    fontsize=9, va="center")
    ax.set_xlim(t0 * 1000, t1 * 1000 + 560)
    ax.set_yticks([0, 90, 180, 270, 360])
    ax.set_xlabel("ms since the flip started")
    ax.set_ylabel("body turned, deg")
    ax.set_title("K7 405 g on four floors - a what-if: only the carpet is fitted to real flips")
    fig.tight_layout()
    return _save(fig, path)


def landscape(S, L, E, best, path):
    """E[i, j]: fit error at strength S[i], latency L[j]"""
    fig, ax = plt.subplots(figsize=(8, 4.8))
    from matplotlib.colors import LinearSegmentedColormap
    cmap = LinearSegmentedColormap.from_list("blues_r", BLUES[::-1])
    im = ax.imshow(E.T, origin="lower", aspect="auto", cmap=cmap,
                   extent=[S[0] - (S[1] - S[0]) / 2, S[-1] + (S[1] - S[0]) / 2, L[0] - (L[1] - L[0]) / 2,
                           L[-1] + (L[1] - L[0]) / 2])
    for i, s in enumerate(S):
        for j, l in enumerate(L):
            ax.text(s, l, f"{E[i, j]:.0f}", ha="center", va="center", fontsize=8,
                    color="#ffffff" if E[i, j] < np.percentile(E, 45) else INK)
    ax.plot(*best, "o", ms=16, mfc="none", mec=CRITICAL, mew=2)
    ax.grid(False)
    ax.set_xlabel("servo strength (stall torque x 0.8 N m)")
    ax.set_ylabel("servo latency, ms")
    ax.set_title("System identification: fit error over two of the servo parameters")
    cb = fig.colorbar(im, ax=ax, pad=0.02)
    cb.set_label("fit error (lower = closer to the real flips)")
    cb.outline.set_visible(False)
    fig.tight_layout()
    return _save(fig, path)


def landing(recs: List, sim_before, sims_l2_before: List[Dict], sims_l2_after: List[Dict], volts, path,
            sim_after=None, older: Optional[List] = None):
    """left: body rotation around the touchdown, real (today's strong flips + older ones) vs the sim;
    right: how far it rocks back after the paws touch, K7 405 g vs the landing-tuned table, every model x voltage"""
    fig, ax = plt.subplots(1, 2, figsize=(14, 4.8), gridspec_kw=dict(width_ratios=[1.25, 1]))
    a = ax[0]
    for r in older or []:
        k = (r.t > r.metrics["t_td"] - 0.4) & (r.t < r.metrics["t_td"] + 0.9)
        a.plot((r.t[k] - r.metrics["t_td"]) * 1000, r.pitch[k] - r.metrics["rot_settled"] + 360, color=GRID, lw=1.3,
               label="real, flips 81-83 (weaker battery)" if r is older[0] else None)
    for n, r in enumerate(recs):
        k = (r.t > r.metrics["t_td"] - 0.4) & (r.t < r.metrics["t_td"] + 0.9)
        a.plot((r.t[k] - r.metrics["t_td"]) * 1000, r.pitch[k] - r.metrics["rot_settled"] + 360, color=[INK2, MUTED][n % 2],
               lw=1.6, label=f"real, {r.name} (rocks back {r.metrics['overshoot']:.0f} deg)")
    for s, lab, col, ls in ((sim_before, "sim, K7 405 g", C[0], "--"), (sim_after, "sim, K7 405 L2", C[1], "-")):
        if s is None:
            continue
        m = s.metrics
        k = (s.t > m["t_td"] - 0.4) & (s.t < m["t_td"] + 0.9)
        a.plot((s.t[k] - m["t_td"]) * 1000, s.pitch[k] - m["rot_settled"] + 360, color=col, ls=ls,
               label=f"{lab} (rocks back {m['overshoot']:.0f} deg)")
    a.axhline(360, color=AXIS, lw=0.8)
    a.axvline(0, color=MUTED, lw=1, ls=":")
    a.annotate("front paws touch", (0, 150), xytext=(6, 0), textcoords="offset points", color=INK2, fontsize=9)
    a.set_xlabel("ms from the touchdown")
    a.set_ylabel("body turned, deg (360 = where it settles)")
    a.set_yticks([90, 180, 270, 360, 400])
    a.set_title("The strong flips land, then sit back nose-up")
    a.legend(loc="lower right", fontsize=8.5)
    b = ax[1]
    nv = len(volts)
    per = len(sims_l2_before) // nv
    x = np.arange(nv)
    for k, (outs, nm, col) in enumerate(((sims_l2_before, "K7 405 g", MUTED), (sims_l2_after, "K7 405 L2", C[1]))):
        for i in range(nv):
            grp = outs[i * per:(i + 1) * per]
            xs = x[i] + (k - 0.5) * 0.36 + np.linspace(-0.1, 0.1, per)
            for xx, o in zip(xs, grp):
                b.plot(xx, o["overshoot"], "o" if o["upright"] else "X", color=col if o["upright"] else CRITICAL,
                       ms=7 if o["upright"] else 8, mec=SURFACE, mew=0.8, label=None)
            b.plot([x[i] + (k - 0.5) * 0.36 - 0.14, x[i] + (k - 0.5) * 0.36 + 0.14],
                   [np.mean([o["overshoot"] for o in grp])] * 2, color=col, lw=2)
        b.plot([], [], "o", color=col, label=nm)
    b.plot([], [], "X", color=CRITICAL, label="did not end upright")
    b.set_xticks(x)
    b.set_xticklabels([f"{v:g} V" for v in volts])
    b.set_ylabel("rocks back after the touchdown, deg")
    b.set_title("Sim: 6 fitted models at each battery voltage")
    b.legend(loc="upper left", fontsize=8.5)
    fig.tight_layout()
    return _save(fig, path)
