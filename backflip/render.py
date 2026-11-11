"""Pictures of a simulated flip: slow-motion video / GIF, a strip of snapshots, and a chronophotograph (every pose
of the flip ghosted into one picture).  Renders offscreen; on a headless Linux box set MUJOCO_GL=egl."""
from __future__ import annotations

import subprocess
from pathlib import Path
from typing import List, Optional, Sequence

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .robot import Robot
from .sim import FlipResult


def _font(size):
    for name in ("DejaVuSans.ttf", "Arial.ttf", "arial.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default()


class Camera:
    def __init__(self, robot: Robot, width=640, height=480, azimuth=150.0, elevation=-12.0, distance=0.5,
                 lookat=(0.0, 0.0, 0.1), track=True, shadows=True):
        m = robot.m
        m.vis.global_.offwidth = max(width, m.vis.global_.offwidth)
        m.vis.global_.offheight = max(height, m.vis.global_.offheight)
        self.robot, self.track = robot, track
        self.r = mujoco.Renderer(m, height, width)
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.cam.azimuth, self.cam.elevation, self.cam.distance = azimuth, elevation, distance
        self.cam.lookat[:] = lookat
        self.opt = mujoco.MjvOption()
        self.shadows = shadows

    def shot(self, qpos) -> np.ndarray:
        m, d = self.robot.m, self.robot.d
        d.qpos[:] = qpos
        mujoco.mj_forward(m, d)
        if self.track:
            self.cam.lookat[0] = 0.85 * self.cam.lookat[0] + 0.15 * float(qpos[0])
        self.r.update_scene(d, self.cam, self.opt)
        self.r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = self.shadows
        return self.r.render().copy()

    def close(self):
        self.r.close()


def _label(img: np.ndarray, lines: Sequence[str], size=18) -> np.ndarray:
    im = Image.fromarray(img)
    dr = ImageDraw.Draw(im)
    f = _font(size)
    y = 8
    for ln in lines:
        dr.text((10, y), ln, fill=(25, 25, 30), font=f)
        y += size + 4
    return np.asarray(im)


def video(results: List[FlipResult], labels: List[str], path, slow: float = 4.0, t0: float = 2.25,
          t1: float = 3.6, width=640, height=480, fps=50, **cam):
    """side-by-side slow-motion video (mp4 through ffmpeg, or .gif)"""
    rb = Robot()
    cam.setdefault("shadows", False)                 # the shadow map streaks at this scale; off reads cleaner
    cams = [Camera(rb, width, height, **cam) for _ in results]
    t = results[0].t
    ks = np.where((t >= t0) & (t <= t1))[0]
    every = max(1, int(round(1000.0 / (fps * slow * 5))))        # sim frames per video frame
    frames = []
    for k in ks[::every]:
        row = []
        for r, c, lb in zip(results, cams, labels):
            j = min(k, len(r.t) - 1)
            row.append(_label(c.shot(r.qpos[j]), [lb, f"t = {1000 * r.t[j]:4.0f} ms   turned {r.pitch[j]:4.0f} deg",
                                                  f"{slow:g}x slower"]))
        frames.append(np.concatenate(row, axis=1))
    for c in cams:
        c.close()
    path = Path(path)
    if path.suffix == ".gif":
        ims = [Image.fromarray(f).convert("P", palette=Image.ADAPTIVE, colors=128) for f in frames]
        ims[0].save(path, save_all=True, append_images=ims[1:], duration=int(1000 / fps), loop=0, optimize=True)
    else:
        h, w = frames[0].shape[:2]
        p = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s",
                              f"{w}x{h}", "-r", str(fps), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p",
                              "-crf", "22", str(path)], stdin=subprocess.PIPE)
        for f in frames:
            p.stdin.write(f.tobytes())
        p.stdin.close()
        p.wait()
    return path


def strip(result: FlipResult, times_s: Sequence[float], path, width=300, height=260, **cam):
    """one snapshot per time, side by side, each labelled"""
    rb = Robot()
    c = Camera(rb, width, height, track=False, shadows=False, **cam)
    tiles = []
    for ts in times_s:
        k = int(np.argmin(np.abs(result.t - ts)))
        c.cam.lookat[0] = float(result.qpos[k][0])
        tiles.append(_label(c.shot(result.qpos[k]), [f"{1000 * ts:.0f} ms", f"{result.pitch[k]:.0f} deg"], 16))
    c.close()
    Image.fromarray(np.concatenate(tiles, axis=1)).save(path)
    return path


def chronophoto(result: FlipResult, times_s: Sequence[float], path, width=1400, height=560, spread=0.0,
                azimuth=90.0, elevation=-4.0, distance=0.62, lookat=None, title: Optional[str] = None,
                crop: bool = True):
    """every pose of the flip in one picture (a chronophotograph).  spread > 0 moves pose n by n * spread metres
    sideways so the poses do not overlap (a backflip lands almost where it took off); later poses are more solid."""
    rb = Robot()
    ks = [int(np.argmin(np.abs(result.t - ts))) for ts in times_s]
    poses = []
    for n, k in enumerate(ks):
        q = result.qpos[k].copy()
        if spread:                                   # even spacing: the flip's own travel is taken out
            q[0] = result.qpos[ks[0]][0] + n * spread
        poses.append(q)
    xs = [float(q[0]) for q in poses]
    look = lookat or ((min(xs) + max(xs)) / 2, 0.0, 0.11)
    c = Camera(rb, width, height, azimuth=azimuth, elevation=elevation, distance=distance, lookat=look, track=False,
               shadows=False)
    refl = rb.m.mat_reflectance.copy()
    rb.m.mat_reflectance[:] = 0.0                    # no floor reflections: they would leak into the masks
    away = poses[0].copy()
    away[2] = -5.0                                   # robot out of view: the empty background
    bg = c.shot(away).astype(float)
    out = bg.copy()
    union = np.zeros(bg.shape[:2], bool)
    for n, q in enumerate(poses):
        img = c.shot(q).astype(float)
        mask = (np.abs(img - bg).sum(axis=2) > 18)[..., None]
        union |= mask[..., 0]
        alpha = 1.0 if spread else 0.35 + 0.65 * (n + 1) / len(ks)
        out = np.where(mask, (1 - alpha) * out + alpha * img, out)
    c.close()
    rb.m.mat_reflectance[:] = refl
    if crop:
        ys, xs_ = np.where(union)
        y0, y1 = max(0, ys.min() - 70), min(height, ys.max() + 50)
        x0, x1 = max(0, xs_.min() - 60), min(width, xs_.max() + 60)
        out = out[y0:y1, x0:x1]
    im = Image.fromarray(out.clip(0, 255).astype(np.uint8))
    if title:
        ImageDraw.Draw(im).text((16, 12), title, fill=(25, 25, 30), font=_font(26))
    im.save(path)
    return path
