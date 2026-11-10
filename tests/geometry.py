"""Independent exact geometry for verifying GeoPF trajectories.

Nothing here reuses the C++ distance code: every primitive gets a textbook
signed-distance function (negative = inside / penetrating) so that the
planner's own (possibly buggy) distance bookkeeping can be cross-checked.
"""

from __future__ import annotations

import numpy as np


def _v(x):
    return np.asarray(x, dtype=float)


# ------------------------------------------------------------ primitives ---

def sd_sphere(p, pos, rad):
    return np.linalg.norm(p - _v(pos), axis=-1) - rad


def d_segment(p, a, b):
    a, b = _v(a), _v(b)
    ab = b - a
    t = np.clip(((p - a) @ ab) / (ab @ ab), 0.0, 1.0)
    return np.linalg.norm(p - (a + t[..., None] * ab), axis=-1)


def _rect_frame(pts):
    pts = _v(pts)
    n = np.cross(pts[1] - pts[0], pts[3] - pts[0])
    n /= np.linalg.norm(n)
    return pts, n


def d_rectangle(p, pts):
    """Unsigned distance to a planar convex quad (corners in order)."""
    pts, n = _rect_frame(pts)
    h = (p - pts[0]) @ n                         # signed height above plane
    proj = p - h[..., None] * n
    inside = np.ones(len(p), dtype=bool)
    for i in range(4):
        a, b = pts[i], pts[(i + 1) % 4]
        inside &= ((np.cross(b - a, proj - a)) @ n) >= 0
    if not inside.any():                         # orientation may be CW
        inside = np.ones(len(p), dtype=bool)
        for i in range(4):
            a, b = pts[i], pts[(i + 1) % 4]
            inside &= ((np.cross(b - a, proj - a)) @ n) <= 0
    edge = np.min([d_segment(p, pts[i], pts[(i + 1) % 4]) for i in range(4)], axis=0)
    return np.where(inside, np.abs(h), edge)


def rectangle_crossings(traj, pts):
    """Indices k where segment traj[k]->traj[k+1] passes through the quad."""
    pts, n = _rect_frame(pts)
    h = (traj - pts[0]) @ n
    idx = np.nonzero(np.sign(h[:-1]) * np.sign(h[1:]) < 0)[0]
    hits = []
    for k in idx:
        t = h[k] / (h[k] - h[k + 1])
        x = traj[k] + t * (traj[k + 1] - traj[k])
        if d_rectangle(x[None], pts)[0] < 1e-9:
            hits.append(int(k))
    return hits


def sd_box(p, pts):
    """Signed distance to the box given by 8 corners (GeoPF ordering:
    p1-p4 one face, p5-p8 opposite, p5 above p1). Assumes a rectangular box."""
    pts = _v(pts)
    c = pts.mean(axis=0)
    axes = [pts[1] - pts[0], pts[3] - pts[0], pts[4] - pts[0]]
    half = np.array([np.linalg.norm(a) / 2 for a in axes])
    R = np.stack([a / np.linalg.norm(a) for a in axes])
    q = np.abs((p - c) @ R.T) - half
    outside = np.linalg.norm(np.maximum(q, 0), axis=-1)
    inside = np.minimum(q.max(axis=-1), 0)
    return outside + inside


def sd_cylinder(p, a, b, rad):
    a, b = _v(a), _v(b)
    ax = b - a
    L = np.linalg.norm(ax)
    ax /= L
    t = (p - a) @ ax
    radial = np.linalg.norm((p - a) - t[..., None] * ax, axis=-1)
    dr = radial - rad
    dz = np.abs(t - L / 2) - L / 2
    outside = np.hypot(np.maximum(dr, 0), np.maximum(dz, 0))
    inside = np.minimum(np.maximum(dr, dz), 0)
    return outside + inside


# ----------------------------------------------------------------- scene ---

def obstacle_distances(traj, scene):
    """Dict label -> per-sample signed distance array."""
    out = {}
    for i, s in enumerate(scene.get("sphere", [])):
        out[f"sphere[{i}]"] = sd_sphere(traj, s["pos"], s["rad"])
    for i, l in enumerate(scene.get("line", [])):
        out[f"line[{i}]"] = d_segment(traj, l["start"], l["end"])
    for i, pl in enumerate(scene.get("plane", [])):
        out[f"plane[{i}]"] = d_rectangle(traj, pl["points"])
    for i, cb in enumerate(scene.get("cubic", [])):
        out[f"cubic[{i}]"] = sd_box(traj, cb["points"])
    for i, cy in enumerate(scene.get("cylinder", [])):
        out[f"cylinder[{i}]"] = sd_cylinder(traj, cy["start"], cy["end"], cy["rad"])
    wb = scene.get("workspace_boundary")
    if wb:
        out["boundary"] = wb["rad"] - np.linalg.norm(traj - _v(wb["pos"]), axis=-1)
    return out


def verify(traj, scene):
    """Independent verdict on a trajectory."""
    traj = np.asarray(traj, dtype=float).reshape(-1, 3)
    full = np.vstack([_v(scene["start"]), traj]) if len(traj) else _v(scene["start"])[None]
    res = {"nan": bool(~np.isfinite(full).all())}
    finite = full[np.isfinite(full).all(axis=1)]
    d = obstacle_distances(finite, scene) if len(finite) else {}
    res["min_dist"] = {k: float(v.min()) for k, v in d.items()}
    res["penetrated"] = sorted(k for k, v in d.items() if v.min() < 0)
    res["crossed"] = sorted(f"plane[{i}]" for i, pl in enumerate(scene.get("plane", []))
                            if rectangle_crossings(finite, pl["points"]))
    res["final_err"] = float(np.linalg.norm(finite[-1] - _v(scene["goal"]))) if len(finite) else None
    return res
