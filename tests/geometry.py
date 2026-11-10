"""Independent exact geometry for verifying GeoPF trajectories.

Nothing here reuses the C++ distance code: every primitive gets a textbook
distance function (signed where the primitive has an inside, negative =
penetrating) so that the planner's own distance bookkeeping can be
cross-checked.

The trajectory is checked as a polyline, not only at its samples: lines and
spheres are measured against every step segment exactly, rectangles are
tested for crossings, and boxes / cylinders are checked on a trajectory
resampled finely enough that no step can jump over them.
"""

from __future__ import annotations

import itertools

import numpy as np


def _v(x):
    return np.asarray(x, dtype=float)


# ------------------------------------------------------------ primitives ---

def sd_sphere(p, pos, rad):
    return np.linalg.norm(p - _v(pos), axis=-1) - rad


def d_segment(p, a, b):
    """Distance from points p to the segment a-b."""
    a, b = _v(a), _v(b)
    ab = b - a
    t = np.clip(((p - a) @ ab) / (ab @ ab), 0.0, 1.0)
    return np.linalg.norm(p - (a + t[..., None] * ab), axis=-1)


def d_point_to_steps(q, traj):
    """Distance from point q to each step segment traj[k] -> traj[k+1]."""
    a, d = traj[:-1], traj[1:] - traj[:-1]
    dd = np.einsum("ij,ij->i", d, d)
    t = np.where(dd > 0, np.einsum("ij,ij->i", _v(q) - a, d) / np.where(dd > 0, dd, 1), 0.0)
    t = np.clip(t, 0.0, 1.0)
    return np.linalg.norm(a + t[:, None] * d - _v(q), axis=-1)


def d_segment_to_steps(q0, q1, traj):
    """Distance from segment q0-q1 to each step segment (closest points of
    two segments, clamped; Ericson 5.1.9)."""
    q0, q1 = _v(q0), _v(q1)
    p0, d1 = traj[:-1], traj[1:] - traj[:-1]
    d2 = q1 - q0
    r = p0 - q0
    a = np.einsum("ij,ij->i", d1, d1)
    e = d2 @ d2
    f = r @ d2
    c = np.einsum("ij,ij->i", d1, r)
    b = d1 @ d2
    denom = a * e - b * b
    safe_a = np.where(a > 0, a, 1)
    s = np.where(denom > 1e-12 * a * e, np.clip((b * f - c * e) / np.where(denom > 0, denom, 1), 0, 1), 0.0)
    t = (b * s + f) / e
    s = np.where(t < 0, np.clip(-c / safe_a, 0, 1), np.where(t > 1, np.clip((b - c) / safe_a, 0, 1), s))
    t = np.clip(t, 0, 1)
    s = np.where(a > 0, s, 0.0)
    return np.linalg.norm(p0 + s[:, None] * d1 - (q0 + t[:, None] * d2), axis=-1)


def plane_frame(pts):
    """Corners of a planar quad sorted counter-clockwise around its unit
    normal, whatever order they were given in (around the edge, Z order...)."""
    pts = _v(pts)
    c = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - c)
    u, n = vt[0], vt[2]
    w = np.cross(n, u)
    order = np.argsort(np.arctan2((pts - c) @ w, (pts - c) @ u))
    return pts[order], n


def _inside_quad(x, pts, n):
    """x: points in the plane of the sorted quad pts."""
    inside = np.ones(len(x), dtype=bool)
    for i in range(4):
        a, b = pts[i], pts[(i + 1) % 4]
        inside &= (np.cross(b - a, x - a) @ n) >= -1e-9 * ((b - a) @ (b - a))
    return inside


def d_rectangle(p, pts):
    """Unsigned distance to a planar convex quad (corners in any order)."""
    pts, n = plane_frame(pts)
    h = (p - pts[0]) @ n                         # signed height above plane
    inside = _inside_quad(p - h[..., None] * n, pts, n)
    edge = np.min([d_segment(p, pts[i], pts[(i + 1) % 4]) for i in range(4)], axis=0)
    return np.where(inside, np.abs(h), edge)


def rectangle_crossings(traj, pts):
    """Indices k where segment traj[k]->traj[k+1] passes through the quad."""
    pts, n = plane_frame(pts)
    h = (traj - pts[0]) @ n
    idx = np.nonzero(((h[:-1] > 0) & (h[1:] < 0)) | ((h[:-1] < 0) & (h[1:] > 0)))[0]
    if not len(idx):
        return []
    t = h[idx] / (h[idx] - h[idx + 1])
    x = traj[idx] + t[:, None] * (traj[idx + 1] - traj[idx])
    return [int(k) for k in idx[_inside_quad(x, pts, n)]]


def box_frame(pts):
    """Centre, edge vectors and half sizes of a rectangular box given by its 8
    corners in any order."""
    pts = _v(pts)
    d = pts[1:] - pts[0]
    tol = 1e-4 * np.linalg.norm(d, axis=1).max()
    for i, j, k in itertools.combinations(range(7), 3):
        e = d[[i, j, k]]
        ln = np.linalg.norm(e, axis=1)
        if (ln <= tol).any():
            continue
        cos = np.abs(e @ e.T) / np.outer(ln, ln)
        if (cos[np.triu_indices(3, 1)] > 1e-4).any():
            continue
        gen = np.array([pts[0] + np.array(m) @ e for m in itertools.product((0, 1), repeat=3)])
        if all(np.linalg.norm(pts - g, axis=1).min() <= tol for g in gen):
            return pts.mean(axis=0), e / ln[:, None], ln / 2
    raise ValueError("cubic corners do not form a rectangular box")


def sd_box(p, pts):
    """Signed distance to the box given by its 8 corners (any order)."""
    c, axes, half = box_frame(pts)
    q = np.abs((p - c) @ axes.T) - half
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


def resample(traj, spacing):
    """Insert points so that consecutive points are at most `spacing` apart."""
    if len(traj) < 2:
        return traj
    seg = np.linalg.norm(np.diff(traj, axis=0), axis=1)
    n = np.maximum(1, np.ceil(seg / spacing).astype(int))
    if n.sum() > 5_000_000:
        raise ValueError("trajectory too long to resample; increase spacing")
    t = np.concatenate([np.arange(k) / k for k in n])
    i = np.repeat(np.arange(len(seg)), n)
    out = traj[i] + t[:, None] * (traj[i + 1] - traj[i])
    return np.vstack([out, traj[-1:]])


# ----------------------------------------------------------------- scene ---

def _volume_spacing(scene, traj):
    """A sample spacing small enough that no box or cylinder can be skipped."""
    sizes = []
    for cb in scene.get("cubic", []):
        sizes += list(2 * box_frame(cb["points"])[2])
    for cy in scene.get("cylinder", []):
        sizes += [2 * cy["rad"], np.linalg.norm(_v(cy["end"]) - _v(cy["start"]))]
    if not sizes:
        return None
    extent = max(np.ptp(traj, axis=0).max(), 1e-12)
    return max(min(sizes) / 4, extent * 1e-6)


def obstacle_distances(traj, scene):
    """Dict label -> distances along the trajectory (signed where the
    primitive has an inside)."""
    out = {}
    if len(traj) < 2:
        traj = np.vstack([traj, traj])           # a single point: one zero-length step
    for i, s in enumerate(scene.get("sphere", [])):
        out[f"sphere[{i}]"] = d_point_to_steps(s["pos"], traj) - s["rad"]
    for i, l in enumerate(scene.get("line", [])):
        out[f"line[{i}]"] = d_segment_to_steps(l["start"], l["end"], traj)
    for i, pl in enumerate(scene.get("plane", [])):
        out[f"plane[{i}]"] = d_rectangle(traj, pl["points"])
    spacing = _volume_spacing(scene, traj)
    if spacing is not None:
        dense = resample(traj, spacing)
        for i, cb in enumerate(scene.get("cubic", [])):
            out[f"cubic[{i}]"] = sd_box(dense, cb["points"])
        for i, cy in enumerate(scene.get("cylinder", [])):
            out[f"cylinder[{i}]"] = sd_cylinder(dense, cy["start"], cy["end"], cy["rad"])
    return out


def boundary_violation(traj, scene):
    """True if the trajectory leaves the workspace boundary after having been
    inside it (starting outside and moving in is not a violation)."""
    wb = scene.get("workspace_boundary")
    if not wb:
        return False
    inside = np.linalg.norm(traj - _v(wb["pos"]), axis=-1) <= wb["rad"]
    first = np.argmax(inside) if inside.any() else len(inside)
    return bool((~inside[first:]).any()) or not inside.any()


def verify(traj, scene):
    """Independent verdict on a trajectory."""
    traj = np.asarray(traj, dtype=float).reshape(-1, 3)
    full = np.vstack([_v(scene["start"]), traj]) if len(traj) else _v(scene["start"])[None]
    res = {"nan": bool(~np.isfinite(full).all())}
    finite = full[np.isfinite(full).all(axis=1)]
    d = obstacle_distances(finite, scene) if len(finite) else {}
    extent = max(np.ptp(finite, axis=0).max(), 1.0) if len(finite) else 1.0
    res["min_dist"] = {k: float(v.min()) for k, v in d.items()}
    # Signed distances: inside is < 0. Lines have no inside: touching counts.
    res["penetrated"] = sorted(
        k for k, v in d.items()
        if v.min() < (1e-9 * extent if k.startswith("line") else 0))
    wb = scene.get("workspace_boundary")
    if wb and len(finite):
        res["min_dist"]["boundary"] = float(
            (wb["rad"] - np.linalg.norm(finite - _v(wb["pos"]), axis=-1)).min())
        if boundary_violation(finite, scene):
            res["penetrated"].append("boundary")
    res["crossed"] = sorted(f"plane[{i}]" for i, pl in enumerate(scene.get("plane", []))
                            if rectangle_crossings(finite, pl["points"]))
    res["final_err"] = float(np.linalg.norm(finite[-1] - _v(scene["goal"]))) if len(finite) else None
    return res
