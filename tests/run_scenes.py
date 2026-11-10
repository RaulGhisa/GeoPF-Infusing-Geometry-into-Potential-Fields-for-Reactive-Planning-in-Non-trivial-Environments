"""Run every scene in scenes.py through the geopf CLI and verify the result
with independent geometry.

    uv run --no-project --with numpy --with matplotlib python tests/run_scenes.py [--plot] [--random N]

Writes (all under tests/out/, which git ignores):
  scenes/<name>.json   scene files usable with `geopf -c`
  <name>.csv           planned trajectories
  <name>.png           3D plots (with --plot)
  report.md            summary table

Exits with status 1 if any scene's verdict differs from the expected one
(PASS unless scenes.EXPECTED says otherwise).
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
from geometry import verify  # noqa: E402
from scenes import EXPECTED, SCENES, box  # noqa: E402

BIN = Path(os.environ.get("GEOPF_BIN", ROOT / "build" / "geopf"))
OUT = HERE / "out"
SCENE_DIR = OUT / "scenes"


def run(scene, params, max_iter=30000):
    args = [str(BIN), "--scene", json.dumps(scene), "--max-iter", str(max_iter)]
    for k, v in params.items():
        args += ["--" + k.replace("_", "-"), str(v)]
    p = subprocess.run(args, capture_output=True, text=True, timeout=120)
    if p.returncode not in (0, 1):
        return {"status": "cli_error:" + p.stderr.strip()}, np.zeros((0, 3))
    summ = json.loads(p.stdout)
    traj = np.array([[np.nan if c is None else c for c in r] for r in summ.pop("trajectory")],
                    dtype=float).reshape(-1, 3)
    return summ, traj


def classify(summ, ver):
    rep = summ["status"]
    really_ok = not ver["nan"] and not ver["penetrated"] and not ver["crossed"]
    if rep == "reached_goal" and really_ok:
        return "PASS"
    if rep == "reached_goal":
        return "FALSE_SUCCESS"          # planner says success but path hits obstacle
    if rep == "collision" and really_ok:
        return "FALSE_COLLISION"
    if rep == "collision":
        return "COLLISION"              # reported and confirmed
    return "FAIL"


def plot(name, desc, scene, traj, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(6, 5))
    ax = fig.add_subplot(projection="3d")
    for s in scene.get("sphere", []):
        u, v = np.mgrid[0:2 * np.pi:20j, 0:np.pi:10j]
        c, r = s["pos"], s["rad"]
        ax.plot_wireframe(c[0] + r * np.cos(u) * np.sin(v), c[1] + r * np.sin(u) * np.sin(v),
                          c[2] + r * np.cos(v), color="r", lw=0.4)
    for pl in scene.get("plane", []):
        ax.add_collection3d(Poly3DCollection([pl["points"]], alpha=0.3, color="tab:orange"))
    for cb in scene.get("cubic", []):
        p = np.array(cb["points"])
        faces = [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]
        ax.add_collection3d(Poly3DCollection([p[f] for f in faces], alpha=0.25, color="tab:blue"))
    for cy in scene.get("cylinder", []):
        a, b, r = np.array(cy["start"]), np.array(cy["end"]), cy["rad"]
        axis = (b - a) / np.linalg.norm(b - a)
        n1 = np.cross(axis, [1, 0, 0] if abs(axis[0]) < 0.9 else [0, 1, 0])
        n1 /= np.linalg.norm(n1)
        n2 = np.cross(axis, n1)
        th, t = np.meshgrid(np.linspace(0, 2 * np.pi, 24), np.linspace(0, 1, 2))
        P = a[:, None, None] + (b - a)[:, None, None] * t + r * (
            n1[:, None, None] * np.cos(th) + n2[:, None, None] * np.sin(th))
        ax.plot_surface(*P, alpha=0.3, color="tab:green")
    for ln in scene.get("line", []):
        ax.plot(*np.array([ln["start"], ln["end"]]).T, "k-", lw=3)
    good = traj[np.isfinite(traj).all(axis=1)]
    if len(good):
        ax.plot(*good.T, "b-", lw=1.2)
    ax.scatter(*scene["start"], c="g", s=30, label="start")
    ax.scatter(*scene["goal"], c="m", marker="*", s=80, label="goal")
    pts = np.vstack([scene["start"], scene["goal"], good if len(good) else np.empty((0, 3))])
    c, r = pts.mean(0), max(np.ptp(pts, 0).max() / 2, 0.1)
    ax.set(xlim=(c[0] - r, c[0] + r), ylim=(c[1] - r, c[1] + r), zlim=(c[2] - r, c[2] + r),
           xlabel="x", ylabel="y", zlabel="z")
    ax.set_title(f"{name}\n{desc}", fontsize=8)
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def random_scenes(n, seed=0):
    rng = np.random.default_rng(seed)
    for i in range(n):
        start, goal = np.array([0.0, 0, 0.5]), np.array([0.6, 0, 0.5])
        scene = {"start": start.tolist(), "goal": goal.tolist(), "sphere": [], "cubic": []}
        for _ in range(rng.integers(1, 4)):
            c = start + rng.uniform(0.2, 0.8) * (goal - start) + rng.normal(0, 0.04, 3)
            if rng.random() < 0.5:
                r = rng.uniform(0.02, 0.08)
                if np.linalg.norm(c - start) > r + 0.03 and np.linalg.norm(c - goal) > r + 0.03:
                    scene["sphere"].append({"pos": c.tolist(), "rad": float(r)})
            else:
                sz = rng.uniform(0.03, 0.12, 3)
                if min(np.linalg.norm(c - start), np.linalg.norm(c - goal)) > sz.max() + 0.03:
                    scene["cubic"].append(box(c, sz))
        yield f"random_{i:03d}", scene


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--random", type=int, default=0)
    ap.add_argument("--only", default="")
    a = ap.parse_args()
    OUT.mkdir(exist_ok=True)
    SCENE_DIR.mkdir(parents=True, exist_ok=True)

    rows = []
    mismatches = []
    for name, (desc, scene, params) in SCENES.items():
        if a.only and a.only not in name:
            continue
        (SCENE_DIR / f"{name}.json").write_text(json.dumps(scene, indent=1))
        summ, traj = run(scene, params)
        ver = verify(traj, scene) if len(traj) else {"nan": False, "penetrated": [], "crossed": [],
                                                       "min_dist": {}, "final_err": None}
        np.savetxt(OUT / f"{name}.csv", traj, delimiter=",", fmt="%.9g")
        verdict = classify(summ, ver) if "iterations" in summ else "CLI_ERROR"
        expected = EXPECTED.get(name, "PASS")
        if verdict != expected:
            mismatches.append(name)
        obstacle_d = [v for k, v in ver["min_dist"].items() if k != "boundary"]
        md = min(obstacle_d) if obstacle_d else float("nan")
        rows.append((name, verdict, expected, summ["status"], summ.get("iterations"), md,
                     summ.get("min_obstacle_distance"), ver["final_err"],
                     ",".join(ver["penetrated"] + ver["crossed"]) or "-", desc))
        if a.plot:
            plot(name, desc, scene, traj, OUT / f"{name}.png")

    hdr = ("scene", "verdict", "expected", "reported", "iters", "true_min_d", "reported_min_d",
           "final_err", "hit", "description")
    fmt = lambda v: f"{v:.4g}" if isinstance(v, float) else str(v)
    lines = ["| " + " | ".join(hdr) + " |", "|" + "---|" * len(hdr)]
    lines += ["| " + " | ".join(fmt(v) for v in r) + " |" for r in rows]

    if a.random:
        stats = {}
        for name, scene in random_scenes(a.random):
            summ, traj = run(scene, {}, max_iter=20000)
            v = classify(summ, verify(traj, scene)) if "iterations" in summ else "CLI_ERROR"
            key = (v, summ["status"])
            stats[key] = stats.get(key, 0) + 1
        lines += ["", f"### Random stress test ({a.random} scenes, 1-3 spheres/boxes near the line)",
                  "", "| verdict | reported status | count |", "|---|---|---|"]
        lines += [f"| {k[0]} | {k[1]} | {c} |" for k, c in sorted(stats.items())]

    if mismatches:
        lines += ["", "**Unexpected verdicts:** " + ", ".join(mismatches)]
    report = "\n".join(lines)
    (OUT / "report.md").write_text(report + "\n")
    print(report)
    return 1 if mismatches else 0


if __name__ == "__main__":
    sys.exit(main())
