"""Thin Python wrapper around the ``geopf`` command-line planner.

Passes start, goal, obstacles, and tuning parameters to the ``geopf`` binary
as command-line arguments and reads the resulting trajectory back from stdout
(no JSON input or temporary files required).

Only the standard library is required; if NumPy is installed the trajectory
is returned as an ``(N, 3)`` array, otherwise as a list of ``[x, y, z]``.

Example::

    from geopf import plan

    result = plan(
        start=[0.0, 0.0, 0.5],
        goal=[0.6, 0.0, 0.5],
        obstacles={
            "sphere": [{"pos": [0.3, 0.01, 0.5], "rad": 0.05}],
            "workspace_boundary": {"pos": [0.3, 0.0, 0.5], "rad": 0.6},
        },
        params={"k_attr": 1.0, "k_circ": 0.1},
    )
    print(result.success, result.path_length, result.trajectory.shape)

The binary is looked up in this order: the ``binary`` argument, the
``GEOPF_BIN`` environment variable, ``geopf`` on ``PATH``, ``../build/geopf``
relative to this file.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Mapping, Optional, Sequence

__all__ = ["plan", "plan_files", "build_cli_args", "PlanResult", "GeoPFError", "find_binary"]


class GeoPFError(RuntimeError):
    """Raised when the geopf binary rejects its input (exit status 2)."""


@dataclass
class PlanResult:
    status: str                      # reached_goal | max_iterations | collision | numerical_error
    success: bool
    iterations: int
    path_length: float
    min_obstacle_distance: Optional[float]
    mean_obstacle_distance: Optional[float]
    runtime_us: float
    trajectory: Any                  # (N, 3) numpy array, or list of [x, y, z]
    summary: dict = field(repr=False)  # the raw JSON summary


def find_binary(binary: Optional[str] = None) -> str:
    candidates = [binary, os.environ.get("GEOPF_BIN"), shutil.which("geopf"),
                  str(Path(__file__).resolve().parent.parent / "build" / "geopf")]
    for c in candidates:
        if c and Path(c).is_file() and os.access(c, os.X_OK):
            return c
    raise FileNotFoundError(
        "geopf binary not found; build it (see README) and/or set GEOPF_BIN")


def _run(args: Sequence[str], binary: Optional[str], timeout: Optional[float]) -> PlanResult:
    proc = subprocess.run([find_binary(binary), *args],
                          capture_output=True, text=True, timeout=timeout)
    if proc.returncode not in (0, 1):
        raise GeoPFError(proc.stderr.strip() or f"geopf exited with {proc.returncode}")
    summary = json.loads(proc.stdout)
    traj = summary.pop("trajectory")
    try:
        import numpy as np
        traj = np.asarray(traj, dtype=float).reshape(-1, 3)
    except ImportError:
        pass
    return PlanResult(
        status=summary["status"],
        success=summary["success"],
        iterations=summary["iterations"],
        path_length=summary["path_length"],
        min_obstacle_distance=summary["min_obstacle_distance"],
        mean_obstacle_distance=summary["mean_obstacle_distance"],
        runtime_us=summary["runtime_us"],
        trajectory=traj,
        summary=summary,
    )


def _flatten(x: Any) -> list[float]:
    if hasattr(x, "tolist"):
        x = x.tolist()
    if isinstance(x, (list, tuple)):
        out: list[float] = []
        for item in x:
            out.extend(_flatten(item))
        return out
    return [float(x)]


def _csv(*parts: Any) -> str:
    nums: list[float] = []
    for p in parts:
        nums.extend(_flatten(p))
    return ",".join(f"{v:.12g}" for v in nums)


_OBSTACLE_KEYS = {"pos", "rad", "start", "end", "points", "point1",
                  "obs_start", "obs_end", "obs_rad"}


def _iter_entries(section: Any, sizes: tuple[int, ...]) -> Iterable[Any]:
    """Split an obstacle section into entries.

    ``sizes`` is how many numbers one obstacle has (e.g. 12 for a plane).
    A section holding exactly that many numbers is a single obstacle, whether
    flat, nested (``[[x, y, z], ...]``) or a NumPy array of any shape;
    anything else is a list (or array) of obstacles along its first axis.
    """
    if isinstance(section, Mapping):
        if _OBSTACLE_KEYS & set(section):         # one obstacle given as a dict
            return [section]
        return list(section.values())             # named entries: {"circle1": {...}}
    if hasattr(section, "tolist"):
        section = section.tolist()
    if not isinstance(section, (list, tuple)):
        return [section]
    if not any(isinstance(item, Mapping) for item in section):
        try:
            if len(_flatten(section)) in sizes:
                return [section]
        except (TypeError, ValueError):
            pass
    return [s for s in section if not (isinstance(s, (list, tuple)) and len(s) == 0)]


def _get_entry(d: Mapping[str, Any], *keys: str) -> Any:
    for k in keys:
        if k in d and d[k] is not None:
            return d[k]
    return None


def build_cli_args(
    start: Sequence[float],
    goal: Sequence[float],
    obstacles: Optional[Mapping[str, Any]] = None,
    params: Optional[Mapping[str, Any]] = None,
) -> list[str]:
    """Build the command-line argument list for ``geopf`` from Python data."""
    args: list[str] = ["--start", _csv(start), "--goal", _csv(goal)]
    obs = dict(obstacles or {})

    wb = _get_entry(obs, "workspace_boundary", "worksapce_boundary")
    if wb is not None:
        if isinstance(wb, Mapping):
            args += ["--boundary", _csv(wb["pos"], wb["rad"])]
        else:
            args += ["--boundary", _csv(wb)]

    spheres = _get_entry(obs, "sphere", "circle")
    if spheres is not None:
        for s in _iter_entries(spheres, (3, 4)):
            if isinstance(s, Mapping):
                if "rad" in s:
                    args += ["--sphere", _csv(s["pos"], s["rad"])]
                else:
                    args += ["--sphere", _csv(s["pos"])]
            else:
                args += ["--sphere", _csv(s)]

    lines = _get_entry(obs, "line")
    if lines is not None:
        for ln in _iter_entries(lines, (6,)):
            if isinstance(ln, Mapping):
                args += ["--line", _csv(ln["start"], ln["end"])]
            else:
                args += ["--line", _csv(ln)]

    planes = _get_entry(obs, "plane")
    if planes is not None:
        for pl in _iter_entries(planes, (12,)):
            if isinstance(pl, Mapping):
                pts = pl.get("points")
                if pts is None:
                    pts = [pl[f"point{i}"] for i in range(1, 5)]
                args += ["--plane", _csv(pts)]
            else:
                args += ["--plane", _csv(pl)]

    cubics = _get_entry(obs, "cubic", "cuboid")
    if cubics is not None:
        for cb in _iter_entries(cubics, (24,)):
            if isinstance(cb, Mapping):
                pts = cb.get("points")
                if pts is None:
                    pts = [cb[f"point{i}"] for i in range(1, 9)]
                args += ["--cubic", _csv(pts)]
            else:
                args += ["--cubic", _csv(cb)]

    cylinders = _get_entry(obs, "cylinder")
    if cylinders is not None:
        for cy in _iter_entries(cylinders, (7,)):
            if isinstance(cy, Mapping):
                st = cy.get("start") if "start" in cy else cy.get("obs_start")
                en = cy.get("end") if "end" in cy else cy.get("obs_end")
                rd = cy.get("rad") if "rad" in cy else cy.get("obs_rad")
                args += ["--cylinder", _csv(st, en, rd)]
            else:
                args += ["--cylinder", _csv(cy)]

    for key, val in dict(params or {}).items():
        if key == "limit_iter":
            key = "max_iter"
        flag = "--" + key.replace("_", "-")
        v = _flatten(val)[0]
        args += [flag, str(int(v) if key == "max_iter" else v)]

    return args


def plan(start: Sequence[float], goal: Sequence[float],
         obstacles: Optional[Mapping[str, Any]] = None,
         params: Optional[Mapping[str, Any]] = None,
         binary: Optional[str] = None,
         timeout: Optional[float] = None) -> PlanResult:
    """Plan from ``start`` to ``goal`` by passing arguments to the ``geopf`` CLI.

    ``obstacles`` accepts ``workspace_boundary``, ``sphere``, ``line``,
    ``plane``, ``cubic``, and ``cylinder`` (either as dicts or flat coordinate
    sequences); ``params`` accepts ``k_attr``, ``k_circ``, ``dt``, ``max_iter``,
    etc. NumPy arrays are accepted anywhere.
    """
    try:
        args = build_cli_args(start, goal, obstacles, params)
    except (KeyError, TypeError, ValueError, IndexError) as exc:
        raise GeoPFError(f"invalid scene argument: {exc}") from exc
    return _run(args, binary, timeout)


def plan_files(*config_files: str, binary: Optional[str] = None,
               timeout: Optional[float] = None, **overrides: Any) -> PlanResult:
    """Plan from existing YAML/JSON files, e.g. ``plan_files("scene.yaml",
    "params.yaml", k_attr=0.5, goal=[0.6, 0, 0.5])``."""
    args: list[str] = []
    for c in config_files:
        args += ["-c", str(c)]
    for key, value in overrides.items():
        flag = "--" + key.replace("_", "-")
        if key in ("start", "goal"):
            value = _csv(value)
        args += [flag, str(value)]
    return _run(args, binary, timeout)


if __name__ == "__main__":
    import sys
    res = plan_files(*sys.argv[1:])
    print(res)
