"""Thin Python wrapper around the ``geopf`` command-line planner.

Only the standard library is required; if NumPy is installed the trajectory
is returned as an ``(N, 3)`` array, otherwise as a list of ``[x, y, z]``.

Example::

    from geopf import plan

    result = plan(
        start=[0.0, 0.0, 0.5],
        goal=[0.6, 0.0, 0.5],
        obstacles={
            "sphere": [{"pos": [0.3, 0.0, 0.5], "rad": 0.05}],
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
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence

__all__ = ["plan", "plan_files", "PlanResult", "GeoPFError", "find_binary"]


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
    proc = subprocess.run([find_binary(binary), *args, "--include-trajectory"],
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


def _tolist(x: Any) -> Any:
    """Convert numpy arrays / scalars (possibly nested) to plain JSON types."""
    if hasattr(x, "tolist"):
        return x.tolist()
    if isinstance(x, Mapping):
        return {k: _tolist(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_tolist(v) for v in x]
    return x


def plan(start: Sequence[float], goal: Sequence[float],
         obstacles: Optional[Mapping[str, Any]] = None,
         params: Optional[Mapping[str, Any]] = None,
         binary: Optional[str] = None,
         timeout: Optional[float] = None) -> PlanResult:
    """Plan from ``start`` to ``goal``.

    ``obstacles`` uses the same keys as the YAML scene format
    (``workspace_boundary``, ``sphere``, ``line``, ``plane``, ``cubic``,
    ``cylinder``); ``params`` the same keys as params.yaml (``k_attr``,
    ``dt``, ``max_iter``, ...). NumPy arrays are accepted anywhere.
    """
    config = {"start": _tolist(start), "goal": _tolist(goal),
              **_tolist(dict(obstacles or {})), **_tolist(dict(params or {}))}
    # JSON is valid YAML, so the binary reads it directly.
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
        json.dump(config, f)
        path = f.name
    try:
        return _run(["-c", path], binary, timeout)
    finally:
        os.unlink(path)


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
            value = ",".join(str(v) for v in _tolist(value))
        args += [flag, str(value)]
    return _run(args, binary, timeout)


if __name__ == "__main__":
    import sys
    res = plan_files(*sys.argv[1:])
    print(res)
