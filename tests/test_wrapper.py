"""Checks that python/geopf.py turns every supported obstacle shape into the
right CLI arguments.

    uv run --no-project --with numpy --with pytest pytest tests/test_wrapper.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "python"))
from geopf import build_cli_args  # noqa: E402

S, G = [0, 0, 0.5], [0.6, 0, 0.5]
CORNERS = [[0.3, -0.1, 0.4], [0.3, 0.1, 0.4], [0.3, 0.1, 0.6], [0.3, -0.1, 0.6]]
BOX = [[x, y, z] for x in (0.25, 0.35) for y in (-0.05, 0.05) for z in (0.45, 0.55)]


def flags(obstacles, name):
    args = build_cli_args(S, G, obstacles)
    return [args[i + 1] for i, a in enumerate(args) if a == name]


def count(value):
    return len(value.split(","))


def test_one_plane_in_any_shape():
    for plane in (CORNERS, np.array(CORNERS), np.array(CORNERS).ravel(),
                  [CORNERS], np.array([CORNERS]), {"points": CORNERS}):
        out = flags({"plane": plane}, "--plane")
        assert len(out) == 1 and count(out[0]) == 12, plane


def test_several_planes():
    out = flags({"plane": np.array([CORNERS, CORNERS])}, "--plane")
    assert len(out) == 2 and all(count(v) == 12 for v in out)


def test_boxes():
    assert [count(v) for v in flags({"cubic": np.array(BOX)}, "--cubic")] == [24]
    assert [count(v) for v in flags({"cubic": [BOX, BOX]}, "--cubic")] == [24, 24]


def test_spheres():
    assert [count(v) for v in flags({"sphere": [0.3, 0, 0.5]}, "--sphere")] == [3]
    assert [count(v) for v in flags({"sphere": np.array([0.3, 0, 0.5, 0.05])}, "--sphere")] == [4]
    assert [count(v) for v in flags({"sphere": np.zeros((3, 4))}, "--sphere")] == [4, 4, 4]
    assert flags({"sphere": {"pos": [0.3, 0, 0.5], "rad": 0.05}}, "--sphere") == ["0.3,0,0.5,0.05"]
    assert len(flags({"sphere": {"a": [0.3, 0, 0.5], "b": [0.4, 0, 0.5]}}, "--sphere")) == 2


def test_empty_sections_add_nothing():
    for empty in ([], np.array([]), np.zeros((0, 3))):
        assert flags({"sphere": empty}, "--sphere") == []


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
    print("ok")
