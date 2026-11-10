"""Example GeoPF scenes, each targeting one behaviour of the planner.

`SCENES` maps a name to (description, scene dict, extra params). Scene dicts
use the same keys as the CLI's YAML/JSON input, so they can be written out
with json.dump and fed to `geopf -c`.
"""

from __future__ import annotations

import numpy as np


def box(center, size, rot=np.eye(3)):
    """8 corners in GeoPF order (p1-p4 bottom face, p5 above p1, ...)."""
    c = np.asarray(center, float)
    h = np.asarray(size, float) / 2
    ex, ey, ez = (np.asarray(rot, float).T * h[:, None])  # scaled axes
    sgn = [(-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
           (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)]
    return {"points": [(c + a * ex + b * ey + d * ez).tolist() for a, b, d in sgn]}


def rect(center, u, v):
    """Rectangle with half-edge vectors u, v (corners in order)."""
    c, u, v = (np.asarray(x, float) for x in (center, u, v))
    return {"points": [(c - u - v).tolist(), (c + u - v).tolist(),
                       (c + u + v).tolist(), (c - u + v).tolist()]}


def rot_xyz(rx, ry, rz):
    cx, sx, cy, sy, cz, sz = np.cos(rx), np.sin(rx), np.cos(ry), np.sin(ry), np.cos(rz), np.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def scale_scene(scene, k):
    """Uniformly scale every coordinate/radius (e.g. m -> mm)."""
    def s(x):
        if isinstance(x, dict):
            return {kk: s(vv) for kk, vv in x.items()}
        if isinstance(x, list):
            return [s(v) for v in x]
        return x * k
    return s(scene)


S, G = [0.0, 0.0, 0.5], [0.6, 0.0, 0.5]
BOX_C = [0.3, 0.0, 0.5]
BOX = box(BOX_C, [0.1, 0.1, 0.1])
R45 = rot_xyz(np.deg2rad(30), np.deg2rad(40), 0)

SCENES = {
    # ---------------------------------------------------------- baselines
    "free_space": ("no obstacles", {"start": S, "goal": G}, {}),
    "sphere_offset_readme": (
        "README example: sphere 1 cm off the start-goal line",
        {"start": S, "goal": G, "sphere": [{"pos": [0.3, 0.01, 0.5], "rad": 0.05}]}, {}),

    # ---------------------------------------------------------- spheres
    "sphere_collinear": (
        "sphere centred exactly on the start-goal line (common for sim data)",
        {"start": S, "goal": G, "sphere": [{"pos": [0.3, 0.0, 0.5], "rad": 0.05}]}, {}),
    "sphere_large_r015": (
        "large sphere (r=0.15), 2 cm off-line",
        {"start": S, "goal": G, "sphere": [{"pos": [0.3, 0.02, 0.5], "rad": 0.15}]}, {}),
    "goal_near_sphere": (
        "goal 2 cm from a sphere surface (e.g. pre-grasp pose)",
        {"start": S, "goal": [0.6, 0.0, 0.5],
         "sphere": [{"pos": [0.6, 0.07, 0.5], "rad": 0.05}]}, {}),

    # ---------------------------------------------------------- planes
    "wall_head_on": (
        "axis-aligned 0.2x0.2 wall, start-goal line through its centre",
        {"start": S, "goal": G, "plane": [rect([0.3, 0, 0.5], [0, 0.1, 0], [0, 0, 0.1])]}, {}),
    "wall_off_centre": (
        "axis-aligned wall hit 3 cm off-centre",
        {"start": S, "goal": G, "plane": [rect([0.3, 0.03, 0.5], [0, 0.1, 0], [0, 0, 0.1])]}, {}),
    "wall_wide": (
        "1.0 m wide wall (edges far from the line)",
        {"start": S, "goal": G, "plane": [rect([0.3, 0.02, 0.5], [0, 0.5, 0], [0, 0, 0.15])]}, {}),
    "wall_tilted": (
        "0.2x0.2 wall rotated 30deg/40deg about x/y (not axis aligned)",
        {"start": S, "goal": G,
         "plane": [rect([0.3, 0.02, 0.5], (R45 @ [0, 0.1, 0]).tolist(),
                        (R45 @ [0, 0, 0.1]).tolist())]}, {}),
    "wall_tilted_near_miss": (
        "tilted wall whose bounding box (but not the wall) covers the path",
        {"start": [0.0, 0.0, 0.5], "goal": [0.6, 0.0, 0.5],
         "plane": [rect([0.3, 0.12, 0.62], (rot_xyz(0.6, 0.6, 0.6) @ [0, 0.12, 0]).tolist(),
                        (rot_xyz(0.6, 0.6, 0.6) @ [0, 0, 0.12]).tolist())]}, {}),
    "table_goal_above": (
        "1x1 m table plane, goal 3 cm above it, start 30 cm above",
        {"start": [0.3, 0.0, 0.8], "goal": [0.5, 0.1, 0.53],
         "plane": [rect([0.4, 0, 0.5], [0.5, 0, 0], [0, 0.5, 0])]}, {}),
    "table_goal_across": (
        "move across a table 3 cm above its surface",
        {"start": [0.2, -0.2, 0.53], "goal": [0.6, 0.2, 0.53],
         "plane": [rect([0.4, 0, 0.5], [0.5, 0, 0], [0, 0.5, 0])]}, {}),

    # ---------------------------------------------------------- boxes
    **{f"box_from_{n}": (
        f"10 cm box, approach from {n} (2 cm off-line)",
        {"start": (np.array(BOX_C) + 0.3 * np.array(d) + 0.02 * np.array(o)).tolist(),
         "goal": (np.array(BOX_C) - 0.3 * np.array(d) + 0.02 * np.array(o)).tolist(),
         "cubic": [BOX]}, {})
       for n, d, o in [("-x", [-1, 0, 0], [0, 1, 1]), ("+x", [1, 0, 0], [0, 1, 1]),
                       ("-y", [0, -1, 0], [1, 0, 1]), ("+y", [0, 1, 0], [1, 0, 1]),
                       ("-z", [0, 0, -1], [1, 1, 0]), ("+z", [0, 0, 1], [1, 1, 0])]},
    "box_rotated": (
        "10 cm box rotated 30/40 deg",
        {"start": S, "goal": G, "cubic": [box(BOX_C, [0.1, 0.1, 0.1], R45)]}, {}),
    "goal_on_box_top": (
        "goal 3 cm above a 20 cm box (top-down grasp approach)",
        {"start": [0.1, 0.0, 0.8], "goal": [0.3, 0.0, 0.63],
         "cubic": [box([0.3, 0, 0.5], [0.2, 0.2, 0.2])]}, {}),

    # ---------------------------------------------------------- cylinders
    "cylinder_side": (
        "vertical pole (r=3 cm) 1 cm off the line",
        {"start": S, "goal": G,
         "cylinder": [{"start": [0.3, 0.01, 0.3], "end": [0.3, 0.01, 0.7], "rad": 0.03}]}, {}),
    "cylinder_endcap_start": (
        "approach the cylinder cap at its 'start' end",
        {"start": [0.3, 0.01, 0.0], "goal": [0.3, 0.01, 1.0],
         "cylinder": [{"start": [0.3, 0.0, 0.4], "end": [0.3, 0.0, 0.6], "rad": 0.05}]}, {}),
    "cylinder_endcap_end": (
        "approach the cylinder cap at its 'end' end",
        {"start": [0.3, 0.01, 1.0], "goal": [0.3, 0.01, 0.0],
         "cylinder": [{"start": [0.3, 0.0, 0.4], "end": [0.3, 0.0, 0.6], "rad": 0.05}]}, {}),
    "cylinder_endcap_end_offaxis": (
        "approach the 'end' cap 3 cm off-axis",
        {"start": [0.3, 0.03, 1.0], "goal": [0.3, 0.03, 0.0],
         "cylinder": [{"start": [0.3, 0.0, 0.4], "end": [0.3, 0.0, 0.6], "rad": 0.05}]}, {}),
    "cylinder_large_r015": (
        "fat cylinder (r=0.15) - larger than the 0.1 influence radius",
        {"start": S, "goal": G,
         "cylinder": [{"start": [0.3, 0.02, 0.2], "end": [0.3, 0.02, 0.8], "rad": 0.15}]}, {}),

    # ---------------------------------------------------------- lines
    "line_bar": (
        "horizontal bar crossing the path",
        {"start": S, "goal": G, "line": [{"start": [0.3, -0.2, 0.5], "end": [0.3, 0.2, 0.5]}]}, {}),

    # ---------------------------------------------------------- units/scale
    "scaled_x10_sphere": (
        "sphere_offset_readme scaled x10 (e.g. mobile-robot scale)",
        scale_scene({"start": S, "goal": G, "sphere": [{"pos": [0.3, 0.01, 0.5], "rad": 0.05}]}, 10),
        {}),
    "millimetres_sphere": (
        "sphere_offset_readme in millimetres",
        scale_scene({"start": S, "goal": G, "sphere": [{"pos": [0.3, 0.01, 0.5], "rad": 0.05}]}, 1000),
        {}),

    # ---------------------------------------------------------- boundary
    "boundary_tight": (
        "workspace boundary just containing start and goal",
        {"start": S, "goal": G, "workspace_boundary": {"pos": [0.3, 0, 0.5], "rad": 0.32}}, {}),
    "start_outside_boundary": (
        "start slightly outside the workspace boundary",
        {"start": [-0.05, 0, 0.5], "goal": G,
         "workspace_boundary": {"pos": [0.3, 0, 0.5], "rad": 0.33}}, {}),

    "boundary_offcentre": (
        "same tight boundary but centre 1 cm off the start-goal line",
        {"start": S, "goal": G, "workspace_boundary": {"pos": [0.3, 0.01, 0.5], "rad": 0.32}}, {}),

    # ---------------------------------------------------------- goal near surface
    **{f"table_goal_{int(h * 100)}cm": (
        f"table plane, goal {int(h * 100)} cm above it",
        {"start": [0.3, 0.0, 0.8], "goal": [0.5, 0.1, 0.5 + h],
         "plane": [rect([0.4, 0, 0.5], [0.5, 0, 0], [0, 0.5, 0])]}, {})
       for h in (0.06, 0.09, 0.11)},
    "table_as_box_goal_above": (
        "table modelled as a thin box, goal 3 cm above",
        {"start": [0.3, 0.0, 0.8], "goal": [0.5, 0.1, 0.53],
         "cubic": [box([0.4, 0, 0.49], [1.0, 1.0, 0.02])]}, {}),

    # ---------------------------------------------------------- corner ordering
    "box_corners_binary_order": (
        "same 10 cm box but corners in itertools.product (x,y,z bit) order",
        {"start": [0.0, 0.02, 0.52], "goal": [0.6, 0.02, 0.52],
         "cubic": [{"points": [[0.3 + a, 0.0 + b, 0.5 + c] for a in (-.05, .05)
                               for b in (-.05, .05) for c in (-.05, .05)]}]}, {}),
    "plane_corners_zorder": (
        "wall with corners in Z order (p1,p2,p4,p3) instead of around the edge",
        {"start": S, "goal": G,
         "plane": [{"points": [[0.3, -0.1, 0.4], [0.3, 0.1, 0.4], [0.3, -0.1, 0.6], [0.3, 0.1, 0.6]]}]},
        {}),

    # ---------------------------------------------------------- units with rescaled params
    "millimetres_sphere_scaled_params": (
        "mm scene with v_max/goal_tolerance scaled x1000 (gains/influence radii are not scalable)",
        scale_scene({"start": S, "goal": G, "sphere": [{"pos": [0.3, 0.01, 0.5], "rad": 0.05}]}, 1000),
        {"v_max": 40, "goal_tolerance": 10}),

    # ---------------------------------------------------------- local minima
    "concave_cup": (
        "3-wall cup facing the start (classic PF local minimum)",
        {"start": S, "goal": G,
         "plane": [rect([0.35, 0, 0.5], [0, 0.1, 0], [0, 0, 0.1]),
                   rect([0.25, 0.1, 0.5], [0.1, 0, 0], [0, 0, 0.1]),
                   rect([0.25, -0.1, 0.5], [0.1, 0, 0], [0, 0, 0.1])]}, {}),
}
