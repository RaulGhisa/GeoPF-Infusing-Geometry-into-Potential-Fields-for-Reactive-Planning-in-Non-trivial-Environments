# Original Code for GeoPF

https://ieeexplore.ieee.org/document/11159155

GeoPF plans a collision-free path for a point (e.g. a robot end-effector)
from a start to a goal among geometric obstacles (spheres, line segments,
rectangular planes, boxes and cylinders) using potential fields that are
shaped by the obstacle geometry.

- [Command-line tool](#command-line-tool): build and run `geopf`
- [Input format](#input-format): scene and parameter files
- [Output](#output): trajectory CSV and JSON summary
- [Using it from Python](#using-it-from-python)
- [Repository layout](#repository-layout)

## Command-line tool

### Build

Requires CMake ≥ 3.18 and a C++17 compiler. Eigen 3 and yaml-cpp are used if
installed, otherwise they are downloaded automatically at configure time
(turn this off with `-DGEOPF_FETCH_DEPS=OFF`).

```bash
cmake -S . -B build
cmake --build build -j
./build/geopf --help
# optional: sudo cmake --install build   # installs geopf to /usr/local/bin
```

DQ Robotics and CoppeliaSim/V-REP are **not** needed for the CLI.

### Run

You can pass all scene data directly as command-line arguments (no JSON or YAML files needed), and `geopf` outputs the result (including the trajectory) to `stdout`:

```bash
./build/geopf \
  --start 0,0,0.5 \
  --goal 0.6,0,0.5 \
  --boundary 0.3,0,0.5,0.6 \
  --sphere 0.15,0.1,0.5,0.03 \
  --line 0.45,-0.2,0.4,0.45,0.2,0.4 \
  --plane 0.3,-0.1,0.4,0.3,0.1,0.4,0.3,0.1,0.6,0.3,-0.1,0.6 \
  --cylinder 0.5,0.25,0.3,0.5,0.25,0.7,0.03 \
  --k-attr 1.0
```

Or stream raw `x,y,z` trajectory rows to `stdout` with `--format csv`:

```bash
./build/geopf --start 0,0,0.5 --goal 0.6,0,0.5 --sphere 0.3,0.01,0.5,0.05 --format csv
```

You can also load from YAML/JSON config files (or combine files with CLI flags):

```bash
./build/geopf -c examples/scene.yaml -c examples/params.yaml -o trajectory.csv
```

```json
{"status":"reached_goal","success":true,"iterations":2030,"path_length":0.757669579048,
 "min_obstacle_distance":0.0306107329543,"mean_obstacle_distance":0.272052478689,
 "runtime_us":6888,"num_obstacles":5,"start":[0,0,0.5],"goal":[0.6,0,0.5],
 "final_position":[0.593586316617,0,0.492338392474],"trajectory_file":"trajectory.csv",
 "trajectory":[[0.00015,0,0.5], ...]}
```

### Options

Numbers in obstacle/position flags can be comma-separated (`--start 0,0,0.5`) or space-separated (`--start 0 0 0.5`).

| Option | Description |
|---|---|
| `--start X,Y,Z` | Start position. |
| `--goal X,Y,Z` | Goal position. |
| `--boundary X,Y,Z,R` | Workspace boundary sphere (centre `X,Y,Z`, radius `R`). |
| `--sphere X,Y,Z[,R]` | Add a sphere obstacle (repeatable). If `R` is omitted, uses `--sphere-rad`. |
| `--line X1,Y1,Z1,X2,Y2,Z2` | Add a line segment obstacle (repeatable). |
| `--plane P1,P2,P3,P4` | Add a rectangular plane: 12 numbers (`X1,Y1,Z1,...,X4,Y4,Z4`), 4 corners in order around the perimeter (repeatable). |
| `--cubic P1,...,P8` | Add a box: 24 numbers, 8 corners (`point1..4` one face, `point5..8` opposite face; repeatable). |
| `--cylinder X1,Y1,Z1,X2,Y2,Z2,R` | Add a cylinder (axis start, axis end, radius; repeatable). |
| `-c, --config FILE` | Scene/parameter file (YAML or JSON), or `-` for stdin. Repeatable; obstacle flags are added on top. |
| `--scene TEXT` | Inline YAML/JSON string (repeatable). |
| `--k-attr V`, `--k-boundary V`, `--k-circ V`, `--k-line V`, `--k-plane V`, `--k-cubic V`, `--k-cylinder V` | Override a force gain. |
| `--sphere-rad R` | Default sphere radius when omitted in `--sphere` or config. Default `0.01`. |
| `--dt V` | Integration time step. Default `0.01`. |
| `--v-max V` | Speed limit. Default `0.04`. |
| `--mass V` | Point mass. Default `1.0`. |
| `--goal-tolerance V` | Stop once this close to the goal. Default `0.01`. |
| `--max-iter N` | Give up after N steps. Default `100000`. |
| `--format json\|csv` | `stdout` format: `json` (default: single-line JSON with `"trajectory"`) or `csv` (`x,y,z` rows on `stdout`, JSON summary on `stderr`). |
| `--no-trajectory` | Omit `"trajectory"` from the JSON output on `stdout`. |
| `-o, --out FILE` | Also write the trajectory as CSV to `FILE`. |
| `--dump-scene DIR` | Write the trajectory and all obstacles as CSV files with the names the [`matlabPlot/`](matlabPlot) scripts expect (`trajectory.csv`, `sphere.csv`, `planepoint1.csv`, …). |
| `-h, --help`, `--version` | |

Options also accept the `--opt=value` form. With no obstacles, `--start` and `--goal` alone are enough (planning in free space).

### Exit status

| Code | Meaning |
|---|---|
| `0` | Goal reached. |
| `1` | Planning ran but failed: `status` is `max_iterations`, `collision` or `numerical_error`. The JSON summary is still printed. |
| `2` | Bad usage or invalid config (message on stderr, nothing on stdout). |

## Input format

Everything can go in one file or be split across several (e.g. scene +
params, as in [`examples/`](examples)). All keys are optional except `start`
and `goal`, which can also be given on the command line. Coordinates are 3D
`[x, y, z]`; units are whatever you use consistently (the defaults assume
metres).

### Scene

```yaml
start: [0.0, 0.0, 0.5]
goal:  [0.6, 0.0, 0.5]

workspace_boundary:          # sphere the point must stay inside
  pos: [0.3, 0.0, 0.5]
  rad: 0.6

sphere:
  - {pos: [0.15, 0.10, 0.50], rad: 0.03}
  - [0.2, 0.2, 0.5]          # bare position -> radius = sphere_rad (0.01)

line:                        # line segments
  - {start: [0.45, -0.2, 0.4], end: [0.45, 0.2, 0.4]}

plane:                       # rectangle; 4 corners in order around the edge
  - points: [[0.3, -0.1, 0.4], [0.3, 0.1, 0.4], [0.3, 0.1, 0.6], [0.3, -0.1, 0.6]]

cubic:                       # box, 8 corners (see below)
  - points: [[0.1,-0.3,0.3], [0.2,-0.3,0.3], [0.2,-0.2,0.3], [0.1,-0.2,0.3],
             [0.1,-0.3,0.4], [0.2,-0.3,0.4], [0.2,-0.2,0.4], [0.1,-0.2,0.4]]

cylinder:
  - {start: [0.5, 0.25, 0.3], end: [0.5, 0.25, 0.7], rad: 0.03}
```

**Box corners:** `point1`–`point4` are one face in order around its edge,
`point5`–`point8` are the opposite face, with `point5` opposite `point1`,
`point6` opposite `point2`, and so on.

**Original format.** The files used by the original research code
(`GeomObstacles.yml`, `RobotPos.yml`, `TuningParams.yml`) work unchanged:

- sections can be maps with named entries (`circle: {circle1: {...}, circle2: {...}}`) instead of lists;
- `circle` is accepted as a synonym of `sphere`;
- `worksapce_boundary` (original spelling) is accepted as well as `workspace_boundary`;
- corners can be given as `point1: [...]`, `point2: [...]`, … instead of `points:`;
- cylinders can use `obs_start` / `obs_end` / `obs_rad`;
- radii can be written as `0.03` or `[0.03]`;
- `limit_iter` is read as `max_iter`;
- keys the CLI doesn't use (`mockup_num`, `path`, …) are ignored.

### Parameters

| Key | Default | Meaning |
|---|---|---|
| `k_attr` | 1.0 | Pull towards the goal. |
| `k_boundary` | 0.1 | Push from the workspace boundary. |
| `k_circ` | 0.1 | Push from spheres. |
| `k_line` | 0.1 | Push from line segments. |
| `k_plane` | 0.1 | Push from planes. |
| `k_cubic` | 0.1 | Push from boxes. |
| `k_cylinder` | 0.1 | Push from cylinders. |
| `dt` | 0.01 | Integration time step. |
| `v_max` | 0.04 | Speed limit. |
| `mass` | 1.0 | Point mass. |
| `goal_tolerance` | 0.01 | Stop distance to the goal. |
| `max_iter` (or `limit_iter`) | 100000 | Maximum number of steps. |
| `sphere_rad` | 0.01 | Radius for spheres given as a bare position. |

The default gains come from the reference MATLAB script; tune them for your
scene. The obstacle forces only act within about 0.05 of an obstacle.

## Output

**Trajectory on `stdout` (`--format csv` or default `--format json`).**
- With `--format json` (the default), `stdout` is a single-line JSON object containing both the summary metrics and `"trajectory": [[x, y, z], ...]` (omit the array with `--no-trajectory`).
- With `--format csv`, `stdout` is raw `x,y,z` rows (one per integration step, no header) and the JSON summary is printed to `stderr`.
- `-o FILE` additionally writes the `x,y,z` CSV rows to `FILE`.

The start position is not included in the trajectory rows; the last row is the final position.

```python
traj = numpy.loadtxt("trajectory.csv", delimiter=",", ndmin=2)   # shape (N, 3)
```

**JSON summary fields.** Diagnostics from the planner always go to `stderr`, so `stdout` is clean.

| Field | Meaning |
|---|---|
| `status` | `reached_goal`, `max_iterations`, `collision` (the point touched or entered an obstacle) or `numerical_error` (NaN position, see limitations). |
| `success` | `true` if `status == "reached_goal"`. |
| `iterations` | Number of steps (= trajectory rows). |
| `path_length` | Sum of step lengths. |
| `min_obstacle_distance`, `mean_obstacle_distance` | Closest and average distance to the obstacles over the run, as computed by the force functions (`null` if there were no obstacles). |
| `runtime_us` | Planning time in microseconds. |
| `num_obstacles` | Number of obstacles, not counting the workspace boundary. |
| `start`, `goal`, `final_position` | `[x, y, z]`. |
| `trajectory_file` | Path given with `-o`, or `null`. |
| `trajectory` | List of `[x, y, z]` (included by default in `json` mode; omitted with `--no-trajectory`). |

### Limitations

- GeoPF is a reactive planner: it can fail (`max_iterations`) in scenes with
  local minima. Try other gains or a different start/goal.
- If the start, the goal and the centre of an obstacle lie exactly on one
  line, the sideways force direction is undefined and the run ends with
  `numerical_error`. Moving the obstacle or the goal by a tiny amount avoids it.
- When the point ends up inside an obstacle, the force functions push it in a
  random direction, so those runs are not reproducible.
- Obstacles are static. Moving obstacles are only supported by the original
  `DynamicMockupAgent.cpp`.

## Using it from Python

[`python/geopf.py`](python/geopf.py) is a single-file wrapper that converts your Python scene data into CLI flags (`--start`, `--goal`, `--sphere`, `--plane`, …), invokes `geopf` via `subprocess`, and reads the trajectory back from `stdout`. No temporary files or JSON inputs are needed. It only uses the standard library; if NumPy is installed the trajectory comes back as an `(N, 3)` array.

```python
from geopf import plan, plan_files

# Scene passed as CLI flags under the hood (dicts or flat coordinate lists/arrays work)
res = plan(
    start=[0.0, 0.0, 0.5],
    goal=[0.6, 0.0, 0.5],
    obstacles={
        "workspace_boundary": [0.3, 0.0, 0.5, 0.6],
        "sphere": [[0.3, 0.01, 0.5, 0.05]],
        "plane": [[[0.45, -0.1, 0.4], [0.45, 0.1, 0.4],
                   [0.45, 0.1, 0.6], [0.45, -0.1, 0.6]]],
    },
    params={"k_attr": 1.0, "k_circ": 0.1, "max_iter": 20000},
)
print(res.status, res.success, res.iterations, res.path_length)
traj = res.trajectory          # (N, 3)

# Or from existing YAML files, with CLI overrides
res = plan_files("examples/scene.yaml", "examples/params.yaml",
                 k_attr=0.5, goal=[0.5, 0.1, 0.5])
```

`plan` and `plan_files` return a `PlanResult` whose fields match the summary (`status`, `success`, `iterations`, `path_length`, `min_obstacle_distance`, `mean_obstacle_distance`, `runtime_us`, `trajectory`, plus `summary`). A failed plan is **not** an exception, so check `res.success`. Invalid input raises `GeoPFError`.

Or call the executable directly with `subprocess` (no wrapper needed):

```python
import subprocess, numpy as np
from io import StringIO

# 1. Get raw CSV trajectory rows directly on stdout:
p = subprocess.run(
    ["./build/geopf",
     "--start", "0,0,0.5", "--goal", "0.6,0,0.5",
     "--sphere", "0.3,0.01,0.5,0.05",
     "--format", "csv"],
    capture_output=True, text=True, check=True,
)
traj = np.loadtxt(StringIO(p.stdout), delimiter=",", ndmin=2)  # shape (N, 3)
```

Each call starts a new process (about 1 ms plus planning time), so the CLI
suits planning whole paths, not running inside a high-rate control loop.

## Repository layout

| Path | Contents |
|---|---|
| `src/force_*.cpp`, `include/geometry_force.h`, `include/attr_force.h` | GeoPF force functions, one per primitive (built as the `geopf_core` library). |
| `src/geopf_cli.cpp` | The `geopf` command-line tool. |
| `python/geopf.py` | Python wrapper for the CLI. |
| `examples/` | Example scene and parameter files. |
| `src/SingleAgent.cpp`, `src/MockupAgent.cpp`, `src/HorizontalMockupAgent.cpp`, `src/DynamicMockupAgent.cpp` | Original experiment programs: single scene, batch benchmarks and moving obstacles. They need DQ Robotics, contain hard-coded paths and are not built by CMake. |
| `matlab/` | MATLAB prototypes per primitive, some driving a Franka arm in CoppeliaSim via DQ Robotics. |
| `matlabPlot/` | MATLAB plotting scripts for the CSV outputs (see `--dump-scene`). |

## Citation



Please cite: 


@ARTICLE{11159155,

  author={Gong, Yuhe and Laha, Riddhiman and Figueredo, Luis},

  journal={IEEE Robotics and Automation Letters}, 

  title={GeoPF: Infusing Geometry Into Potential Fields for Reactive Planning in Non-Trivial Environments}, 

  year={2025},

  volume={10},
  
  number={11},
  
  pages={11546-11553},
  
  keywords={Robots;Path planning;Force;Real-time systems;Geometry;Tuning;Collision avoidance;Trajectory;Reactive planning;redundant robots;manipulation planning;motion and path planning},
  
  doi={10.1109/LRA.2025.3609237}}