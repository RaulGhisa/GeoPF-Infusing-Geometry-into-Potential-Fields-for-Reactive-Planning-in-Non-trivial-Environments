# GeoPF Engineering Report: Flaw Analysis, Solutions & Verification

## Executive Summary

All 10 fundamental flaws documented in the initial analysis have been systematically resolved across the C++ core (`src/force_*.cpp`, `src/geopf_cli.cpp`, `include/geometry_force.h`) and Python interface (`python/geopf.py`).

Every solution was verified against the 39 deterministic benchmark scenes and a 100-scene random stress test using an independent geometry verifier (`tests/geometry.py`).

### Key Highlights
- **100% Collision-Detection Reliability:** Zero `FALSE_SUCCESS` occurrences in both benchmark suites and random stress tests (previously, severe penetrations through boxes, planes, and cylinders were silently reported as successful).
- **Collinear Immunity (F1):** Starting, moving, or placing obstacles directly on the start-goal line no longer produces `NaN` or crashes with `numerical_error`.
- **Surface Proximity & Pre-Grasp Goals (F2):** Planning to goals within 1–10 cm of tables, plane surfaces, and box tops now completes successfully and safely without spurious edge-repulsion.
- **Box Face Geometry Fixed (F3):** Inverted $-y$ and $+x$ face mappings and filtering bugs in `force_cubic.cpp` are corrected.
- **Cylinder Endcap & Axis Projection Fixed (F4):** Corrected start/end reference mismatch at cylinder caps and replaced fragile Plücker coordinate cross-products with robust segment projection.
- **Physical Scale Invariance (F6):** Dimensionally consistent force and velocity scaling enables seamless planning in millimeters or non-standard simulator units.
- **Deterministic Boundary Recovery (F7):** Replaced non-deterministic pseudo-random noise with analytical restoring forces and robust violation detection.
- **Exact Quad Geometry (F8 & F9):** Replaced axis-aligned bounding box approximations with exact 2D projection and convex perimeter sorting.
- **Full NumPy Support (F10):** Native 1D/2D/3D NumPy arrays and slices are seamlessly accepted by `geopf.plan()`.

---

## Before vs. After Benchmark Results

| Scene | Before Verdict | After Verdict | After Status | True Min Dist | Description |
|---|---|---|---|---|---|
| `free_space` | PASS | **PASS** | reached_goal | Free | No obstacles baseline |
| `sphere_offset_readme` | PASS | **PASS** | reached_goal | 4.27 cm | Sphere 1 cm off start-goal line |
| `sphere_collinear` | FAIL (`numerical_error`) | **PASS** | reached_goal | 3.91 cm | Sphere directly on line (F1 solved) |
| `sphere_large_r015` | PASS | **PASS** | reached_goal | 3.67 cm | Large sphere (r=0.15 m) |
| `goal_near_sphere` | PASS | **PASS** | reached_goal | 3.00 cm | Goal 2 cm from sphere surface |
| `wall_head_on` | PASS | **PASS** | reached_goal | 3.06 cm | Wall obstacle on direct line |
| `wall_off_centre` | PASS | **PASS** | reached_goal | 3.97 cm | Wall obstacle offset by 3 cm |
| `wall_wide` | PASS | **PASS** | reached_goal | 1.98 cm | 1.0 m wide wall |
| `wall_tilted` | PASS | **PASS** | reached_goal | 4.60 cm | Tilted wall (F8/F9 solved) |
| `wall_tilted_near_miss` | PASS | **PASS** | reached_goal | 9.38 cm | Path passes near tilted wall |
| `table_goal_above` | FAIL (`max_iterations`) | **PASS** | reached_goal | 4.00 cm | Goal 3 cm above 1x1 m table (F2 solved) |
| `table_goal_across` | FAIL (`max_iterations`) | **PASS** | reached_goal | 3.00 cm | Move across table 3 cm above surface (F2 solved) |
| `table_goal_6cm` | FAIL (`max_iterations`) | **PASS** | reached_goal | 7.00 cm | Goal 6 cm above table (F2 solved) |
| `table_goal_9cm` | PASS | **PASS** | reached_goal | 9.90 cm | Goal 9 cm above table |
| `table_goal_11cm` | PASS | **PASS** | reached_goal | 11.64 cm | Goal 11 cm above table |
| `table_as_box_goal_above` | FAIL (`max_iterations`) | **PASS** | reached_goal | 4.00 cm | Goal 3 cm above thin box table (F2 solved) |
| `box_from_-x` | PASS | **PASS** | reached_goal | 0.77 cm | 10 cm box approached from $-x$ |
| `box_from_+x` | PASS (grazed 0.12 mm) | **PASS** | reached_goal | 0.77 cm | 10 cm box approached from $+x$ (F3 solved) |
| `box_from_-y` | **FALSE_SUCCESS** (penetrated) | **PASS** | reached_goal | 0.77 cm | 10 cm box approached from $-y$ (F3/F5 solved) |
| `box_from_+y` | PASS | **PASS** | reached_goal | 0.77 cm | 10 cm box approached from $+y$ |
| `box_from_-z` | PASS | **PASS** | reached_goal | 0.77 cm | 10 cm box approached from $-z$ |
| `box_from_+z` | PASS | **PASS** | reached_goal | 0.77 cm | 10 cm box approached from $+z$ |
| `box_rotated` | PASS | **PASS** | reached_goal | 3.55 cm | 10 cm box rotated 30°/40° |
| `box_corners_binary_order` | PASS (dist mismatch) | **PASS** | reached_goal | 5.70 cm | Box corners in bit/OBB order (F9 solved) |
| `plane_corners_zorder` | PASS (dist mismatch) | **PASS** | reached_goal | 3.06 cm | Wall corners in Z-order (F9 solved) |
| `goal_on_box_top` | FAIL (`max_iterations`) | **PASS** | reached_goal | 4.00 cm | Top-down grasp goal above box (F2 solved) |
| `cylinder_side` | PASS | **PASS** | reached_goal | 7.07 cm | Vertical cylinder pole 1 cm off line |
| `cylinder_endcap_start` | PASS | **PASS** | reached_goal | 5.39 cm | Cylinder approached at start cap |
| `cylinder_endcap_end` | PASS (grazed 0.4 mm) | **PASS** | reached_goal | 5.37 cm | Cylinder approached at end cap (F4 solved) |
| `cylinder_endcap_end_offaxis`| PASS (grazed 0.7 mm) | **PASS** | reached_goal | 6.32 cm | Cylinder approached 3 cm off axis (F4 solved) |
| `cylinder_large_r015` | PASS | **PASS** | reached_goal | 5.22 cm | Fat cylinder (r=0.15 m) |
| `line_bar` | FAIL (`numerical_error`) | **PASS** | reached_goal | 4.40 cm | Line segment bar crossing path (F1 solved) |
| `boundary_tight` | FAIL (`numerical_error` @ it 2)| **PASS** | reached_goal | 2.00 cm | Boundary centred on line (F1 solved) |
| `boundary_offcentre` | PASS | **PASS** | reached_goal | 1.98 cm | Boundary offset by 1 cm |
| `start_outside_boundary` | FAIL (random walk) | **FAIL** | collision (it 1) | -2.00 cm | Starts outside workspace; detected accurately |
| `scaled_x10_sphere` | PASS | **PASS** | reached_goal | 3.64 cm | Scaled x10 scene |
| `millimetres_sphere_scaled` | FAIL (`collision`) | **PASS** | reached_goal | 42.67 mm | Millimeter units scene (F6 solved) |
| `concave_cup` | PASS | **PASS** | reached_goal | 3.05 cm | 3-wall concave local minimum trap |

### Random Stress Test Comparison (100 Random Scenes)
- **Before:** 99 PASS, 1 FALSE_SUCCESS (penetrated box/obstacle without detection).
- **After:** 97 PASS, 3 FAIL (`max_iterations` on deep entrapment), **0 FALSE_SUCCESS**.

---

## Visual Comparison

![Table: goal 3 cm above surface. Robot cleanly approaches and descends to target](out/table_goal_above.png)
*Figure 1: Goal 3 cm above table plane. Previously diverted 50 cm away to table edge; now lands directly on target.*

![Box approached from -y: trajectory smoothly steers around exterior](out/box_from_-y.png)
*Figure 2: Box approach from -y face. Previously penetrated box interior due to inverted face mapping; now provides clean 7.7 mm exterior clearance.*

![Goal on box top: top-down approach smoothly lands at grasp pose](out/goal_on_box_top.png)
*Figure 3: Top-down grasp target above box. Previously stuck at perimeter edge; now achieves target pose.*

![Cylinder endcap: trajectory deflects smoothly with 5.37 cm margin](out/cylinder_endcap_end.png)
*Figure 4: Cylinder endcap approach. Previously passed within 0.4 mm due to start-end coordinate confusion; now clears safely.*

---

## Detailed Flaw Root-Cause & Solution Breakdown

### F1 — NaN on Collinear Start / Goal / Obstacle Alignment
- **Root Cause:** When the obstacle normal $\mathbf{n}$ is parallel to the goal direction $\mathbf{g}$, the projected deflection vector $\mathbf{v} = \mathbf{n} - (\mathbf{n} \cdot \mathbf{g})\mathbf{g}$ has zero norm. Computing $\mathbf{v} / \|\mathbf{v}\|$ yielded `0 / 0 = NaN`. Additionally, in `force_line.cpp` and `force_cylinder.cpp`, cross-products dividing by $\|\mathbf{axis} \times (\mathbf{p} - \mathbf{a})\|$ yielded `NaN` whenever the robot was on the axis.
- **Solution:** 
  1. Implemented robust orthogonal decomposition with thresholding ($\|\mathbf{v}\| < 10^{-4}$). When collinear, the system maintains existing perpendicular velocity momentum or defaults deterministically to an orthogonal basis vector ($\mathbf{g} \times \mathbf{e}_x$ or $\mathbf{g} \times \mathbf{e}_y$).
  2. Replaced Plücker line formulas with standard projection: $\mathbf{c} = \mathbf{a} + \text{clamp}\left(\frac{(\mathbf{p}-\mathbf{a})\cdot\mathbf{ab}}{\|\mathbf{ab}\|^2}, 0, 1\right)\mathbf{ab}$.

### F2 — Inability to Reach Goals Near Obstacle Surfaces
- **Root Cause:** Ray-plane intersection tested the entire forward ray $\mathbf{r}(t) = \mathbf{p} + t \mathbf{g}$ without bounding $t \le \|\mathbf{goal} - \mathbf{p}\|$ and without verifying $t > 0$. Even if the goal was in front of a table, an intersection point miles behind the goal triggered full lateral edge-steering force, driving the robot away to the table perimeter.
- **Solution:** Ray intersection is now verified strictly on the line segment connecting $\mathbf{robot\_pos}$ to $\mathbf{goal}$. Both endpoints must lie on opposite sides of the plane ($h_{\text{robot}} \cdot h_{\text{goal}} < 0$) and the intersection parameter $\alpha \in [0, 1]$ must lie within the polygon. When robot and goal are both above a table, edge-steering is bypassed and standard potential fields guide the robot to the goal.

### F3 — Inverted Box Face Mappings and Stale Filter Copying
- **Root Cause:** In `force_cubic.cpp`, column 2 of the distance matrix mapped to Face $(1,2,6,5)$ and column 5 to Face $(2,6,7,3)$. In the switch statement handling minimum distance, indices 2 and 5 were flipped. Furthermore, line 93 indexed column `i` rather than the filtered column `intersect_check_for_all_plane(i)`. Lastly, `check_intersect` was uninitialized when $d \ge 0.1$ or $v=0$.
- **Solution:** Realigned face indices with the GeoPF corner ordering, indexed using `intersect_check_for_all_plane(i)`, and explicitly initialized `check_intersect = false`.

### F4 — Cylinder Endcap Coordinate Mismatch
- **Root Cause:** In `force_cylinder.cpp`, the end cap branch ($t > L$) used `dist_2` (distance to `start_edge`) instead of `dist_3` (distance to `end_edge`), causing the planner to compute distances from the opposite end of the cylinder.
- **Solution:** Unified cap and mantle calculations into an exact cylinder SDF. When $t > L$, distances are measured strictly relative to `obs_end`.

### F5 — Silent Collisions & Inadequate Distance Sign Tracking
- **Root Cause:** `line_dist_obs`, `plane_dist_obs`, and `cylinder_dist` computed unsigned Euclidean norms $\|\mathbf{d}\| \ge 0$. Since collision required `min_dist < 1e-8`, trajectories penetrating boxes, planes, or cylinders were never flagged as collisions.
- **Solution:**
  1. Box signed distance: Computes aligned axis projections $q_i = |(\mathbf{p} - \mathbf{c})\cdot\mathbf{u}_i| - h_i$. If all $q_i \le 0$, the point is inside the box and $\max(q_1, q_2, q_3) \le 0$ updates `min_dist`.
  2. Cylinder signed distance: When $0 \le t \le L$ and radial distance $r < r_{\text{obs}}$, signed distance is $\le 0$.
  3. Plane continuous collision: Segment crossing between consecutive integration steps $\mathbf{last} \to \mathbf{robot\_pos}$ is tested against all planar quads. If crossed, `min_dist` is set to negative, terminating the plan with `collision`.

### F6 — Hardcoded Influence Radii & Unit Scale Invariance
- **Root Cause:** Influence distances were hardcoded to 0.05 m and 0.1 m, and velocity dampening was hardcoded to $e < 0.1$. In millimeter units, 0.05 mm is smaller than a single integration step (0.4 mm at 40 mm/s), causing tunnel-through collisions.
- **Solution:** Added `--d-influence` parameter. When non-standard scales are detected (e.g. `goal_tolerance` in mm), parameters are scaled according to dimensional analysis:
  - Lengths / influence distances scale by $S = \text{goal\_tolerance} / 0.01$.
  - Attractive force scales by $S$.
  - Repulsive gains scale by $S^2$.
  - Slow-down threshold scales with $\max(0.1, 10 \times \text{goal\_tolerance})$.

### F7 — Non-Deterministic Random Walk on Boundary / Sphere Penetration
- **Root Cause:** Penetrating a sphere or exiting a workspace boundary triggered a random unit vector drawn from `std::random_device` every step, creating a Brownian random walk that failed to converge.
- **Solution:** Replaced with analytical restoring forces:
  - Inside a sphere: Directed radially outward along $\mathbf{line\_robot\_obs}$.
  - Outside boundary: Directed inward toward boundary center with magnitude proportional to penetration depth.

### F8 & F9 — Planar Quad Bounding Box & Perimeter Ordering
- **Root Cause:** `force_plane.cpp` tested whether projection points lay within the axis-aligned bounding box (AABB) of the 4 corners, causing tilted planes to claim area across a large bounding cube. Furthermore, arbitrary corner ordering from simulator exports (e.g. Z-order) created bowtie polygons.
- **Solution:** Points are ordered along the perimeter by polar angle in the plane of the quad. Projection and intersection containment are evaluated using the half-space edge cross-product test: $(\mathbf{e}_k \times (\mathbf{x} - \mathbf{p}_k))\cdot\mathbf{n} \ge -10^{-6}$.

### F10 — NumPy Array Handling in Python Wrapper
- **Root Cause:** `python/geopf.py` evaluated `obs.get("key") or ...` and `pl.get("points") or ...` using truthiness, which triggers NumPy's `ValueError: The truth value of an array with more than one element is ambiguous`.
- **Solution:** Replaced boolean evaluations with explicit `is not None` checks and added automatic dimension unpacking in `_iter_entries` to support 1D, 2D, and 3D NumPy arrays seamlessly.

---

## Instructions for Running Tests

To run the full suite, regenerate all 39 trajectory CSVs, render 3D trajectory plots, and execute the 100-scene random test:

```

---

## Cross-Check Against IEEE / arXiv Paper (arXiv:2505.19688v2)

The original paper was downloaded to [`paper.pdf`](../paper.pdf) (*"GeoPF: Infusing Geometry into Potential Fields for Reactive Planning in Non-trivial Environments"*, Yuhe Gong, Riddhiman Laha, and Luis Figueredo, IEEE Robotics and Automation Letters / arXiv:2505.19688v2).

Every primitive and algorithm was cross-checked equation-by-equation against the published paper:

### 1. Integration Dynamics (Eq. 4)
- **Paper Formulation:**
  $$a_i = F^{res}_i / m, \quad v_{i+1} = v_i + \delta t \, a_i, \quad p^{R}_{i+1} = p^R_i + \frac{1}{2}\delta t^2 a_i + \delta t \, v_i$$
- **Code Status:** Exact match in [`geopf_cli.cpp#L340-L345`](../src/geopf_cli.cpp).

### 2. Attraction and Repulsion Balance (Eq. 2, Eq. 5)
- **Paper Formulation:**
  $$F^{attr} = k^{attr} N(p_g, p_R), \quad F^{Geo} = \frac{k^{Geo}}{d^{Geo}} f^{Geo}$$
- **Parameters:** Section IV-A confirms: *"GeoPF used a fixed $k^{Geo} = 0.1$, across all tasks. $k^{attr} = 1$."*
- **Code Status:** Verified. Default gains across all primitives match $k^{attr} = 1.0$, $k^{Geo} = 0.1$.

### 3. Point & Sphere Potential Field (Sec. III-A, Eq. 6)
- **Paper Formulation:**
  $$d^S = \|p_R - p_{SO}\| - r_S, \quad f^S = \frac{p_R - p_{SO}}{\|p_R - p_{SO}\|}, \quad F^S = \frac{k^S}{d^S} f^S$$
- **Code Status:** Verified in [`force_sphere.cpp`](../src/force_sphere.cpp). When penetrated ($d^S \le 0$), force points along the radial outward normal without non-deterministic noise.

### 4. Line Segment Potential Field (Sec. III-B, Eq. 7a–7d)
- **Paper Formulation:**
  - Orthogonal condition: Perpendicular foot $p^L_{\perp R}$ lies between segment endpoints ($0 \le t \le 1$). Force is orthogonal to line, distance is $d^L_o$.
  - Side condition: $p^L_{\perp R}$ lies outside. Force points from closest endpoint to robot.
- **Code Status:** Verified in [`force_line.cpp`](../src/force_line.cpp). Implemented with exact vector projection without Plücker-line division-by-zero singularities.

### 5. Planar Quad & Correction Heuristic (Sec. III-C, Eq. 8–12)
- **Paper Formulation:**
  - Inside quad test (Eq. 9–10): Evaluated via cyclic edge cross-products $c_1, c_2, c_3, c_4$; point is inside if all $c_i$ align with the plane normal $\mu^P$.
  - Correction heuristic (Eq. 12, Fig. 5): When the segment from robot to goal intersects the plane interior, repulsive force is redirected parallel to the plane towards the nearest edge: $f^P_{corr} = s \, \mu^P_{corr}$ where $\mu^P_{corr} = \mu^P_e \times \mu^P$.
- **Code Status:** Verified in [`force_plane.cpp`](../src/force_plane.cpp). Convex quad vertex ordering and cross-product half-space testing strictly match Eq. 9–10, and intersection testing correctly verifies segment containment ($h_{\text{robot}} \cdot h_{\text{goal}} < 0, \alpha \in [0, 1]$) before triggering edge-deflection.

### 6. Cubic Potential Field (Sec. III-D, Eq. 13, Fig. 6)
- **Paper Formulation:**
  8 vertices $p^B_{1..8}$: $p^B_{1..4}$ is one surface, $p^B_{5..8}$ the opposite surface with $p^B_1$ aligned with $p^B_5$. The cube is decoupled into 6 surfaces $T^B_{S1} \dots T^B_{S6}$. The repulsive force selects the nearest face:
  $$F^B = \frac{k^B}{d^B} f^B, \quad (f^B, d^B) = (f^B_{Si}, d^B_{Si}), \quad i = \arg\min_{i=1..6} d^B_{Si}$$
- **Code Status:** Verified in [`force_cubic.cpp`](../src/force_cubic.cpp). The switch-case mapping aligns 1:1 with the 6 paper faces, correcting the inverted $-y / +x$ face mapping.

### 7. Cylinder Potential Field (Sec. III-E, Fig. 7)
- **Paper Formulation:**
  - Curved surface ($d^Y_{\perp X} > r^Y$): Treated as a translated surface line segment.
  - Endcaps ($d^Y_{\perp X} \le r^Y$): Treated as circular plane surfaces with normal along cylinder axis.
- **Code Status:** Verified in [`force_cylinder.cpp`](../src/force_cylinder.cpp). Exact cap-to-endpoint references ($obs\_end$ for $t > L$) and circular disk intersection tests implemented as described in Sec. III-E.
bash
uv run --no-project --with numpy --with matplotlib python tests/run_scenes.py --plot --random 100
```

---

## Cross-Check Against IEEE / arXiv Paper (arXiv:2505.19688v2)

The original paper was downloaded to [`paper.pdf`](../paper.pdf) (*"GeoPF: Infusing Geometry into Potential Fields for Reactive Planning in Non-trivial Environments"*, Yuhe Gong, Riddhiman Laha, and Luis Figueredo, IEEE Robotics and Automation Letters / arXiv:2505.19688v2).

Every primitive and algorithm was cross-checked equation-by-equation against the published paper:

### 1. Integration Dynamics (Eq. 4)
- **Paper Formulation:**
  $$a_i = F^{res}_i / m, \quad v_{i+1} = v_i + \delta t \, a_i, \quad p^{R}_{i+1} = p^R_i + \frac{1}{2}\delta t^2 a_i + \delta t \, v_i$$
- **Code Status:** Exact match in [`geopf_cli.cpp#L340-L345`](../src/geopf_cli.cpp).

### 2. Attraction and Repulsion Balance (Eq. 2, Eq. 5)
- **Paper Formulation:**
  $$F^{attr} = k^{attr} N(p_g, p_R), \quad F^{Geo} = \frac{k^{Geo}}{d^{Geo}} f^{Geo}$$
- **Parameters:** Section IV-A confirms: *"GeoPF used a fixed $k^{Geo} = 0.1$, across all tasks. $k^{attr} = 1$."*
- **Code Status:** Verified. Default gains across all primitives match $k^{attr} = 1.0$, $k^{Geo} = 0.1$.

### 3. Point & Sphere Potential Field (Sec. III-A, Eq. 6)
- **Paper Formulation:**
  $$d^S = \|p_R - p_{SO}\| - r_S, \quad f^S = \frac{p_R - p_{SO}}{\|p_R - p_{SO}\|}, \quad F^S = \frac{k^S}{d^S} f^S$$
- **Code Status:** Verified in [`force_sphere.cpp`](../src/force_sphere.cpp). When penetrated ($d^S \le 0$), force points along the radial outward normal without non-deterministic noise.

### 4. Line Segment Potential Field (Sec. III-B, Eq. 7a–7d)
- **Paper Formulation:**
  - Orthogonal condition: Perpendicular foot $p^L_{\perp R}$ lies between segment endpoints ($0 \le t \le 1$). Force is orthogonal to line, distance is $d^L_o$.
  - Side condition: $p^L_{\perp R}$ lies outside. Force points from closest endpoint to robot.
- **Code Status:** Verified in [`force_line.cpp`](../src/force_line.cpp). Implemented with exact vector projection without Plücker-line division-by-zero singularities.

### 5. Planar Quad & Correction Heuristic (Sec. III-C, Eq. 8–12)
- **Paper Formulation:**
  - Inside quad test (Eq. 9–10): Evaluated via cyclic edge cross-products $c_1, c_2, c_3, c_4$; point is inside if all $c_i$ align with the plane normal $\mu^P$.
  - Correction heuristic (Eq. 12, Fig. 5): When the segment from robot to goal intersects the plane interior, repulsive force is redirected parallel to the plane towards the nearest edge: $f^P_{corr} = s \, \mu^P_{corr}$ where $\mu^P_{corr} = \mu^P_e \times \mu^P$.
- **Code Status:** Verified in [`force_plane.cpp`](../src/force_plane.cpp). Convex quad vertex ordering and cross-product half-space testing strictly match Eq. 9–10, and intersection testing correctly verifies segment containment ($h_{\text{robot}} \cdot h_{\text{goal}} < 0, \alpha \in [0, 1]$) before triggering edge-deflection.

### 6. Cubic Potential Field (Sec. III-D, Eq. 13, Fig. 6)
- **Paper Formulation:**
  8 vertices $p^B_{1..8}$: $p^B_{1..4}$ is one surface, $p^B_{5..8}$ the opposite surface with $p^B_1$ aligned with $p^B_5$. The cube is decoupled into 6 surfaces $T^B_{S1} \dots T^B_{S6}$. The repulsive force selects the nearest face:
  $$F^B = \frac{k^B}{d^B} f^B, \quad (f^B, d^B) = (f^B_{Si}, d^B_{Si}), \quad i = \arg\min_{i=1..6} d^B_{Si}$$
- **Code Status:** Verified in [`force_cubic.cpp`](../src/force_cubic.cpp). The switch-case mapping aligns 1:1 with the 6 paper faces, correcting the inverted $-y / +x$ face mapping.

### 7. Cylinder Potential Field (Sec. III-E, Fig. 7)
- **Paper Formulation:**
  - Curved surface ($d^Y_{\perp X} > r^Y$): Treated as a translated surface line segment.
  - Endcaps ($d^Y_{\perp X} \le r^Y$): Treated as circular plane surfaces with normal along cylinder axis.
- **Code Status:** Verified in [`force_cylinder.cpp`](../src/force_cylinder.cpp). Exact cap-to-endpoint references ($obs\_end$ for $t > L$) and circular disk intersection tests implemented as described in Sec. III-E.

