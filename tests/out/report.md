| scene | verdict | reported | iters | true_min_d | reported_min_d | final_err | hit | description |
|---|---|---|---|---|---|---|---|---|
| free_space | PASS | reached_goal | 1730 | nan | None | 0.009968 | - | no obstacles |
| sphere_offset_readme | PASS | reached_goal | 1834 | 0.04267 | 0.04267 | 0.009943 | - | README example: sphere 1 cm off the start-goal line |
| sphere_collinear | PASS | reached_goal | 1850 | 0.03906 | 0.03906 | 0.009968 | - | sphere centred exactly on the start-goal line (common for sim data) |
| sphere_large_r015 | PASS | reached_goal | 1984 | 0.03665 | 0.03665 | 0.00997 | - | large sphere (r=0.15), 2 cm off-line |
| goal_near_sphere | PASS | reached_goal | 1743 | 0.02995 | 0.03001 | 0.009948 | - | goal 2 cm from a sphere surface (e.g. pre-grasp pose) |
| wall_head_on | PASS | reached_goal | 1989 | 0.03061 | 0.03061 | 0.009971 | - | axis-aligned 0.2x0.2 wall, start-goal line through its centre |
| wall_off_centre | PASS | reached_goal | 1936 | 0.0397 | 0.0397 | 0.009965 | - | axis-aligned wall hit 3 cm off-centre |
| wall_wide | PASS | reached_goal | 2088 | 0.01975 | 0.01975 | 0.009959 | - | 1.0 m wide wall (edges far from the line) |
| wall_tilted | PASS | reached_goal | 2101 | 0.04604 | 0.04604 | 0.009946 | - | 0.2x0.2 wall rotated 30deg/40deg about x/y (not axis aligned) |
| wall_tilted_near_miss | PASS | reached_goal | 1796 | 0.09379 | 0.09379 | 0.009974 | - | tilted wall whose bounding box (but not the wall) covers the path |
| table_goal_above | PASS | reached_goal | 1201 | 0.03995 | 0.04001 | 0.009948 | - | 1x1 m table plane, goal 3 cm above it, start 30 cm above |
| table_goal_across | PASS | reached_goal | 1807 | 0.03 | 0.03 | 0.009979 | - | move across a table 3 cm above its surface |
| box_from_-x | PASS | reached_goal | 1934 | 0.007732 | 0.007732 | 0.009943 | - | 10 cm box, approach from -x (2 cm off-line) |
| box_from_+x | PASS | reached_goal | 1934 | 0.007732 | 0.007732 | 0.009943 | - | 10 cm box, approach from +x (2 cm off-line) |
| box_from_-y | PASS | reached_goal | 1934 | 0.007732 | 0.007732 | 0.009943 | - | 10 cm box, approach from -y (2 cm off-line) |
| box_from_+y | PASS | reached_goal | 1934 | 0.007732 | 0.007732 | 0.009943 | - | 10 cm box, approach from +y (2 cm off-line) |
| box_from_-z | PASS | reached_goal | 1934 | 0.007732 | 0.007732 | 0.009943 | - | 10 cm box, approach from -z (2 cm off-line) |
| box_from_+z | PASS | reached_goal | 1934 | 0.007732 | 0.007732 | 0.009943 | - | 10 cm box, approach from +z (2 cm off-line) |
| box_rotated | PASS | reached_goal | 2002 | 0.03552 | 0.03552 | 0.009949 | - | 10 cm box rotated 30/40 deg |
| goal_on_box_top | PASS | reached_goal | 1019 | 0.03996 | 0.04002 | 0.009958 | - | goal 3 cm above a 20 cm box (top-down grasp approach) |
| cylinder_side | PASS | reached_goal | 1860 | 0.0707 | 0.0707 | 0.009979 | - | vertical pole (r=3 cm) 1 cm off the line |
| cylinder_endcap_start | PASS | reached_goal | 2782 | 0.05387 | 0.05387 | 0.009991 | - | approach the cylinder cap at its 'start' end |
| cylinder_endcap_end | PASS | reached_goal | 2782 | 0.0537 | 0.0537 | 0.009992 | - | approach the cylinder cap at its 'end' end |
| cylinder_endcap_end_offaxis | PASS | reached_goal | 2751 | 0.06323 | 0.06323 | 0.009991 | - | approach the 'end' cap 3 cm off-axis |
| cylinder_large_r015 | PASS | reached_goal | 2059 | 0.05218 | 0.05218 | 0.00997 | - | fat cylinder (r=0.15) - larger than the 0.1 influence radius |
| line_bar | PASS | reached_goal | 1784 | 0.04403 | 0.04403 | 0.009952 | - | horizontal bar crossing the path |
| scaled_x10_sphere | PASS | reached_goal | 14177 | 0.03643 | 0.03643 | 0.009992 | - | sphere_offset_readme scaled x10 (e.g. mobile-robot scale) |
| millimetres_sphere | FAIL | max_iterations | 30000 | 236.7 | 236.7 | 586.5 | - | sphere_offset_readme in millimetres |
| boundary_tight | PASS | reached_goal | 1742 | 0.02 | None | 0.009969 | - | workspace boundary just containing start and goal |
| start_outside_boundary | FAIL | collision | 1 | -0.02 | None | 0.649 | boundary | start slightly outside the workspace boundary |
| boundary_offcentre | PASS | reached_goal | 1762 | 0.01983 | None | 0.009943 | - | same tight boundary but centre 1 cm off the start-goal line |
| table_goal_6cm | PASS | reached_goal | 1127 | 0.06997 | 0.07003 | 0.009974 | - | table plane, goal 6 cm above it |
| table_goal_9cm | PASS | reached_goal | 1079 | 0.09902 | 0.09905 | 0.009941 | - | table plane, goal 9 cm above it |
| table_goal_11cm | PASS | reached_goal | 1049 | 0.1164 | 0.1165 | 0.009952 | - | table plane, goal 11 cm above it |
| table_as_box_goal_above | PASS | reached_goal | 1201 | 0.03995 | 0.04001 | 0.009948 | - | table modelled as a thin box, goal 3 cm above |
| box_corners_binary_order | PASS | reached_goal | 1892 | 0.05701 | 0.0569 | 0.009988 | - | same 10 cm box but corners in itertools.product (x,y,z bit) order |
| plane_corners_zorder | PASS | reached_goal | 1989 | 0.03061 | 0.03061 | 0.009971 | - | wall with corners in Z order (p1,p2,p4,p3) instead of around the edge |
| millimetres_sphere_scaled_params | PASS | reached_goal | 1834 | 42.67 | 42.67 | 9.943 | - | mm scene with v_max/goal_tolerance scaled x1000 (gains/influence radii are not scalable) |
| concave_cup | PASS | reached_goal | 2001 | 0.0305 | 0.0305 | 0.009959 | - | 3-wall cup facing the start (classic PF local minimum) |
