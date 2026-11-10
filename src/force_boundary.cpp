#include "../include/geometry_force.h"
#include <algorithm>
#include <cmath>

// Workspace boundary: a sphere the robot should stay inside. It is a soft
// limit, not an obstacle: it never touches min_dist, so leaving it does not
// end a run (as in the original code). Outside it, the force pulls the robot
// back in.
Vector3d boundary_force(const Vector3d &obs_pos, const double &obs_rad,
                        const Vector3d &robot_pos, const Vector3d &robot_vel,
                        const Vector3d &goal, const double &k_circle,
                        double d_inf) {
  Vector3d d_goal = goal - robot_pos;
  double d_goal_norm = d_goal.norm();
  if (d_goal_norm < 1e-9) return Vector3d::Zero();
  Vector3d line_goal_robot = d_goal / d_goal_norm;

  Vector3d d_obs = robot_pos - obs_pos;
  double d_obs_norm = d_obs.norm();
  double dist_obs = obs_rad - d_obs_norm;  // distance to the boundary, > 0 inside

  Vector3d circ_force(0, 0, 0);
  if (dist_obs < d_inf) {
    // Points from the boundary surface into the workspace.
    const Vector3d inward =
        d_obs_norm > 1e-9 ? Vector3d(-d_obs / d_obs_norm) : Vector3d(-1, 0, 0);
    if (dist_obs > 0) {
      circ_force = (k_circle / dist_obs) * deflection_dir(inward, line_goal_robot, robot_vel);
    } else {
      // Outside: pull straight back in. k / (0.1 d_inf) is the inside force
      // at 10% of the influence distance; it grows with how far out the
      // robot is. Both terms scale like the other repulsive forces.
      circ_force = inward * (k_circle / (0.1 * d_inf)) * (1.0 - dist_obs / d_inf);
    }
  }

  return circ_force;
}
