#include "../include/geometry_force.h"
#include <algorithm>
#include <cmath>

// min_dist / ave_dist / ave_num are declared extern in geometry_force.h

Vector3d sphere_force(const Vector3d &obs_pos, const double &obs_rad, const Vector3d &robot_pos,
                      const Vector3d &robot_vel, const Vector3d &goal, const double &k_circle,
                      double d_inf) {
    Vector3d d_goal = goal - robot_pos;
    double d_goal_norm = d_goal.norm();
    if (d_goal_norm < 1e-9) return Vector3d::Zero();
    Vector3d line_goal_robot = d_goal / d_goal_norm;

    Vector3d d_obs = robot_pos - obs_pos;
    double d_obs_norm = d_obs.norm();
    double dist_obs = d_obs_norm - obs_rad;

    Vector3d circ_force(0, 0, 0);

    if (dist_obs < d_inf) {
        const Vector3d line_robot_obs =
            d_obs_norm > 1e-9 ? Vector3d(d_obs / d_obs_norm) : Vector3d(1, 0, 0);
        if (dist_obs > 0) {
            circ_force = (k_circle / dist_obs) *
                         deflection_dir(line_robot_obs, line_goal_robot, robot_vel);
        } else {
            // Inside the sphere. min_dist <= 0 below makes every caller stop
            // with a collision, so this only keeps the force finite.
            circ_force = line_robot_obs * (k_circle / (0.1 * d_inf));
        }
    }

    if (min_dist > dist_obs) {
        min_dist = dist_obs;
    }

    ave_dist = ave_dist + dist_obs;
    ave_num = ave_num + 1;

    return circ_force;
}
