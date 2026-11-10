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
        Vector3d line_robot_obs;
        if (d_obs_norm > 1e-9) {
            line_robot_obs = d_obs / d_obs_norm;
        } else {
            line_robot_obs = Vector3d(1, 0, 0);
        }

        Vector3d force_direction = line_robot_obs - (line_goal_robot * (line_robot_obs.dot(line_goal_robot)));
        if (force_direction.norm() < 1e-4) {
            Vector3d v_perp = robot_vel - line_goal_robot * (line_goal_robot.dot(robot_vel));
            if (v_perp.norm() > 1e-4) {
                force_direction = v_perp.normalized();
            } else if (std::abs(line_goal_robot.x()) < 0.9) {
                force_direction = line_goal_robot.cross(Vector3d(1, 0, 0)).normalized();
            } else {
                force_direction = line_goal_robot.cross(Vector3d(0, 1, 0)).normalized();
            }
        } else {
            force_direction.normalize();
        }

        if (dist_obs <= 0) {
            // Penetrated sphere: push directly outward away from sphere center
            circ_force = line_robot_obs * (k_circle / std::max(1e-4, dist_obs + obs_rad) + 10.0);
        } else {
            circ_force = (k_circle / dist_obs) * force_direction;
        }
    }

    if (min_dist > dist_obs) {
        min_dist = dist_obs;
    }

    ave_dist = ave_dist + dist_obs;
    ave_num = ave_num + 1;

    return circ_force;
}
