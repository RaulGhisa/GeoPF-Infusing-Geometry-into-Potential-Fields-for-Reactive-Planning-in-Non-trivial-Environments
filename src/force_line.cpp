#include "../include/geometry_force.h"
#include <algorithm>
#include <cmath>

Vector3d line_dist_obs(const Vector3d &obs_start, const Vector3d &obs_end, const Vector3d &robot_pos) {
  Vector3d ab = obs_end - obs_start;
  double ab_sq = ab.squaredNorm();
  if (ab_sq < 1e-12) {
    return robot_pos - obs_start;
  }
  double t = (robot_pos - obs_start).dot(ab) / ab_sq;
  Vector3d closest;
  if (t <= 0.0) {
    closest = obs_start;
  } else if (t >= 1.0) {
    closest = obs_end;
  } else {
    closest = obs_start + t * ab;
  }
  return robot_pos - closest;
}

Vector3d line_force(const Vector3d &obs_start, const Vector3d &obs_end, const Vector3d &robot_pos, const Vector3d &robot_vel,
                    const Vector3d &goal, const double &k_line, double d_inf) {
  Vector3d dist_obs = line_dist_obs(obs_start, obs_end, robot_pos);
  double d_norm = dist_obs.norm();

  Vector3d lin_force(0, 0, 0);
  if (d_norm < d_inf) {
    Vector3d d_goal = goal - robot_pos;
    double d_goal_norm = d_goal.norm();
    if (d_goal_norm > 1e-9) {
      Vector3d line_goal_robot = d_goal / d_goal_norm;
      Vector3d line_robot_obs;
      if (d_norm > 1e-9) {
        line_robot_obs = dist_obs / d_norm;
      } else {
        Vector3d ab = (obs_end - obs_start).normalized();
        if (std::abs(ab.x()) < 0.9) line_robot_obs = ab.cross(Vector3d(1, 0, 0)).normalized();
        else line_robot_obs = ab.cross(Vector3d(0, 1, 0)).normalized();
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

      lin_force = (k_line / std::max(1e-4, d_norm)) * force_direction;
    }
  }

  if (min_dist > d_norm) {
    min_dist = d_norm;
  }
  ave_dist += d_norm;
  ave_num += 1;
  return lin_force;
}
