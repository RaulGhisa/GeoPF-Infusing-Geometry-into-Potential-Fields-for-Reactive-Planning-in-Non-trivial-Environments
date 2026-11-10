#include "../include/geometry_force.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

Vector3d cylinder_force(const Vector3d &obs_start, const Vector3d &obs_end, const double &obs_rad, 
                        const Vector3d &robot_pos, const Vector3d &robot_vel,
                        const Vector3d &goal, const double &k_cylinder, double d_inf) {

  Vector3d axis_vec = obs_end - obs_start;
  double L = axis_vec.norm();
  if (L < 1e-9) {
    return sphere_force(obs_start, obs_rad, robot_pos, robot_vel, goal, k_cylinder, d_inf);
  }
  Vector3d u = axis_vec / L;

  double t = (robot_pos - obs_start).dot(u);
  Vector3d r_vec = (robot_pos - obs_start) - t * u;
  double r = r_vec.norm();
  Vector3d r_unit;
  if (r > 1e-9) {
    r_unit = r_vec / r;
  } else {
    if (std::abs(u.x()) < 0.9) r_unit = u.cross(Vector3d(1, 0, 0)).normalized();
    else r_unit = u.cross(Vector3d(0, 1, 0)).normalized();
  }

  Vector3d dist_obs;
  bool is_inside = false;
  double signed_dist = 10000.0;

  if (t >= 0.0 && t <= L) {
    // Mantle zone
    Vector3d p_surf = obs_start + t * u + obs_rad * r_unit;
    dist_obs = robot_pos - p_surf;
    if (r < obs_rad) {
      is_inside = true;
      signed_dist = std::max(r - obs_rad, std::max(-t, t - L));
    }
  } else if (t < 0.0) {
    // Start cap zone
    if (r <= obs_rad) {
      Vector3d p_cap = obs_start + r_vec;
      dist_obs = robot_pos - p_cap;
    } else {
      Vector3d p_rim = obs_start + obs_rad * r_unit;
      dist_obs = robot_pos - p_rim;
    }
  } else {
    // End cap zone (t > L)
    if (r <= obs_rad) {
      Vector3d p_cap = obs_end + r_vec;
      dist_obs = robot_pos - p_cap;
    } else {
      Vector3d p_rim = obs_end + obs_rad * r_unit;
      dist_obs = robot_pos - p_rim;
    }
  }

  if (is_inside && min_dist > signed_dist) {
    min_dist = signed_dist;
  }

  double d_norm = dist_obs.norm();
  Vector3d cylin_force(0, 0, 0);

  if (d_norm < d_inf) {
    Vector3d d_goal = goal - robot_pos;
    double d_goal_norm = d_goal.norm();
    if (d_goal_norm > 1e-9) {
      Vector3d line_goal_robot = d_goal / d_goal_norm;
      Vector3d line_robot_obs;
      if (d_norm > 1e-9) line_robot_obs = dist_obs / d_norm;
      else line_robot_obs = r_unit;

      Vector3d force_direction = line_robot_obs - (line_goal_robot * (line_robot_obs.dot(line_goal_robot)));
      if (force_direction.norm() < 1e-4) {
        Vector3d v_perp = robot_vel - line_goal_robot * (line_goal_robot.dot(robot_vel));
        if (v_perp.norm() > 1e-4) force_direction = v_perp.normalized();
        else if (std::abs(line_goal_robot.x()) < 0.9) force_direction = line_goal_robot.cross(Vector3d(1, 0, 0)).normalized();
        else force_direction = line_goal_robot.cross(Vector3d(0, 1, 0)).normalized();
      } else {
        force_direction.normalize();
      }

      cylin_force = (k_cylinder / std::max(1e-4, d_norm)) * force_direction;

      // Cap intersection check: does path to goal hit either cap disk?
      if (t < 0.0) {
        double h_rob = t;
        double h_g = (goal - obs_start).dot(u);
        if (h_rob * h_g < -1e-6) {
          double alpha = h_rob / (h_rob - h_g);
          if (alpha >= 0.0 && alpha <= 1.0) {
            Vector3d p_inter = robot_pos + alpha * (goal - robot_pos);
            if ((p_inter - obs_start).norm() <= obs_rad) {
              Vector3d steer = r_unit;
              cylin_force = (k_cylinder / std::max(1e-4, d_norm)) * steer;
            }
          }
        }
      } else if (t > L) {
        double h_rob = t - L;
        double h_g = (goal - obs_end).dot(u);
        if (h_rob * h_g < -1e-6) {
          double alpha = h_rob / (h_rob - h_g);
          if (alpha >= 0.0 && alpha <= 1.0) {
            Vector3d p_inter = robot_pos + alpha * (goal - robot_pos);
            if ((p_inter - obs_end).norm() <= obs_rad) {
              Vector3d steer = r_unit;
              cylin_force = (k_cylinder / std::max(1e-4, d_norm)) * steer;
            }
          }
        }
      }
    }
  }

  if (min_dist > d_norm && !is_inside) {
    min_dist = d_norm;
  }
  ave_dist += d_norm;
  ave_num += 1;

  return cylin_force;
}
