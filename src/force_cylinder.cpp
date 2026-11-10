#include "../include/geometry_force.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

bool segment_hits_cylinder(const Vector3d &obs_start, const Vector3d &obs_end, double obs_rad,
                           const Vector3d &a, const Vector3d &b) {
  const Vector3d axis = obs_end - obs_start;
  const double L = axis.norm();
  const Vector3d d = b - a, w = a - obs_start;
  if (L < 1e-12) return line_dist_obs(a, b, obs_start).norm() <= obs_rad;
  const Vector3d u = axis / L;

  // Part of the segment between the two cap planes...
  double lo = 0.0, hi = 1.0;
  const double s0 = w.dot(u), ds = d.dot(u);
  if (std::abs(ds) < 1e-300) {
    if (s0 < 0 || s0 > L) return false;
  } else {
    double t0 = -s0 / ds, t1 = (L - s0) / ds;
    if (t0 > t1) std::swap(t0, t1);
    lo = std::max(lo, t0);
    hi = std::min(hi, t1);
    if (lo > hi) return false;
  }
  // ...that is also within obs_rad of the axis: |wp + t dp|^2 <= r^2.
  const Vector3d wp = w - s0 * u, dp = d - ds * u;
  const double A = dp.squaredNorm(), B = 2 * wp.dot(dp), C = wp.squaredNorm() - obs_rad * obs_rad;
  if (A < 1e-300) return C <= 0;
  const double disc = B * B - 4 * A * C;
  if (disc < 0) return false;
  const double sq = std::sqrt(disc);
  lo = std::max(lo, (-B - sq) / (2 * A));
  hi = std::min(hi, (-B + sq) / (2 * A));
  return lo <= hi;
}

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

      cylin_force = (k_cylinder / std::max(1e-4, d_norm)) *
                    deflection_dir(line_robot_obs, line_goal_robot, robot_vel);

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
