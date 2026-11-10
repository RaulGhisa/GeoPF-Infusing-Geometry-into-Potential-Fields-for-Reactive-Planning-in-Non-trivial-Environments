#include "../include/geometry_force.h"
#include <algorithm>
#include <cmath>

Vector3d deflection_dir(const Vector3d &n, const Vector3d &g, const Vector3d &robot_vel) {
  const Vector3d d = n - g * n.dot(g);
  if (d.norm() >= 1e-4) return d.normalized();
  const Vector3d v_perp = robot_vel - g * g.dot(robot_vel);
  if (v_perp.norm() > 1e-4) return v_perp.normalized();
  return (std::abs(g.x()) < 0.9 ? g.cross(Vector3d::UnitX()) : g.cross(Vector3d::UnitY()))
      .normalized();
}

// Closest points of two segments (Ericson, Real-Time Collision Detection 5.1.9).
double segment_segment_distance(const Vector3d &a0, const Vector3d &a1,
                                const Vector3d &b0, const Vector3d &b1) {
  const Vector3d d1 = a1 - a0, d2 = b1 - b0, r = a0 - b0;
  const double a = d1.squaredNorm(), e = d2.squaredNorm(), f = d2.dot(r);
  const double tiny = 1e-300;
  double s = 0, t = 0;
  if (a <= tiny && e <= tiny) return r.norm();
  if (a <= tiny) {
    t = std::clamp(f / e, 0.0, 1.0);
  } else {
    const double c = d1.dot(r);
    if (e <= tiny) {
      s = std::clamp(-c / a, 0.0, 1.0);
    } else {
      const double b = d1.dot(d2), denom = a * e - b * b;
      s = denom > 1e-12 * a * e ? std::clamp((b * f - c * e) / denom, 0.0, 1.0) : 0.0;
      t = (b * s + f) / e;
      if (t < 0) {
        t = 0;
        s = std::clamp(-c / a, 0.0, 1.0);
      } else if (t > 1) {
        t = 1;
        s = std::clamp((b - c) / a, 0.0, 1.0);
      }
    }
  }
  return ((a0 + s * d1) - (b0 + t * d2)).norm();
}

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
      lin_force = (k_line / std::max(1e-4, d_norm)) *
                  deflection_dir(line_robot_obs, line_goal_robot, robot_vel);
    }
  }

  if (min_dist > d_norm) {
    min_dist = d_norm;
  }
  ave_dist += d_norm;
  ave_num += 1;
  return lin_force;
}
