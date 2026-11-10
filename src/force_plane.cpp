#include "../include/geometry_force.h"
#include <algorithm>
#include <cmath>
#include <vector>

int sign(double x) {
  if (x > 0) return 1;
  if (x < 0) return -1;
  return 0;
}

// Helper: order 4 coplanar points in perimeter order around their normal
static void get_ordered_plane(const Vector3d &in1, const Vector3d &in2, const Vector3d &in3, const Vector3d &in4,
                              Vector3d &p1, Vector3d &p2, Vector3d &p3, Vector3d &p4, Vector3d &plane_n) {
  Vector3d pts[4] = {in1, in2, in3, in4};
  Vector3d center = (pts[0] + pts[1] + pts[2] + pts[3]) / 4.0;
  
  // Normal from cross products of diagonals
  Vector3d n = (pts[2] - pts[0]).cross(pts[3] - pts[1]);
  if (n.norm() < 1e-9) {
    n = (pts[1] - pts[0]).cross(pts[2] - pts[0]);
  }
  if (n.norm() < 1e-9) {
    n = Vector3d(0, 0, 1);
  } else {
    n.normalize();
  }
  plane_n = n;

  // Tangent frame on plane
  Vector3d u = pts[1] - pts[0];
  u = u - plane_n * (plane_n.dot(u));
  if (u.norm() < 1e-9) {
    if (std::abs(plane_n.x()) < 0.9) u = plane_n.cross(Vector3d(1, 0, 0));
    else u = plane_n.cross(Vector3d(0, 1, 0));
  }
  u.normalize();
  Vector3d v = plane_n.cross(u).normalized();

  // Sort by angle around center
  std::pair<double, int> angles[4];
  for (int i = 0; i < 4; ++i) {
    Vector3d d = pts[i] - center;
    angles[i] = {std::atan2(d.dot(v), d.dot(u)), i};
  }
  std::sort(angles, angles + 4);

  p1 = pts[angles[0].second];
  p2 = pts[angles[1].second];
  p3 = pts[angles[2].second];
  p4 = pts[angles[3].second];

  // Ensure plane_n follows right-hand rule for p1->p2->p3
  Vector3d cross12 = (p2 - p1).cross(p3 - p2);
  if (cross12.dot(plane_n) < 0) {
    plane_n = -plane_n;
  }
}

// Check if a point lying in the plane of the quad is inside its perimeter
static bool is_point_inside_quad(const Vector3d &X, const Vector3d &p1, const Vector3d &p2,
                                 const Vector3d &p3, const Vector3d &p4, const Vector3d &plane_n) {
  const Vector3d p[4] = {p1, p2, p3, p4};
  for (int i = 0; i < 4; ++i) {
    Vector3d edge = p[(i + 1) % 4] - p[i];
    Vector3d to_pt = X - p[i];
    if ((edge.cross(to_pt)).dot(plane_n) < -1e-6) {
      return false;
    }
  }
  return true;
}

Vector3d plane_dist_obs(const Vector3d &obs1, const Vector3d &obs2, const Vector3d &obs3, const Vector3d &obs4, const Vector3d &robot_pos) {
  Vector3d p1, p2, p3, p4, plane_n;
  get_ordered_plane(obs1, obs2, obs3, obs4, p1, p2, p3, p4, plane_n);

  double h = (robot_pos - p1).dot(plane_n);
  Vector3d pp_root = robot_pos - h * plane_n;

  if (is_point_inside_quad(pp_root, p1, p2, p3, p4, plane_n)) {
    return h * plane_n; // Vector from plane surface to robot
  }

  // Outside the rectangle: closest point is on one of the 4 edges
  Vector3d d1 = line_dist_obs(p1, p2, robot_pos);
  Vector3d d2 = line_dist_obs(p2, p3, robot_pos);
  Vector3d d3 = line_dist_obs(p3, p4, robot_pos);
  Vector3d d4 = line_dist_obs(p4, p1, robot_pos);

  Vector3d best_d = d1;
  double min_sq = d1.squaredNorm();
  if (d2.squaredNorm() < min_sq) { min_sq = d2.squaredNorm(); best_d = d2; }
  if (d3.squaredNorm() < min_sq) { min_sq = d3.squaredNorm(); best_d = d3; }
  if (d4.squaredNorm() < min_sq) { min_sq = d4.squaredNorm(); best_d = d4; }

  return best_d;
}

Vector3d plane_force(const Vector3d &obs1, const Vector3d &obs2, const Vector3d &obs3, const Vector3d &obs4, const Vector3d &robot_pos, const Vector3d &robot_vel,
                    const Vector3d &goal, const double &k_plane, double d_inf) {
  Vector3d dist_obs = plane_dist_obs(obs1, obs2, obs3, obs4, robot_pos);
  double d_norm = dist_obs.norm();

  Vector3d plan_force(0, 0, 0);
  if (d_norm < d_inf) {
    Vector3d d_goal = goal - robot_pos;
    double d_goal_norm = d_goal.norm();
    if (d_goal_norm > 1e-9) {
      Vector3d line_goal_robot = d_goal / d_goal_norm;
      Vector3d line_robot_obs;
      if (d_norm > 1e-9) line_robot_obs = dist_obs / d_norm;
      else line_robot_obs = Vector3d(0, 0, 1);

      Vector3d force_direction = line_robot_obs - (line_goal_robot * (line_robot_obs.dot(line_goal_robot)));
      if (force_direction.norm() < 1e-4) {
        Vector3d v_perp = robot_vel - line_goal_robot * (line_goal_robot.dot(robot_vel));
        if (v_perp.norm() > 1e-4) force_direction = v_perp.normalized();
        else if (std::abs(line_goal_robot.x()) < 0.9) force_direction = line_goal_robot.cross(Vector3d(1, 0, 0)).normalized();
        else force_direction = line_goal_robot.cross(Vector3d(0, 1, 0)).normalized();
      } else {
        force_direction.normalize();
      }

      plan_force = (k_plane / std::max(1e-4, d_norm)) * force_direction;

      // Check intersection: does the segment from robot_pos to goal cross the rectangle?
      Vector3d p1, p2, p3, p4, plane_n;
      get_ordered_plane(obs1, obs2, obs3, obs4, p1, p2, p3, p4, plane_n);

      double h_robot = (robot_pos - p1).dot(plane_n);
      double h_goal = (goal - p1).dot(plane_n);

      // Only intersects if robot and goal are on opposite sides of the plane
      if (h_robot * h_goal < -1e-6) {
        double alpha = h_robot / (h_robot - h_goal);
        if (alpha >= 0.0 && alpha <= 1.0) {
          Vector3d intersection_point = robot_pos + alpha * (goal - robot_pos);
          if (is_point_inside_quad(intersection_point, p1, p2, p3, p4, plane_n)) {
            // Path to goal is blocked by this rectangle!
            Vector3d d_edges[4] = {
              line_dist_obs(p1, p2, robot_pos),
              line_dist_obs(p2, p3, robot_pos),
              line_dist_obs(p3, p4, robot_pos),
              line_dist_obs(p4, p1, robot_pos)
            };
            int minIdx = 0;
            double minD = d_edges[0].squaredNorm();
            for (int k = 1; k < 4; ++k) {
              if (d_edges[k].squaredNorm() < minD) {
                minD = d_edges[k].squaredNorm();
                minIdx = k;
              }
            }
            Vector3d edge_v, close_v;
            if (minIdx == 0) { edge_v = p2 - p1; close_v = p1 - p4; }
            else if (minIdx == 1) { edge_v = p3 - p2; close_v = p2 - p1; }
            else if (minIdx == 2) { edge_v = p4 - p3; close_v = p3 - p2; }
            else { edge_v = p1 - p4; close_v = p4 - p3; }

            edge_v.normalize();
            close_v.normalize();
            Vector3d steer = plane_n.cross(edge_v);
            if (close_v.dot(steer) < 0) steer = -steer;
            plan_force = (k_plane / std::max(1e-4, d_norm)) * steer;
          }
        }
      }
    }
  }

  if (min_dist > d_norm) {
    min_dist = d_norm;
  }
  ave_dist += d_norm;
  ave_num += 1;

  return plan_force;
}

Vector4d plane_force_closest_goal(const Vector3d &obs1, const Vector3d &obs2, const Vector3d &obs3, const Vector3d &obs4, const Vector3d &robot_pos, const Vector3d &robot_vel,
                    const Vector3d &goal, const double &k_plane, double d_inf) {
  Vector3d dist_obs = plane_dist_obs(obs1, obs2, obs3, obs4, robot_pos);
  double d_norm = dist_obs.norm();

  Vector3d plan_force(0, 0, 0);
  bool check_intersect = false; // Always explicitly initialized

  if (d_norm < d_inf) {
    Vector3d d_goal = goal - robot_pos;
    double d_goal_norm = d_goal.norm();
    if (d_goal_norm > 1e-9) {
      Vector3d line_goal_robot = d_goal / d_goal_norm;
      Vector3d line_robot_obs;
      if (d_norm > 1e-9) line_robot_obs = dist_obs / d_norm;
      else line_robot_obs = Vector3d(0, 0, 1);

      Vector3d force_direction = line_robot_obs - (line_goal_robot * (line_robot_obs.dot(line_goal_robot)));
      if (force_direction.norm() < 1e-4) {
        Vector3d v_perp = robot_vel - line_goal_robot * (line_goal_robot.dot(robot_vel));
        if (v_perp.norm() > 1e-4) force_direction = v_perp.normalized();
        else if (std::abs(line_goal_robot.x()) < 0.9) force_direction = line_goal_robot.cross(Vector3d(1, 0, 0)).normalized();
        else force_direction = line_goal_robot.cross(Vector3d(0, 1, 0)).normalized();
      } else {
        force_direction.normalize();
      }

      plan_force = (k_plane / std::max(1e-4, d_norm)) * force_direction;

      // Check intersection
      Vector3d p1, p2, p3, p4, plane_n;
      get_ordered_plane(obs1, obs2, obs3, obs4, p1, p2, p3, p4, plane_n);

      double h_robot = (robot_pos - p1).dot(plane_n);
      double h_goal = (goal - p1).dot(plane_n);

      if (h_robot * h_goal < -1e-6) {
        double alpha = h_robot / (h_robot - h_goal);
        if (alpha >= 0.0 && alpha <= 1.0) {
          Vector3d intersection_point = robot_pos + alpha * (goal - robot_pos);
          if (is_point_inside_quad(intersection_point, p1, p2, p3, p4, plane_n)) {
            check_intersect = true;
            Vector3d d_edges[4] = {
              line_dist_obs(p1, p2, robot_pos),
              line_dist_obs(p2, p3, robot_pos),
              line_dist_obs(p3, p4, robot_pos),
              line_dist_obs(p4, p1, robot_pos)
            };
            int minIdx = 0;
            double minD = d_edges[0].squaredNorm();
            for (int k = 1; k < 4; ++k) {
              if (d_edges[k].squaredNorm() < minD) {
                minD = d_edges[k].squaredNorm();
                minIdx = k;
              }
            }
            Vector3d edge_v, close_v;
            if (minIdx == 0) { edge_v = p2 - p1; close_v = p1 - p4; }
            else if (minIdx == 1) { edge_v = p3 - p2; close_v = p2 - p1; }
            else if (minIdx == 2) { edge_v = p4 - p3; close_v = p3 - p2; }
            else { edge_v = p1 - p4; close_v = p4 - p3; }

            edge_v.normalize();
            close_v.normalize();
            Vector3d steer = plane_n.cross(edge_v);
            if (close_v.dot(steer) < 0) steer = -steer;
            plan_force = (k_plane / std::max(1e-4, d_norm)) * steer;
          }
        }
      }
    }
  }

  Vector4d force_with_check;
  force_with_check.block<3, 1>(0, 0) = plan_force;
  force_with_check(3) = check_intersect ? 1.0 : 0.0;

  if (min_dist > d_norm) {
    min_dist = d_norm;
  }
  ave_dist += d_norm;
  ave_num += 1;

  return force_with_check;
}
