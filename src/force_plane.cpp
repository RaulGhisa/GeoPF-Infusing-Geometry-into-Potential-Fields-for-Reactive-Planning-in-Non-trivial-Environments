#include "../include/geometry_force.h"
#include <algorithm>
#include <cmath>
#include <limits>

int sign(double x) {
  if (x > 0) return 1;
  if (x < 0) return -1;
  return 0;
}

Quad make_quad(const Vector3d &c1, const Vector3d &c2, const Vector3d &c3, const Vector3d &c4) {
  const Vector3d pts[4] = {c1, c2, c3, c4};
  const Vector3d center = (c1 + c2 + c3 + c4) / 4.0;

  // Normal: the largest of the corner-triangle cross products, so that any
  // corner order (including Z order, where the "diagonals" are parallel
  // edges) gives a usable normal.
  Vector3d n = Vector3d::Zero();
  const Vector3d cand[3] = {(c2 - c1).cross(c3 - c1), (c2 - c1).cross(c4 - c1),
                            (c3 - c1).cross(c4 - c1)};
  for (const Vector3d &c : cand)
    if (c.norm() > n.norm()) n = c;
  n = n.norm() > 1e-12 ? n.normalized() : Vector3d(0, 0, 1);

  // Sort the corners by angle around the centre in a frame on the plane.
  Vector3d u = (c2 - c1) - n * n.dot(c2 - c1);
  if (u.norm() < 1e-12) u = std::abs(n.x()) < 0.9 ? n.cross(Vector3d::UnitX()) : n.cross(Vector3d::UnitY());
  u.normalize();
  const Vector3d v = n.cross(u);
  std::pair<double, int> angles[4];
  for (int i = 0; i < 4; ++i) {
    const Vector3d d = pts[i] - center;
    angles[i] = {std::atan2(d.dot(v), d.dot(u)), i};
  }
  std::sort(angles, angles + 4);

  Quad q;
  for (int i = 0; i < 4; ++i) q.p[i] = pts[angles[i].second];
  // Sorting by increasing angle around n makes the corners counter-clockwise.
  q.n = n;
  return q;
}

// Is x (a point in the rectangle's plane) inside it or on its edge?
static bool quad_contains(const Quad &q, const Vector3d &x) {
  for (int i = 0; i < 4; ++i) {
    const Vector3d edge = q.p[(i + 1) % 4] - q.p[i];
    if (edge.cross(x - q.p[i]).dot(q.n) < -1e-9 * edge.squaredNorm()) return false;
  }
  return true;
}

Vector3d quad_dist_obs(const Quad &q, const Vector3d &robot_pos) {
  const double h = (robot_pos - q.p[0]).dot(q.n);
  if (quad_contains(q, robot_pos - h * q.n)) return h * q.n;

  // Outside the rectangle: the closest point is on one of the 4 edges.
  Vector3d best = line_dist_obs(q.p[0], q.p[1], robot_pos);
  for (int i = 1; i < 4; ++i) {
    const Vector3d d = line_dist_obs(q.p[i], q.p[(i + 1) % 4], robot_pos);
    if (d.squaredNorm() < best.squaredNorm()) best = d;
  }
  return best;
}

bool segment_crosses_quad(const Quad &q, const Vector3d &a, const Vector3d &b) {
  const double ha = (a - q.p[0]).dot(q.n);
  const double hb = (b - q.p[0]).dot(q.n);
  if (!((ha > 0 && hb < 0) || (ha < 0 && hb > 0))) return false;
  return quad_contains(q, a + (ha / (ha - hb)) * (b - a));
}

Vector4d quad_force(const Quad &q, const Vector3d &robot_pos, const Vector3d &robot_vel,
                    const Vector3d &goal, double k_plane, double d_inf, SteerEdge edge) {
  const Vector3d dist_obs = quad_dist_obs(q, robot_pos);
  const double d_norm = dist_obs.norm();

  Vector3d force = Vector3d::Zero();
  bool blocked = false;
  const Vector3d d_goal = goal - robot_pos;
  if (d_norm < d_inf && d_goal.norm() > 1e-9) {
    const Vector3d g = d_goal.normalized();
    const Vector3d n = d_norm > 1e-9 ? Vector3d(dist_obs / d_norm) : q.n;
    const double mag = k_plane / std::max(1e-4, d_norm);
    force = mag * deflection_dir(n, g, robot_vel);

    // Correction heuristic (paper Eq. 12): if the straight path to the goal
    // goes through the rectangle, push parallel to it, across the edge
    // nearest the robot (walls) or nearest the goal (box faces, as in the
    // MATLAB reference PointToCubeForces.m).
    if (segment_crosses_quad(q, robot_pos, goal)) {
      blocked = true;
      const Vector3d &ref = edge == SteerEdge::NearestGoal ? goal : robot_pos;
      int best = 0;
      double best_d = std::numeric_limits<double>::infinity();
      for (int i = 0; i < 4; ++i) {
        const double d = line_dist_obs(q.p[i], q.p[(i + 1) % 4], ref).squaredNorm();
        if (d < best_d) { best_d = d; best = i; }
      }
      const Vector3d edge_v = (q.p[(best + 1) % 4] - q.p[best]).normalized();
      const Vector3d close_v = (q.p[best] - q.p[(best + 3) % 4]).normalized();
      Vector3d steer = q.n.cross(edge_v);
      if (close_v.dot(steer) < 0) steer = -steer;
      force = mag * steer;
    }
  }

  if (min_dist > d_norm) min_dist = d_norm;
  ave_dist += d_norm;
  ave_num += 1;

  Vector4d out;
  out << force, blocked ? 1.0 : 0.0;
  return out;
}

Vector3d plane_dist_obs(const Vector3d &obs1, const Vector3d &obs2, const Vector3d &obs3, const Vector3d &obs4, const Vector3d &robot_pos) {
  return quad_dist_obs(make_quad(obs1, obs2, obs3, obs4), robot_pos);
}

Vector3d plane_force(const Vector3d &obs1, const Vector3d &obs2, const Vector3d &obs3, const Vector3d &obs4, const Vector3d &robot_pos, const Vector3d &robot_vel,
                    const Vector3d &goal, const double &k_plane, double d_inf) {
  return quad_force(make_quad(obs1, obs2, obs3, obs4), robot_pos, robot_vel, goal, k_plane,
                    d_inf, SteerEdge::NearestRobot)
      .head<3>();
}

Vector4d plane_force_closest_goal(const Vector3d &obs1, const Vector3d &obs2, const Vector3d &obs3, const Vector3d &obs4, const Vector3d &robot_pos, const Vector3d &robot_vel,
                    const Vector3d &goal, const double &k_plane, double d_inf) {
  return quad_force(make_quad(obs1, obs2, obs3, obs4), robot_pos, robot_vel, goal, k_plane,
                    d_inf, SteerEdge::NearestGoal);
}
