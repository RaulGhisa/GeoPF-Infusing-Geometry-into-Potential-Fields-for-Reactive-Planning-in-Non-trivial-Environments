#include "../include/geometry_force.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>
#include <Eigen/Dense>

Box make_box(const Vector3d (&c)[8]) {
  // From corner c[0], find three mutually perpendicular edges e0, e1, e2 such
  // that c[0] + any 0/1 combination of them is one of the corners. Trying the
  // GeoPF order first (corner2, corner4, corner5) keeps such input unchanged.
  Vector3d v[8];
  double scale = 0;
  for (int i = 0; i < 8; ++i) {
    v[i] = c[i] - c[0];
    scale = std::max(scale, v[i].norm());
  }
  const double tol = 1e-4 * scale;  // loose enough for float32 exports
  auto perp = [](const Vector3d &a, const Vector3d &b) {
    return std::abs(a.dot(b)) <= 1e-4 * a.norm() * b.norm();
  };
  auto find_corner = [&](const Vector3d &x) {
    for (int i = 0; i < 8; ++i)
      if ((c[i] - x).norm() <= tol) return i;
    return -1;
  };

  std::vector<std::array<int, 3>> triples = {{1, 3, 4}};
  for (int i = 1; i < 8; ++i)
    for (int j = i + 1; j < 8; ++j)
      for (int k = j + 1; k < 8; ++k) triples.push_back({i, j, k});

  // (e0, e1, e2) multiples of each GeoPF corner.
  static const int kCombo[8][3] = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
                                   {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};
  Box b;
  bool found = false;
  for (const auto &t : triples) {
    const Vector3d e[3] = {v[t[0]], v[t[1]], v[t[2]]};
    if (e[0].norm() <= tol || e[1].norm() <= tol || e[2].norm() <= tol) continue;
    if (!perp(e[0], e[1]) || !perp(e[0], e[2]) || !perp(e[1], e[2])) continue;
    bool ok = true;
    for (int m = 0; m < 8 && ok; ++m) {
      const int idx = find_corner(c[0] + kCombo[m][0] * e[0] + kCombo[m][1] * e[1] +
                                  kCombo[m][2] * e[2]);
      if (idx < 0) ok = false;
      else b.corner[m] = c[idx];
    }
    if (ok) { found = true; break; }
  }
  if (!found) {
    // Not a rectangular box: fall back to taking the corners as given.
    static bool warned = false;
    if (!warned) {
      warned = true;
      std::cerr << "geopf: warning: cubic corners do not form a rectangular box; "
                   "using them in the given order\n";
    }
    for (int m = 0; m < 8; ++m) b.corner[m] = c[m];
  }

  const Vector3d *p = b.corner;
  b.center = Vector3d::Zero();
  for (int m = 0; m < 8; ++m) b.center += p[m] / 8.0;
  const Vector3d edges[3] = {p[1] - p[0], p[3] - p[0], p[4] - p[0]};
  for (int k = 0; k < 3; ++k) {
    b.half[k] = edges[k].norm() / 2.0;
    b.axis[k] = b.half[k] > 0 ? Vector3d(edges[k].normalized()) : Vector3d::Zero();
  }
  // Faces in the order of the original code (1234, 5678, 1265, 5148, 4378, 2673).
  b.face[0] = make_quad(p[0], p[1], p[2], p[3]);
  b.face[1] = make_quad(p[4], p[5], p[6], p[7]);
  b.face[2] = make_quad(p[0], p[1], p[5], p[4]);
  b.face[3] = make_quad(p[4], p[0], p[3], p[7]);
  b.face[4] = make_quad(p[3], p[2], p[6], p[7]);
  b.face[5] = make_quad(p[1], p[5], p[6], p[2]);
  return b;
}

double box_signed_distance(const Box &b, const Vector3d &p) {
  const Vector3d r = p - b.center;
  Vector3d q;
  for (int k = 0; k < 3; ++k) q(k) = std::abs(r.dot(b.axis[k])) - b.half[k];
  return q.cwiseMax(0.0).norm() + std::min(q.maxCoeff(), 0.0);
}

bool segment_hits_box(const Box &b, const Vector3d &a, const Vector3d &b_end) {
  // Clip the segment against the three slabs of the box.
  double lo = 0.0, hi = 1.0;
  const Vector3d d = b_end - a;
  for (int k = 0; k < 3; ++k) {
    const double s = (a - b.center).dot(b.axis[k]);
    const double ds = d.dot(b.axis[k]);
    if (std::abs(ds) < 1e-300) {
      if (std::abs(s) > b.half[k]) return false;
      continue;
    }
    double t0 = (-b.half[k] - s) / ds, t1 = (b.half[k] - s) / ds;
    if (t0 > t1) std::swap(t0, t1);
    lo = std::max(lo, t0);
    hi = std::min(hi, t1);
    if (lo > hi) return false;
  }
  return true;
}

Vector3d box_force(const Box &b, const Vector3d &robot_pos, const Vector3d &robot_vel,
                   const Vector3d &goal, double k_cubic, double d_inf) {
  const double sd = box_signed_distance(b, robot_pos);
  if (sd <= 0 && min_dist > sd) min_dist = sd;  // inside: negative distance

  // Nearest face(s); ties (edges, corners) are averaged below.
  VectorXd norm_dist_obs(6);
  for (int i = 0; i < 6; ++i) norm_dist_obs(i) = quad_dist_obs(b.face[i], robot_pos).norm();
  const VectorXd minIndexes = findAllMinIndices(norm_dist_obs);

  Eigen::Matrix<double, 4, Eigen::Dynamic> with_check(4, minIndexes.size());
  for (Eigen::Index i = 0; i < minIndexes.size(); ++i) {
    const int f = static_cast<int>(minIndexes(i));
    with_check.col(i) = quad_force(b.face[f], robot_pos, robot_vel, goal, k_cubic, d_inf,
                                   SteerEdge::NearestGoal);
  }

  // As in the original: among tied faces, prefer those that do not block the path.
  const VectorXd keep = findAllMinIndices(with_check.row(3).transpose());
  Vector3d sum = Vector3d::Zero();
  for (Eigen::Index i = 0; i < keep.size(); ++i)
    sum += with_check.block<3, 1>(0, static_cast<int>(keep(i)));
  return sum / static_cast<double>(keep.size());
}

Vector3d cubic_force(const Vector3d &obs1, const Vector3d &obs2, const Vector3d &obs3, const Vector3d &obs4,
                    const Vector3d &obs5, const Vector3d &obs6, const Vector3d &obs7, const Vector3d &obs8,
                    const Vector3d &robot_pos, const Vector3d &robot_vel,
                    const Vector3d &goal, const double &k_cubic, double d_inf) {
  const Vector3d corners[8] = {obs1, obs2, obs3, obs4, obs5, obs6, obs7, obs8};
  return box_force(make_box(corners), robot_pos, robot_vel, goal, k_cubic, d_inf);
}
