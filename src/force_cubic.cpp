#include "../include/geometry_force.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

Vector3d cubic_force(const Vector3d &obs1, const Vector3d &obs2, const Vector3d &obs3, const Vector3d &obs4, 
                    const Vector3d &obs5, const Vector3d &obs6, const Vector3d &obs7, const Vector3d &obs8, 
                    const Vector3d &robot_pos, const Vector3d &robot_vel,
                    const Vector3d &goal, const double &k_cubic, double d_inf) {

  // Signed distance check for interior of box
  Vector3d center = (obs1 + obs2 + obs3 + obs4 + obs5 + obs6 + obs7 + obs8) / 8.0;
  Vector3d u1 = obs2 - obs1; double h1 = u1.norm() / 2.0; if (h1 > 1e-9) u1.normalize();
  Vector3d u2 = obs4 - obs1; double h2 = u2.norm() / 2.0; if (h2 > 1e-9) u2.normalize();
  Vector3d u3 = obs5 - obs1; double h3 = u3.norm() / 2.0; if (h3 > 1e-9) u3.normalize();

  double q1 = std::abs((robot_pos - center).dot(u1)) - h1;
  double q2 = std::abs((robot_pos - center).dot(u2)) - h2;
  double q3 = std::abs((robot_pos - center).dot(u3)) - h3;

  if (q1 <= 0 && q2 <= 0 && q3 <= 0) {
    // Inside the box! Signed distance is negative
    double sd = std::max(q1, std::max(q2, q3));
    if (min_dist > sd) {
      min_dist = sd;
    }
  }

  Matrix<double, 3, 6> robot_obs;
  robot_obs.col(0) = plane_dist_obs(obs1, obs2, obs3, obs4, robot_pos);
  robot_obs.col(1) = plane_dist_obs(obs5, obs6, obs7, obs8, robot_pos);
  robot_obs.col(2) = plane_dist_obs(obs1, obs2, obs6, obs5, robot_pos);
  robot_obs.col(3) = plane_dist_obs(obs5, obs1, obs4, obs8, robot_pos);
  robot_obs.col(4) = plane_dist_obs(obs4, obs3, obs7, obs8, robot_pos);
  robot_obs.col(5) = plane_dist_obs(obs2, obs6, obs7, obs3, robot_pos);

  VectorXd norm_dist_obs(6); 
  norm_dist_obs = robot_obs.colwise().norm();
  VectorXd minIndexes = findAllMinIndices(norm_dist_obs);

  Eigen::Matrix<double, 4, Eigen::Dynamic> cubic_force_with_check(4, minIndexes.size());

  for (size_t i = 0; i < minIndexes.size(); i++) {
      Eigen::Index minIndex = minIndexes(i);

      Vector3d plane_point1;
      Vector3d plane_point2;
      Vector3d plane_point3;
      Vector3d plane_point4;

      if (minIndex == 0) {
        plane_point1 = obs1; plane_point2 = obs2; plane_point3 = obs3; plane_point4 = obs4;
      } else if (minIndex == 1) {
        plane_point1 = obs5; plane_point2 = obs6; plane_point3 = obs7; plane_point4 = obs8;
      } else if (minIndex == 2) {
        // Face 2 is (obs1, obs2, obs6, obs5)
        plane_point1 = obs1; plane_point2 = obs2; plane_point3 = obs6; plane_point4 = obs5;
      } else if (minIndex == 3) {
        // Face 3 is (obs5, obs1, obs4, obs8)
        plane_point1 = obs5; plane_point2 = obs1; plane_point3 = obs4; plane_point4 = obs8;
      } else if (minIndex == 4) {
        // Face 4 is (obs4, obs3, obs7, obs8)
        plane_point1 = obs4; plane_point2 = obs3; plane_point3 = obs7; plane_point4 = obs8;
      } else if (minIndex == 5) {
        // Face 5 is (obs2, obs6, obs7, obs3)
        plane_point1 = obs2; plane_point2 = obs6; plane_point3 = obs7; plane_point4 = obs3;
      } else {
        std::cerr << "no correct cubic plane finding" << std::endl;
        abort();
      }

      cubic_force_with_check.col(i) = plane_force_closest_goal(plane_point1, plane_point2, plane_point3, plane_point4,
                                                              robot_pos, robot_vel, goal, k_cubic, d_inf);
  }

  VectorXd intersect_check_for_all_plane = findAllMinIndices(cubic_force_with_check.row(3));

  Eigen::Matrix<double, 3, Eigen::Dynamic> cubic_force_after_check(3, intersect_check_for_all_plane.size());
  for (size_t i = 0; i < intersect_check_for_all_plane.size(); i++) {
    int idx = static_cast<int>(intersect_check_for_all_plane(i));
    cubic_force_after_check.col(i) = cubic_force_with_check.block(0, idx, 3, 1);
  }
  
  return cubic_force_after_check.rowwise().mean();
}
