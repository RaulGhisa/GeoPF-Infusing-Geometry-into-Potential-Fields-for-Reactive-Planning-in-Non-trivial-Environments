#include "../include/geometry_force.h"
#include <random>

#include <iostream>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>

Vector3d cubic_force(const Vector3d &obs1, const Vector3d &obs2, const Vector3d &obs3, const Vector3d &obs4, 
                    const Vector3d &obs5, const Vector3d &obs6, const Vector3d &obs7, const Vector3d &obs8, 
                    const Vector3d &robot_pos, const Vector3d &robot_vel,
                    const Vector3d &goal, const double &k_cubic) {

  Matrix<double, 3, 6> robot_obs;
  robot_obs.col(0) = plane_dist_obs(obs1, obs2, obs3, obs4, robot_pos);
  robot_obs.col(1) = plane_dist_obs(obs5, obs6, obs7, obs8, robot_pos);
  robot_obs.col(2) = plane_dist_obs(obs1, obs2, obs6, obs5, robot_pos);
  robot_obs.col(3) = plane_dist_obs(obs5, obs1, obs4, obs8, robot_pos);
  robot_obs.col(4) = plane_dist_obs(obs4, obs3, obs7, obs8, robot_pos);
  robot_obs.col(5) = plane_dist_obs(obs2, obs6, obs7, obs3, robot_pos);
  // std::cout << "obs2: " << obs2 << "obs6: " << obs6 << "obs7: " << obs7 << "obs3: " << obs3 << std::endl;
  // std::cout << "obs1: " << obs1 << "obs2: " << obs2 << "obs6: " << obs6 << "obs5: " << obs5 << std::endl;


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

        plane_point1 = obs1;
        plane_point2 = obs2;
        plane_point3 = obs3;
        plane_point4 = obs4;
      } else if (minIndex == 1) {

        plane_point1 = obs5;
        plane_point2 = obs6;
        plane_point3 = obs7;
        plane_point4 = obs8;
      } else if (minIndex == 2) {

        plane_point1 = obs2;
        plane_point2 = obs6;
        plane_point3 = obs7;
        plane_point4 = obs3;
      } else if (minIndex == 3) {

        plane_point1 = obs5;
        plane_point2 = obs1;
        plane_point3 = obs4;
        plane_point4 = obs8;
        // abort();

      } else if (minIndex == 4) {

        plane_point1 = obs4;
        plane_point2 = obs3;
        plane_point3 = obs7;
        plane_point4 = obs8;
      } else if (minIndex == 5) {

        plane_point1 = obs1;
        plane_point2 = obs2;
        plane_point3 = obs6;
        plane_point4 = obs5;
      } else {

        std::cout << "no correct cubic plane finding" << std::endl;
        abort();
      }
  

      cubic_force_with_check.col(i) = plane_force_closest_goal(plane_point1, plane_point2, plane_point3, plane_point4, robot_pos, robot_vel, goal, k_cubic);
  }

  VectorXd intersect_check_for_all_plane = findAllMinIndices(cubic_force_with_check.row(3));

  Eigen::Matrix<double, 3, Eigen::Dynamic> cubic_force_after_check(3, intersect_check_for_all_plane.size());
  for (size_t i = 0; i < intersect_check_for_all_plane.size(); i++) {

    cubic_force_after_check.col(i) = cubic_force_with_check.block(0,i,3,1);
  }
  
  return cubic_force_after_check.rowwise().mean();

}