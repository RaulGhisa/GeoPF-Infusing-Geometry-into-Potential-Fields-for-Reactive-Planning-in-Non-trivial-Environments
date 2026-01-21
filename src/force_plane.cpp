#include "../include/geometry_force.h"
#include <random>

Vector3d plane_force(const Vector3d &obs1, const Vector3d &obs2, const Vector3d &obs3, const Vector3d &obs4, const Vector3d &robot_pos, const Vector3d &robot_vel,
                    const Vector3d &goal, const double &k_plane) {
  
  // Vector3d dist_obs = plane_dist_obs(obs1, obs2, obs3, obs4, robot_pos, robot_vel, goal, k_plane);
  Vector3d dist1 = line_dist_obs(obs1, obs2, robot_pos);
  Vector3d dist2 = line_dist_obs(obs2, obs3, robot_pos);
  Vector3d dist3 = line_dist_obs(obs3, obs4, robot_pos);
  Vector3d dist4 = line_dist_obs(obs4, obs1, robot_pos);



  // build the plane
  Vector3d line1 = obs4 - obs2;
  Vector3d line2 = obs4 - obs3;
  Vector3d plane_n = line1.cross(line2);
  plane_n = plane_n/plane_n.norm();
  double dist_point1_plane = obs1.dot(plane_n);
  double dist_robot_plane = robot_pos.dot(plane_n) - dist_point1_plane;
  Vector3d dist_ortho = dist_robot_plane * plane_n;

  // perpendicular root
  Vector3d pp_root = robot_pos - dist_ortho;

  // Check if the perpendicular root is inside the plane
  Matrix<double, 3, 4> full_points;
  full_points.col(0) = obs1;
  full_points.col(1) = obs2;
  full_points.col(2) = obs3;
  full_points.col(3) = obs4;
  Vector3d iden(1,1,1);
  Vector3d min_points = full_points.rowwise().minCoeff() - 1e-4 * iden;
  Vector3d max_points = full_points.rowwise().maxCoeff() + 1e-4 * iden;

  // Comparison 
  Vector3d dist_obs(10000, 10000, 10000);
  Eigen::Array<bool, Eigen::Dynamic, 1> compare_min = min_points.array() < pp_root.array();
  Eigen::Array<bool, Eigen::Dynamic, 1> compare_max = pp_root.array() < max_points.array();


  if ( compare_min.all() && compare_max.all()) {

    // if () {
      dist_obs = dist_ortho;
    // }
  } else {
    Eigen::MatrixXd dist(3, 4);
    dist.col(0) = dist1;
    dist.col(1) = dist2;
    dist.col(2) = dist3;
    dist.col(3) = dist4;

    // Step 2: Calculate norms and store in a 4x1 vector
    Eigen::VectorXd dist_norm(4);
    dist_norm << dist1.norm(), dist2.norm(), dist3.norm(), dist4.norm();

    // Step 3: Find the index of the minimum element in dist_norm
    Eigen::Index minIndex;
    double minNorm = dist_norm.minCoeff(&minIndex);

    // Step 4: Extract the row corresponding to the minimum norm
    dist_obs = dist.col(minIndex);
  }

  Vector3d plan_force(0, 0, 0);
  if (dist_obs.norm() < 0.1) {

    if (robot_vel.norm() != 0) {
      Vector3d line_goal_robot = (goal - robot_pos) / (goal - robot_pos).norm();

      Vector3d line_robot_obs = (dist_obs) / (dist_obs).norm();

      Vector3d force_direction = line_robot_obs - (line_goal_robot * (line_robot_obs.transpose() * line_goal_robot));
      force_direction = force_direction / force_direction.norm();
      plan_force = k_plane / dist_obs.norm() * force_direction;
  
      // ***************************************** check intersection point.  ***************************************** //
      double cos_theta = (-dist_ortho).dot(line_goal_robot) / dist_ortho.norm();
      Vector3d intersection_point = line_goal_robot * dist_ortho.norm() /cos_theta + robot_pos;
      Eigen::Array<bool, Eigen::Dynamic, 1> compare_intersection_min = min_points.array() < intersection_point.array();
      Eigen::Array<bool, Eigen::Dynamic, 1> compare_intersection_max = intersection_point.array() < max_points.array();

      if ( compare_intersection_min.all() && compare_intersection_max.all()) {

        // Eigen::MatrixXd dist(3, 4);
        // dist.col(0) = dist1;
        // dist.col(1) = dist2;
        // dist.col(2) = dist3;
        // dist.col(3) = dist4;

        // Step 2: Calculate norms and store in a 4x1 vector
        Eigen::VectorXd dist_norm(4);
        dist_norm << dist1.norm(), dist2.norm(), dist3.norm(), dist4.norm();

        // Step 3: Find the index of the minimum element in dist_norm
        Eigen::Index minIndex;
        double minNorm = dist_norm.minCoeff(&minIndex);

        Vector3d close_line;
        Vector3d edge_line;

        if (minIndex == 0) {

          close_line = obs1 - obs4;
          edge_line = obs2 - obs1;
        } else if (minIndex == 1) {

          close_line = obs2 - obs1;
          edge_line = obs3 - obs2;
        } else if (minIndex == 2) {

          close_line = obs3 - obs2;
          edge_line = obs4 - obs3;
        } else if (minIndex == 3) {

          close_line = obs4 - obs3;
          edge_line = obs1 - obs4;
        }

        close_line = close_line / close_line.norm();
        edge_line = edge_line / edge_line.norm();
        Vector3d parallel_direction_with_sign = plane_n.cross(edge_line) * sign(close_line.dot(plane_n.cross(edge_line)));
        plan_force = k_plane / dist_obs.norm() * parallel_direction_with_sign;

      }
    }
  }
  if (min_dist > dist_obs.norm())
  {
          min_dist = dist_obs.norm();
  }

  ave_dist = ave_dist + dist_obs.norm();
  ave_num = ave_num + 1;

  return  plan_force;
}




Vector3d plane_dist_obs(const Vector3d &obs1, const Vector3d &obs2, const Vector3d &obs3, const Vector3d &obs4, const Vector3d &robot_pos) {
  
  Vector3d dist1 = line_dist_obs(obs1, obs2, robot_pos);
  Vector3d dist2 = line_dist_obs(obs2, obs3, robot_pos);
  Vector3d dist3 = line_dist_obs(obs3, obs4, robot_pos);
  Vector3d dist4 = line_dist_obs(obs4, obs1, robot_pos);

  // build the plane
  Vector3d line1 = obs4 - obs2;
  Vector3d line2 = obs4 - obs3;
  Vector3d plane_n = line1.cross(line2);
  plane_n = plane_n/plane_n.norm();
  double dist_point1_plane = obs1.dot(plane_n);
  double dist_robot_plane = robot_pos.dot(plane_n) - dist_point1_plane;
  Vector3d dist_ortho = dist_robot_plane * plane_n;

  // perpendicular root
  Vector3d pp_root = robot_pos - dist_ortho;

  // Check if the perpendicular root is inside the plane
  Matrix<double, 3, 4> full_points;
  full_points.col(0) = obs1;
  full_points.col(1) = obs2;
  full_points.col(2) = obs3;
  full_points.col(3) = obs4;
  Vector3d iden(1,1,1);
  Vector3d min_points = full_points.rowwise().minCoeff() - 1e-4 * iden;
  Vector3d max_points = full_points.rowwise().maxCoeff() + 1e-4 * iden;

  // Comparison 
  Vector3d dist_obs(10000, 10000, 10000);
  Eigen::Array<bool, Eigen::Dynamic, 1> compare_min = min_points.array() < pp_root.array();
  Eigen::Array<bool, Eigen::Dynamic, 1> compare_max = pp_root.array() < max_points.array();


  if ( compare_min.all() && compare_max.all()) {

    // if () {
      dist_obs = dist_ortho;
    // }
  } else {
    Eigen::MatrixXd dist(3, 4);
    dist.col(0) = dist1;
    dist.col(1) = dist2;
    dist.col(2) = dist3;
    dist.col(3) = dist4;

    // Step 2: Calculate norms and store in a 4x1 vector
    Eigen::VectorXd dist_norm(4);
    dist_norm << dist1.norm(), dist2.norm(), dist3.norm(), dist4.norm();

    // Step 3: Find the index of the minimum element in dist_norm
    Eigen::Index minIndex;
    double minNorm = dist_norm.minCoeff(&minIndex);

    // Step 4: Extract the row corresponding to the minimum norm
    dist_obs = dist.col(minIndex);
  }
  return dist_obs;
}




int sign(double x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}


Vector4d plane_force_closest_goal(const Vector3d &obs1, const Vector3d &obs2, const Vector3d &obs3, const Vector3d &obs4, const Vector3d &robot_pos, const Vector3d &robot_vel,
                    const Vector3d &goal, const double &k_plane) {
  
  // Vector3d dist_obs = plane_dist_obs(obs1, obs2, obs3, obs4, robot_pos, robot_vel, goal, k_plane);
  Vector3d dist1 = line_dist_obs(obs1, obs2, robot_pos);
  Vector3d dist2 = line_dist_obs(obs2, obs3, robot_pos);
  Vector3d dist3 = line_dist_obs(obs3, obs4, robot_pos);
  Vector3d dist4 = line_dist_obs(obs4, obs1, robot_pos);

  // build the plane
  Vector3d line1 = obs4 - obs2;
  Vector3d line2 = obs4 - obs3;
  Vector3d plane_n = line1.cross(line2);
  plane_n = plane_n/plane_n.norm();
  double dist_point1_plane = obs1.dot(plane_n);
  double dist_robot_plane = robot_pos.dot(plane_n) - dist_point1_plane;
  Vector3d dist_ortho = dist_robot_plane * plane_n;

  // perpendicular root
  Vector3d pp_root = robot_pos - dist_ortho;

  // Check if the perpendicular root is inside the plane
  Matrix<double, 3, 4> full_points;
  full_points.col(0) = obs1;
  full_points.col(1) = obs2;
  full_points.col(2) = obs3;
  full_points.col(3) = obs4;
  Vector3d iden(1,1,1);
  Vector3d min_points = full_points.rowwise().minCoeff() - 1e-4 * iden;
  Vector3d max_points = full_points.rowwise().maxCoeff() + 1e-4 * iden;

  // Comparison 
  Vector3d dist_obs(10000, 10000, 10000);
  Eigen::Array<bool, Eigen::Dynamic, 1> compare_min = min_points.array() < pp_root.array();
  Eigen::Array<bool, Eigen::Dynamic, 1> compare_max = pp_root.array() < max_points.array();


  if ( compare_min.all() && compare_max.all()) {

    // if () {
      dist_obs = dist_ortho;
    // }
  } else {
    Eigen::MatrixXd dist(3, 4);
    dist.col(0) = dist1;
    dist.col(1) = dist2;
    dist.col(2) = dist3;
    dist.col(3) = dist4;

    // Step 2: Calculate norms and store in a 4x1 vector
    Eigen::VectorXd dist_norm(4);
    dist_norm << dist1.norm(), dist2.norm(), dist3.norm(), dist4.norm();

    // Step 3: Find the index of the minimum element in dist_norm
    Eigen::Index minIndex;
    double minNorm = dist_norm.minCoeff(&minIndex);

    // Step 4: Extract the row corresponding to the minimum norm
    dist_obs = dist.col(minIndex);
  }

  Vector3d plan_force(0, 0, 0);
  bool check_intersect;
  if (dist_obs.norm() < 0.1) {

    if (robot_vel.norm() != 0) {
      Vector3d line_goal_robot = (goal - robot_pos) / (goal - robot_pos).norm();

      Vector3d line_robot_obs = (dist_obs) / (dist_obs).norm();
      // std::cout << "line_robot_obs: " << line_robot_obs << std::endl;
      // std::cout << "line_goal_robot: " << line_goal_robot << std::endl;
      // std::cout << "line_robot_obs.transpose() * line_goal_robot" << line_robot_obs.transpose() * line_goal_robot << std::endl;


      Vector3d force_direction = line_robot_obs - (line_goal_robot * (line_robot_obs.transpose() * line_goal_robot));
      force_direction = force_direction / force_direction.norm();
      plan_force = k_plane / dist_obs.norm() * force_direction;
      // std::cout << "force_direction: " << force_direction << std::endl;
      // std::cout << "k_plane: " << k_plane << std::endl;
      // std::cout << "plan_force_1: " << plan_force << std::endl;
  
      // ***************************************** check intersection point.  ***************************************** //
      double cos_theta = (-dist_obs).dot(line_goal_robot) / dist_obs.norm();
      Vector3d intersection_point = line_goal_robot * dist_obs.norm() /cos_theta + robot_pos;
      Eigen::Array<bool, Eigen::Dynamic, 1> compare_intersection_min = min_points.array() < intersection_point.array();
      Eigen::Array<bool, Eigen::Dynamic, 1> compare_intersection_max = intersection_point.array() < max_points.array();
      check_intersect = compare_intersection_min.all() && compare_intersection_max.all();

      if ( check_intersect) {

        Eigen::MatrixXd dist(3, 4);
        dist.col(0) = line_dist_obs(obs1, obs2, goal);
        dist.col(1) = line_dist_obs(obs2, obs3, goal);
        dist.col(2) = line_dist_obs(obs3, obs4, goal);
        dist.col(3) = line_dist_obs(obs4, obs1, goal);

        // Step 2: Calculate norms and store in a 4x1 vector
        Eigen::VectorXd dist_norm(4);
        dist_norm <<(dist.col(0)).norm(), (dist.col(1)).norm(), (dist.col(2)).norm(), dist.col(3).norm();

        // Step 3: Find the index of the minimum element in dist_norm
        Eigen::Index minIndex;
        double minNorm = dist_norm.minCoeff(&minIndex);

        Vector3d close_line;
        Vector3d edge_line;

        if (minIndex == 0) {

          close_line = obs1 - obs4;
          edge_line = obs2 - obs1;
        } else if (minIndex == 1) {

          close_line = obs2 - obs1;
          edge_line = obs3 - obs2;
        } else if (minIndex == 2) {

          close_line = obs3 - obs2;
          edge_line = obs4 - obs3;
          
        } else if (minIndex == 3) {

          close_line = obs4 - obs3;
          edge_line = obs1 - obs4;
        }

        close_line = close_line / close_line.norm();
        edge_line = edge_line / edge_line.norm();
        Vector3d parallel_direction_with_sign = plane_n.cross(edge_line) * sign(close_line.dot(plane_n.cross(edge_line)));
        plan_force = k_plane / dist_obs.norm() * parallel_direction_with_sign;
      }
    }
  }

  Vector4d force_with_check;
  force_with_check.block(0,0,3,1) = plan_force;
  force_with_check(3) = static_cast<int>(check_intersect);

  if (min_dist > dist_obs.norm())
  {
          min_dist = dist_obs.norm();
  }
  ave_dist = ave_dist + dist_obs.norm();
  ave_num = ave_num + 1;
  return  force_with_check;
}

