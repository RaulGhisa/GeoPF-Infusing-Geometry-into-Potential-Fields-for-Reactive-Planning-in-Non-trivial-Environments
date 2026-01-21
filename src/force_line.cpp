#include "../include/geometry_force.h"
#include <random>

Vector3d line_force(const Vector3d &obs_start, const Vector3d &obs_end, const Vector3d &robot_pos, const Vector3d &robot_vel,
                    const Vector3d &goal, const double &k_line) {

  Vector3d dist_obs = line_dist_obs(obs_start, obs_end, robot_pos);

  Vector3d lin_force(0, 0, 0);
  if (dist_obs.norm() < 0.05) {

    if (robot_vel.norm() != 0) {
        Vector3d line_goal_robot = (goal - robot_pos) / (goal - robot_pos).norm();
        Vector3d line_robot_obs =
          (dist_obs) / (dist_obs).norm();

        Vector3d force_direction = line_robot_obs - (line_goal_robot * (line_robot_obs.transpose() * line_goal_robot));
        force_direction = force_direction / force_direction.norm();

        // if (dist_obs.norm() < 1e-10)
        // {
        //         std::random_device rd;  // Random seed
        //         std::mt19937 gen(rd()); // Mersenne Twister engine
        //         std::uniform_real_distribution<> dis(-1.0, 1.0);
        //         for (int i = 0; i < obs_start.size(); ++i) {
        //                 force_direction(i) = dis(gen);
        //         }
        //         Vector3d minus_one;
        //         minus_one.setConstant(-1);
        //         force_direction = minus_one + 2 * force_direction;
        //         force_direction = force_direction /
        //         force_direction.norm();
        // }

        lin_force = k_line / dist_obs.norm() * force_direction;
    }
  }
  if (min_dist > dist_obs.norm())
  {
          min_dist = dist_obs.norm();
  }
  ave_dist = ave_dist + dist_obs.norm();
  ave_num = ave_num + 1;
  return lin_force;
}



Vector3d line_dist_obs(const Vector3d &obs_start, const Vector3d &obs_end, const Vector3d &robot_pos) {

  Vector3d line_direction = (obs_start - obs_end) / (obs_start - obs_end).norm();

  Vector3d moment = obs_start.cross(line_direction);

  // Robot to line perpendicular distance
  Vector3d line_robot_to_obs_start = (robot_pos - obs_start) / (robot_pos - obs_start).norm();
  Vector3d line_robot_to_line_ortho = (line_direction.cross(line_robot_to_obs_start)).cross(line_direction);
  line_robot_to_line_ortho = line_robot_to_line_ortho / line_robot_to_line_ortho.norm();
  Vector3d dist_ortho = (robot_pos.cross(line_direction) - moment).norm() * line_robot_to_line_ortho;

  // Perpendicular root
  Vector3d pp_root = robot_pos - dist_ortho;

  // Robot to obs start distance
  Vector3d dist_2 = robot_pos - obs_start;

  // Robot to obs end distance
  Vector3d dist_3 = robot_pos - obs_end;

  Vector3d dist_obs(10000, 10000, 10000);
  if ((pp_root - obs_start).norm() < (obs_end - obs_start).norm() && (pp_root - obs_end).norm() < (obs_end - obs_start).norm()) {

    // if () {

      dist_obs = dist_ortho;

    // }
  } else {

    if (dist_2.norm() <= dist_3.norm()) {

      dist_obs = dist_2;
    } else {

      dist_obs = dist_3;
    }
  }

  return dist_obs;
}
