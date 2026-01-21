
#include "../include/geometry_force.h"
#include <random>

#include <iostream>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>

Vector3d cylinder_force(const Vector3d &obs_start, const Vector3d &obs_end, const double &obs_rad, 
                        const Vector3d &robot_pos, const Vector3d &robot_vel,
                        const Vector3d &goal, const double &k_cylinder) {

  Vector3d cylinder_axis = (obs_start - obs_end)/(obs_start - obs_end).norm();
  Vector3d moment = obs_start.cross(cylinder_axis);

  // Robot to line perpendicular distance
  Vector3d line_robot_to_obs_start = (robot_pos - obs_start) / (robot_pos - obs_start).norm();
  Vector3d line_robot_to_line_ortho = (cylinder_axis.cross(line_robot_to_obs_start)).cross(cylinder_axis);
  line_robot_to_line_ortho = line_robot_to_line_ortho / line_robot_to_line_ortho.norm();
  Vector3d dist_ortho = ((robot_pos.cross(cylinder_axis) - moment).norm()-obs_rad) * line_robot_to_line_ortho;

  // Perpendicular root
  Vector3d pp_root = robot_pos - dist_ortho;

  // Robot to obs start distance, bring forward the line to surface of cylinder
  Vector3d start_edge = obs_start + line_robot_to_line_ortho * obs_rad;
  Vector3d dist_2 = robot_pos - start_edge;

  // Robot to obs end distance, bring forward the line to surface of cylinder
  Vector3d end_edge = obs_end + line_robot_to_line_ortho * obs_rad;
  Vector3d dist_3 = robot_pos - end_edge;

  Vector3d dist_obs(10000, 10000, 10000);
  Vector3d circle_center;
  Vector3d plane_n;
  bool circle_plane_inter_check;
  bool plane_root_check = ((pp_root - start_edge).norm() <= (start_edge - end_edge).norm()) && ((pp_root - end_edge).norm() <= (start_edge - end_edge).norm());


  if (plane_root_check) {

      dist_obs = dist_ortho;
      circle_plane_inter_check = false;
  } else if (cylinder_axis.dot(line_robot_to_obs_start) < 0.99) {
      
      if (dist_2.norm() < dist_3.norm()) {
        
        circle_center = obs_start;
        plane_n = cylinder_axis;
        
        double dist_ortho_circle_norm = robot_pos.dot(plane_n) - circle_center.dot(plane_n);
        Vector3d dist_ortho_circle = dist_ortho_circle_norm * plane_n;
        Vector3d pp_root_circle = robot_pos - dist_ortho_circle;
        if ((pp_root_circle - circle_center).norm() < obs_rad) {
          
          dist_obs = dist_ortho_circle;

        } else {
          
          dist_obs = dist_2;
        }
      } else {

        circle_center = obs_end;
        plane_n = - cylinder_axis;

        double dist_ortho_circle_norm = robot_pos.dot(plane_n) - circle_center.dot(plane_n);
        Vector3d dist_ortho_circle = dist_ortho_circle_norm * plane_n;
        Vector3d pp_root_circle = robot_pos - dist_ortho_circle;
        if ((pp_root_circle - circle_center).norm() < obs_rad) {
          
          dist_obs = dist_ortho_circle;
        } else {
          
          dist_obs = dist_2;
        }
      }
      circle_plane_inter_check = true;
  } else {

    if (dist_2.norm() <= dist_3.norm()) {

      dist_obs = dist_2;
      circle_center = obs_start;

    } else {

      dist_obs = dist_3;
      circle_center = obs_end;
    }
    circle_plane_inter_check = true;
  }

  Vector3d cylin_force(0,0,0);
  if (dist_obs.norm() < 0.1)
  {

    if (robot_vel.norm() != 0) {

      Vector3d line_goal_robot = (goal - robot_pos) / (goal - robot_pos).norm();
      Vector3d line_robot_obs = (dist_obs) / (dist_obs).norm();

      Vector3d force_direction = line_robot_obs - (line_goal_robot * (line_robot_obs.transpose() * line_goal_robot));
      force_direction = force_direction / force_direction.norm();
      cylin_force = k_cylinder / dist_obs.norm() * force_direction;
      if (circle_plane_inter_check) {

        Vector3d plane_point2 = circle_center + line_robot_to_line_ortho * obs_rad;
        Vector3d other_direction = line_robot_to_line_ortho.cross(cylinder_axis);
        other_direction = other_direction / other_direction.norm();
        Vector3d plane_point1 = circle_center + other_direction * obs_rad;
        Vector3d plane_point3 = circle_center - other_direction * obs_rad;

        Vector3d line1 = plane_point3 - plane_point1;
        Vector3d line2 = plane_point3 - plane_point2;

        Vector3d plane_n_circle = line1.cross(line2);
        plane_n_circle = plane_n_circle / plane_n_circle.norm();

        double d_pil = plane_point1.dot(plane_n_circle);
        Vector3d robot_obs = (robot_pos.dot(plane_n_circle) - d_pil) * plane_n_circle;


        // ***************************************** check intersection point.  ***************************************** //
        double cos_theta = (-robot_obs).dot(line_goal_robot) / robot_obs.norm();
        Vector3d intersection_point = line_goal_robot * robot_obs.norm() /cos_theta + robot_pos;
        if ((intersection_point - circle_center).norm() < obs_rad)
        {
          Vector3d obs_center = (robot_pos - circle_center) / (robot_pos - circle_center).norm();
          Vector3d robot_direction = (cylinder_axis.cross(obs_center)).cross(cylinder_axis);
          Vector3d vertex = circle_center + obs_rad * robot_direction;
          Vector3d parallel_direction = (vertex - circle_center)/(vertex - circle_center).norm();

          cylin_force = k_cylinder / dist_obs.norm() * parallel_direction;
        }
      }
    }
  }
  if (min_dist > dist_obs.norm())
  {
          min_dist = dist_obs.norm();
  }
  ave_dist = ave_dist + dist_obs.norm();
  ave_num = ave_num + 1;
  return cylin_force;

}