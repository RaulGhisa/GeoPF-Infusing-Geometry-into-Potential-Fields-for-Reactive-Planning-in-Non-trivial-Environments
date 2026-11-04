#ifndef CIRCFORCE_H
#define CIRCFORCE_H
#include <iostream>
#include <Eigen/Dense>
#include <memory>
#include "../include/IndexMinMax.h"



using namespace Eigen;

extern double min_dist;
extern double ave_dist;
extern double ave_num;

Vector3d sphere_force(const Vector3d &obs_pos, const double &obs_rad, const Vector3d &robot_pos, 
                const Vector3d &robot_vel, const Vector3d &goal, const double &k_circle);

Vector3d boundary_force(const Vector3d &obs_pos, const double &obs_rad, const Vector3d &robot_pos, 
                const Vector3d &robot_vel, const Vector3d &goal, const double &k_circle);

Vector3d line_force(const Vector3d &obs_start, const Vector3d &obs_end, const Vector3d &robot_pos, 
                const Vector3d &robot_vel, const Vector3d &goal, const double &k_line);

Vector3d line_dist_obs(const Vector3d &obs_start, const Vector3d &obs_end, const Vector3d &robot_pos);

Vector3d plane_force(const Vector3d &obs1, const Vector3d &obs2, const Vector3d &obs3, const Vector3d &obs4, const Vector3d &robot_pos, const Vector3d &robot_vel,
                    const Vector3d &goal, const double &k_plane);

Vector4d plane_force_closest_goal(const Vector3d &obs1, const Vector3d &obs2, const Vector3d &obs3, const Vector3d &obs4, const Vector3d &robot_pos, const Vector3d &robot_vel,
                    const Vector3d &goal, const double &k_plane);


Vector3d plane_dist_obs(const Vector3d &obs1, const Vector3d &obs2, const Vector3d &obs3, const Vector3d &obs4, const Vector3d &robot_pos);

int sign(double x);

Vector3d cubic_force(const Vector3d &obs1, const Vector3d &obs2, const Vector3d &obs3, const Vector3d &obs4, 
                    const Vector3d &obs5, const Vector3d &obs6, const Vector3d &obs7, const Vector3d &obs8, 
                    const Vector3d &robot_pos, const Vector3d &robot_vel,
                    const Vector3d &goal, const double &k_plane);

Vector3d cylinder_force(const Vector3d &obs_start, const Vector3d &obs_end, const double &obs_rad, 
                        const Vector3d &robot_pos, const Vector3d &robot_vel,
                        const Vector3d &goal, const double &k_cylinder);
                        
#endif