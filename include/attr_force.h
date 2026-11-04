#ifndef ATTRFORCE_H
#define ATTRFORCE_H
#include <iostream>
#include <Eigen/Dense>
#include <memory>

using namespace Eigen;


Vector3d attr_force(const Vector3d &goal, const Vector3d &robot_pos, const double &k_attr);


#endif