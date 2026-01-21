#ifndef ATTRFORCE_H
#define ATTRFORCE_H
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <dqrobotics/DQ.h>
#include <memory>

using namespace Eigen;
using namespace DQ_robotics;


Vector3d attr_force(const Vector3d &goal, const Vector3d &robot_pos, const double &k_attr);


#endif