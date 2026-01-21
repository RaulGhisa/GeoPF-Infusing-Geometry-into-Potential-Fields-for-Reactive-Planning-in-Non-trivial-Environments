#ifndef MinMax_H
#define MinMax_H
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <dqrobotics/DQ.h>
#include <memory>



using namespace Eigen;
using namespace DQ_robotics;

Eigen::VectorXd findAllMinIndices(const Eigen::VectorXd &eigenVec) ;
Eigen::VectorXd findAllMaxIndices(const Eigen::VectorXd &eigenVec) ;
#endif