#ifndef MinMax_H
#define MinMax_H
#include <iostream>
#include <Eigen/Dense>
#include <memory>



using namespace Eigen;

Eigen::VectorXd findAllMinIndices(const Eigen::VectorXd &eigenVec) ;
Eigen::VectorXd findAllMaxIndices(const Eigen::VectorXd &eigenVec) ;
#endif