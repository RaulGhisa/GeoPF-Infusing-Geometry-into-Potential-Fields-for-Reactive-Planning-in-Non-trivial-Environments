#ifndef SINGLE_AGENT_H
#define SINGLE_AGENT_H
#include <iostream>
#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
#include <dqrobotics/robot_control/DQ_PseudoinverseController.h>
#include <thread>
#include "../include/attr_force.h"
#include "../include/geometry_force.h"
#include "../include/convert_matrix2csv.h"
#include "../include/convert_csv2matrix.h"

#include "../include/IndexMinMax.h"

using namespace DQ_robotics;

extern double min_dist;
extern double ave_dist;
extern double ave_num;

int main(void);

#endif