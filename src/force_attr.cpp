#include "../include/attr_force.h"

Vector3d attr_force(const Vector3d &goal, const Vector3d &robot_pos,
                    const double &k_attr) { 
    return (goal - robot_pos) / (goal - robot_pos).norm() * k_attr;
}
