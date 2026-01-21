#include "../include/geometry_force.h"
#include <random>
#include "../include/SingleAgent.h"



Vector3d sphere_force(const Vector3d &obs_pos, const double &obs_rad, const Vector3d &robot_pos, 
                const Vector3d &robot_vel, const Vector3d &goal, const double &k_circle){
        
        Vector3d line_goal_robot = (goal - robot_pos) / (goal - robot_pos).norm();
        double dist_obs = (robot_pos - obs_pos).norm() - obs_rad;

        Vector3d circ_force(0,0,0);

        
        if (dist_obs < 0.05) {

                if (robot_vel.norm() != 0 ) {

                        Vector3d line_robot_obs = (robot_pos - obs_pos) / (robot_pos - obs_pos).norm();
                        Vector3d force_direction = line_robot_obs - (line_goal_robot * (line_robot_obs.transpose() * line_goal_robot));
                        force_direction = force_direction / force_direction.norm();

                        if (dist_obs < 0) {

                                std::random_device rd;  // Random seed
                                std::mt19937 gen(rd()); // Mersenne Twister engine
                                std::uniform_real_distribution<> dis(-1.0, 1.0);
                                for (int i = 0; i < obs_pos.size(); ++i) {
                                        
                                        force_direction(i) = dis(gen);
                                }
                                Vector3d minus_one;
                                minus_one.setConstant(-1);
                                force_direction = minus_one + 2 * force_direction;
                                force_direction = force_direction / force_direction.norm();
                        }
                        circ_force = k_circle / dist_obs * force_direction;
                }      
        } 
        if (min_dist > dist_obs)
        {
                min_dist = dist_obs;
        }

        ave_dist = ave_dist + dist_obs;
        ave_num = ave_num + 1;
        
        return circ_force;
}


