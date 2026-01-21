
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function pont_to_lineSeg_force = PointToLineSegForces(obs_start, obs_end, robot_pos, robot_vel, goal, k_line)

% Function that computes the circular forces

dist_obs = norm(PTLdist_pos_obs(robot_pos, obs_start, obs_end));
pont_to_lineSeg_force = zeros(1,3);

% define how much distance should consider force
    if dist_obs < 0.05
       if norm(robot_vel) ~= 0
           dist = PointToLinecomp_dist(robot_pos, obs_start, obs_end, goal);
           dist = dist/norm(dist);
           % pont_to_lineSeg_force = (k_line / dist_obs) * cross(robot_vel, cross(dist,robot_vel));
           pont_to_lineSeg_force = (k_line / dist_obs) * dist;
       end
    end
end