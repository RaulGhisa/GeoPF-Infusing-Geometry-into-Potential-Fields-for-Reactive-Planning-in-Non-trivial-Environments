
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function curr_force = CurrForces(obs_pos, obs_rad, robot_pos, robot_vel, goal, k_circ)

% Function that computes the circular forces

dist_obs = norm(robot_pos - obs_pos) - obs_rad;
curr_force = zeros(1,3);
if dist_obs < 0.05
   if norm(robot_vel) ~= 0
       dist = comp_dist(robot_pos, obs_pos, goal, obs_rad);
       dist = dist/norm(dist);
       % curr_force = (k_circ / dist_obs) * cross(robot_vel, cross(dist,robot_vel));
       curr_force = (k_circ / dist_obs) * dist;
   end
end
end