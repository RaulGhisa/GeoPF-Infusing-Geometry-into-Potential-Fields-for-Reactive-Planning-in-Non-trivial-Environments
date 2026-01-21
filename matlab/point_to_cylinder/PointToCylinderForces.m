
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function pont_to_cylinder_force = PointToCylinderForces(obs_start, obs_end, obs_rad, robot_pos, robot_vel, goal, k_line)

% Function that computes the circular forces
line_dire = obs_start- obs_end;
line_dire = line_dire/norm(line_dire);
%obs_start = line_dire * 0.03 + obs_start;
%obs_end = -line_dire * 0.03 + obs_end;
dist_obs = PTYdist_pos_obs(robot_pos, obs_start, obs_end, obs_rad);
dist_obs = norm(dist_obs(1:3));
pont_to_cylinder_force = zeros(1,3);

% define how much distance should consider force
if dist_obs < 0.05
   if norm(robot_vel) ~= 0
       dist = PointToCylindercomp_dist(robot_pos, obs_start, obs_end, obs_rad, goal);
       dist = dist/norm(dist);
       %pont_to_cylinder_force = (k_line / dist_obs) * cross(robot_vel, cross(dist,robot_vel));
       pont_to_cylinder_force = (k_line / dist_obs) * dist;%cross(robot_vel, cross(dist,robot_vel));
   end
end
end