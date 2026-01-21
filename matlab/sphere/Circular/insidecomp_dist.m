function dist = insidecomp_dist(robot_pos, obs_pos, obs_rad, goal)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
line_normal = goal - robot_pos;
line_normal = line_normal/norm(line_normal);
robot_obs = robot_pos - obs_pos;
% line_normal * dot(robot_obs, line_normal)
dist = robot_obs - (line_normal * dot(robot_obs, line_normal));
dist_direction = dist / norm(dist);

dist_value = obs_rad - norm(dist);
dist = dist_value * dist_direction;
if norm(dist) < 1e-10
    dist = -1 + 2 * rand(size(obs_pos));
end

