function dist = comp_dist(robot_pos, obs_pos, goal, obs_rad)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
line_normal = goal - robot_pos;
line_normal = line_normal/norm(line_normal);
robot_obs = (norm(robot_pos - obs_pos) - obs_rad) * (robot_pos - obs_pos)/norm(robot_pos - obs_pos);
% line_normal * dot(robot_obs, line_normal)
robot_obs = robot_obs/norm(robot_obs);
dist = robot_obs - (line_normal * dot(robot_obs, line_normal));
if norm(dist) < 1e-10
    dist = -1 + 2 * rand(size(obs_pos));
end

