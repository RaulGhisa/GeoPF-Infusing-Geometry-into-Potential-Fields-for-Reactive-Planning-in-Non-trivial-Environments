function dist = PointToLinecomp_dist(robot_pos, obs_start, obs_end, goal)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
line_normal = goal - robot_pos

line_normal = line_normal/norm(line_normal);

robot_obs = PTLdist_pos_obs(robot_pos, obs_start, obs_end);
robot_obs = robot_obs/norm(robot_obs)
dist = zeros(size(robot_obs));
for j = 1:size(robot_obs,1)
    dist(j,:) = robot_obs(j,:) - (line_normal * dot(robot_obs(j,:), line_normal));
    %if norm(dist(j,:)) < 1e-10
    %    dist(j,:) = -1 + 2 * rand(size(obs_start));
    %end
end
