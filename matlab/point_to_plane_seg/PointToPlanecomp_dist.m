function dist = PointToPlanecomp_dist(robot_pos, obs_point1, obs_point2, obs_point3, obs_point4, goal)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
line_normal = goal - robot_pos;
line_normal = line_normal/norm(line_normal);

robot_obs = PTPdist_pos_obs(robot_pos, obs_point1, obs_point2, obs_point3, obs_point4);

dist = robot_obs - (line_normal * dot(robot_obs, line_normal));


if norm(dist) < 1e-10
    dist = -1 + 2 * rand(size(obs_point1));
    %dist = -robot_obs;
end


