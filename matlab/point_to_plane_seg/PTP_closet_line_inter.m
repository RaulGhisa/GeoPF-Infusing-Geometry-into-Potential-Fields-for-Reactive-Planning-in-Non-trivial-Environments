function point = PTP_closet_line_inter(robot_pos, obs_point1, obs_point2, obs_point3, obs_point4)
% line 1
dist1 = PTLdist_pos_obs(robot_pos, obs_point1, obs_point2);

% line 2
dist2 = PTLdist_pos_obs(robot_pos, obs_point2, obs_point3);

% line 3
dist3 = PTLdist_pos_obs(robot_pos, obs_point3, obs_point4);

% line 4
dist4 = PTLdist_pos_obs(robot_pos, obs_point4, obs_point1);



dist = [dist1; dist2; dist3; dist4];
dist_norm = [norm(dist1), norm(dist2), norm(dist3), norm(dist4)];

dist_index = find(dist_norm == min(dist_norm));
%s_dist_index = size(dist_index);
%if s_dist_index(2) ~= 1
 %   error("Minimum distance not exist")
%end

point = robot_pos - dist(dist_index(1),:);

end

