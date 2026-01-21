function dist_obs = PTCdist_pos_obs(robot_pos, obs_point1, obs_point2, obs_point3, obs_point4)
% line 1
dist1 = PTLdist_pos_obs(robot_pos, obs_point1, obs_point2);

% line 2
dist2 = PTLdist_pos_obs(robot_pos, obs_point2, obs_point3);

% line 3
dist3 = PTLdist_pos_obs(robot_pos, obs_point3, obs_point4);

% line 4
dist4 = PTLdist_pos_obs(robot_pos, obs_point4, obs_point1);

% point to plane distance

% Build the plane
line1 = obs_point4 - obs_point2;
line2 = obs_point4 - obs_point3;
plane_n = cross(line1, line2);
plane_n = plane_n/norm(plane_n);
n_pi1 = DQ(plane_n);
p_pi1 = DQ(obs_point1);
d_pi1 = dot(p_pi1,n_pi1);

% Calculate distance
d_p_pi1 = (dot(DQ(robot_pos),n_pi1)-d_pi1);
d_p_pi1 = d_p_pi1.q(1);
sig = sign(dot(robot_pos-obs_point1, plane_n));
dist5 = d_p_pi1 *plane_n;

dist = [dist1; dist2; dist3; dist4; dist5];
dist_norm = [norm(dist1), norm(dist2), norm(dist3), norm(dist4), norm(dist5)];

dist_index = find(dist_norm == min(dist_norm));

dist_obs = dist(dist_index(1),:);

end

