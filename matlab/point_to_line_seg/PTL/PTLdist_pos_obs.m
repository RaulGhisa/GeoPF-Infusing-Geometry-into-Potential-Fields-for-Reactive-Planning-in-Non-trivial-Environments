function dist_obs = PTLdist_pos_obs(robot_pos, obs_start, obs_end)
% robot line
n_l1 = norm(DQ(obs_start - obs_end));
l1= DQ(obs_start - obs_end)/ n_l1;
p_l1 = DQ(obs_start);
m1 = cross(p_l1,l1);
%obs_line = l1 + E_*m1;
obs_start = obs_start;
obs_end = obs_end;
% robot_pos to Line distance
rob_obs_start = robot_pos - obs_start;
rob_obs_start = rob_obs_start/norm(rob_obs_start);
robot_line = cross(cross(vec3(l1), rob_obs_start), vec3(l1));
robot_line = robot_line/norm(robot_line);
norm333 = norm(cross(DQ(robot_pos), DQ(l1))-m1);
cross_pdo = cross(DQ(robot_pos), DQ(l1));
line_direction = l1;
robot_pos = robot_pos;
dist1 = vec3(norm(cross(DQ(robot_pos), DQ(l1))-m1) * DQ((robot_line)))';

% robot_pos to obs_start distance
dist2 = robot_pos - obs_start;


% robot_pos to obs_end distance
dist3 = robot_pos - obs_end;

%dist = [dist1; dist2; dist3];


obs_point = [obs_start; obs_end];
pp_root =  robot_pos - dist1;

if min(obs_point(:,1))-1.e-8  <= pp_root(1) && pp_root(1) <= max(obs_point(:,1)) + 1.e-8 ...
    && min(obs_point(:,2))-1.e-8 <= pp_root(2) && pp_root(2) <= max(obs_point(:,2)) + 1.e-8 ...
    && min(obs_point(:,3))-1.e-8  <= pp_root(3) && pp_root(3) <= max(obs_point(:,3)) + 1.e-8
    dist_obs = [dist1];
else
    dist = [dist2; dist3];
    dist_norm = [norm(dist2), norm(dist3)];
    dist_index = find(dist_norm == min(dist_norm));
    dist_obs = dist(dist_index(1),:);
end

