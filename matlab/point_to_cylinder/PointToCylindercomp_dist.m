function dist = PointToCylindercomp_dist(robot_pos, obs_start, obs_end, obs_rad, goal)


line_normal = goal - robot_pos;

line_normal = line_normal/norm(line_normal);

robot_obs_ortho = PTYdist_pos_obs(robot_pos, obs_start, obs_end, obs_rad);
robot_obs = robot_obs_ortho(1:3);
robot_ortho = robot_obs_ortho(4);
dist = zeros(size(robot_obs));
for j = 1:size(robot_obs,1)
    dist(j,:) = robot_obs(j,:) - (line_normal * dot(robot_obs(j,:), line_normal));
    if norm(dist(j,:)) < 1e-10
        dist(j,:) = -1 + 2 * rand(size(obs_start));
    end
end



if 0.9 < robot_ortho && robot_ortho < 1.1
    % check intersection root
    obs_center = robot_obs_ortho(5:7);
    robot_line = robot_obs_ortho(8:10);

    obs_point2 = obs_center + obs_rad * robot_line;
    n_l1 = norm(DQ(obs_start - obs_end));
    l1= DQ(obs_start - obs_end)/ n_l1;
    other_dir = cross(robot_line, vec3(l1));
    other_dir = other_dir / norm(other_dir);
    obs_point1 = obs_center + other_dir * obs_rad;
    obs_point3 = obs_center - other_dir * obs_rad;

    line1 = obs_point3 - obs_point1;
    line2 = obs_point3 - obs_point2;
    plane_n = cross(line1, line2);
    plane_n = plane_n/norm(plane_n);
    n_pi1 = DQ(plane_n);
    p_pi1 = DQ(obs_point1);
    d_pi1 = dot(p_pi1,n_pi1);
    
    % Calculate distance
    d_p_pi1 = (dot(DQ(robot_pos),n_pi1)-d_pi1);
    d_p_pi1 = d_p_pi1.q(1);
    %sig = sign(dot(robot_pos-obs_point1, plane_n));
    robot_obs = d_p_pi1 * plane_n;
       
    % robot_obs = PTPdist_pos_obs(robot_pos, obs_point1, obs_point2, obs_point3, obs_point4);
    % robot_obs_direct = robot_obs/norm(robot_obs);

   robot_goal = goal - robot_pos;
   
   cos_theta = dot(-robot_obs, robot_goal) / (norm(robot_goal)*norm(robot_obs));
   robot_intersect = norm(robot_obs)/cos_theta;
   intersect = robot_goal/norm(robot_goal) * robot_intersect + robot_pos;

   if norm( intersect - obs_center) < obs_rad
       n_l1 = norm(DQ(obs_start - obs_end));
        l1= DQ(obs_start - obs_end)/ n_l1;
        p_l1 = DQ(obs_start);
        m1 = cross(p_l1,l1);
        %obs_line = l1 + E_*m1;
        
        % robot_pos to Line distance
        rob_obs_start = robot_pos - obs_center;
        rob_obs_start = rob_obs_start/norm(rob_obs_start);
        
        robot_line = cross(cross(vec3(l1), rob_obs_start), vec3(l1));
        robot_line = robot_line/norm(robot_line);

        vertex = obs_center + vec3((obs_rad) * DQ((robot_line)))';
       parallel_direction = vertex - obs_center;
       %att_force = AttractorForces(goal, robot_pos, 0.1);
       dist = parallel_direction/norm(parallel_direction);    
   end
end
