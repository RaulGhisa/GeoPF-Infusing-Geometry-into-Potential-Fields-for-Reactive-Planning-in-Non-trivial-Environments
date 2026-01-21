function dist_obs_ortho = PTYdist_pos_obs(robot_pos, obs_start, obs_end, obs_rad)
% robot line
n_l1 = norm(DQ(obs_start - obs_end));
l1= DQ(obs_start - obs_end)/ n_l1;
p_l1 = DQ(obs_start);
m1 = cross(p_l1,l1);
%obs_line = l1 + E_*m1;

% robot_pos to Line distance
rob_obs_start = robot_pos - obs_start;
rob_obs_start = rob_obs_start/norm(rob_obs_start);

    robot_line = cross(cross(vec3(l1), rob_obs_start), vec3(l1));
    robot_line = robot_line/norm(robot_line);
    % bring the line forward to the surface of the cylinder: -obs_rad
    dist1 = vec3((norm(cross(DQ(robot_pos), DQ(l1))-m1)-obs_rad) * DQ((robot_line)))';
    
    
    % the edge of the top or bottom plane
    % robot_pos to obs_start distance, bringing forward
    start_edge = obs_start + vec3((obs_rad) * DQ((robot_line)))';
    dist2 = robot_pos - start_edge;
    
    
    % robot_pos to obs_end distance, bringing forward
    end_edge = obs_end + vec3((obs_rad) * DQ((robot_line)))';
    dist3 = robot_pos - end_edge;
    
    %dist = [dist1; dist2; dist3];
    
    
    obs_point = [start_edge; end_edge];
    pp_root =  robot_pos - dist1;
    
    if min(obs_point(:,1))-1.e-8  <= pp_root(1) && pp_root(1) <= max(obs_point(:,1)) + 1.e-8 ...
        && min(obs_point(:,2))-1.e-8 <= pp_root(2) && pp_root(2) <= max(obs_point(:,2)) + 1.e-8 ...
        && min(obs_point(:,3))-1.e-8  <= pp_root(3) && pp_root(3) <= max(obs_point(:,3)) + 1.e-8
        % bring the line forward to the surface of the cylinder: -obs_rad
        dist_obs = dist1;
        dist_obs_ortho = [dist_obs, 0];
    else
        % robot to line point distance
        if norm(dot(vec3(l1)', rob_obs_start)) < 0.99
            dist = [dist2; dist3];
            dist_norm = [norm(dist2), norm(dist3)];
            dist_index = find(dist_norm == min(dist_norm));
            dist_obs_point = dist(dist_index(1),:);
            
            % robot to plane orthogonal distance
            if dist_index == 1
                obs_center = obs_start;
                plane_n = obs_start - obs_end;
                plane_n = plane_n/norm(plane_n);
            else 
                obs_center = obs_end;
                plane_n = obs_end - obs_start;
                plane_n = plane_n/norm(plane_n);
            end
        
            % Build the plane
            %obs_point2 = obs_center + obs_rad * robot_line;
            %other_dir = cross(robot_line, vec3(l1));
            %other_dir = other_dir / norm(other_dir);
            %obs_point1 = obs_center + other_dir * obs_rad;
            %obs_point3 = obs_center - other_dir * obs_rad;
        
            %line1 = obs_point3 - obs_point1;
            %line2 = obs_point3 - obs_point2;
            %plane_n = cross(line1, line2);
            %plane_n = plane_n/norm(plane_n);
            n_pi1 = DQ(plane_n);
            p_pi1 = DQ(obs_center);
            d_pi1 = dot(p_pi1,n_pi1);
            
            % Calculate distance orthogonal to circular plane
            d_p_pi1 = (dot(DQ(robot_pos),n_pi1)-d_pi1);
            d_p_pi1 = d_p_pi1.q(1);
            %sig = sign(dot(robot_pos-obs_point1, plane_n));
            dist_bot = d_p_pi1 *plane_n;
        
            % calculate the pp root. 
             pp_root =  robot_pos - dist_bot; 
             if norm(pp_root - obs_center) < obs_rad
                 dist_obs = dist_bot;
             else
                 dist_obs = dist_obs_point;
             end
        
        
             dist_obs_ortho = [dist_obs, 1, obs_center, robot_line];

         else   
            % the edge of the top or bottom plane
            % robot_pos to obs_start distance, bringing forward
            dist2 = robot_pos -  obs_start;
            
            
            % robot_pos to obs_end distance, bringing forward
            dist3 = robot_pos -  obs_end;
            
            dist = [dist2; dist3];
            dist_norm = [norm(dist2), norm(dist3)];
            dist_index = find(dist_norm == min(dist_norm));
            dist_obs_point = dist(dist_index(1),:);
            if dist_index == 1
                obs_center = obs_start;
            else 
                obs_center = obs_end;
            end
        
            dist_obs_ortho = [dist_obs_point, 1, obs_center, robot_line];%dist_obs_point/norm(dist_obs_point)];

        end
    end
end


